# Workshop 1 barrel node - Pico 2 W, MicroPython, MQTT with Home Assistant discovery
# Barrel level + pump control (MOSFET on GP15), with settings controlled from Home Assistant.
# The pump output (GP15) is ENABLED in this version. Mode starts as Off; the safety rules
# below apply in every mode.
# Mains-powered node. Tanks can use a 4-20 mA submersible probe ("loop")
# or a variable-resistance float sender ("float"). Add tanks as config entries.
#
# One-time setup on the Pico (REPL, once Wi-Fi is up):
#   import mip; mip.install("umqtt.simple")

import network, time, json, machine, ubinascii, ssl
from machine import ADC, WDT, Pin
from umqtt.simple import MQTTClient
import ota
import secrets

# umqtt.simple sizes packets with len(), which counts characters, not bytes. Any non-ASCII
# character (e.g. the "°" in °C) then makes the packet length wrong and the broker drops
# the connection. Always hand it bytes so the length is exact.
_umqtt_publish = MQTTClient.publish
def _publish_bytes(self, topic, msg, retain=False, qos=0):
    if isinstance(topic, str):
        topic = topic.encode()
    if isinstance(msg, str):
        msg = msg.encode()
    return _umqtt_publish(self, topic, msg, retain, qos)
MQTTClient.publish = _publish_bytes

VERSION = "workshop1-barrel-2026-11-15e"          # bump this in every new main.py you publish

# ---------------- CONFIG ----------------
# Wi-Fi networks and MQTT login live in secrets.py on the Pico, so updating
# this script never overwrites them.
from secrets import WIFI_NETWORKS, MQTT_USER, MQTT_PASS
MQTT_HOST = "91671bbee0974c8388ffbf5b0ce11c52.s1.eu.hivemq.cloud"   # HiveMQ Cloud (same as HA)
MQTT_PORT = 8883                 # TLS
NODE_ID   = "workshop1_barrel"   # MQTT topics + HA ids - separate from the storage node
DEVICE_NAME = "Workshop 1 barrel"
UPDATE_PREFIX = "workshop1-barrel-"   # only install updates whose version starts with this
PUBLISH_S = 30                   # must stay below the 60 s MQTT keepalive
ADC_VREF  = 3.30                 # measure your Pico's 3V3 pin and put the real value here

TANKS = [
    {
        # TL-136 0-1 m probe in the 220 L blue barrel.
        # Wiring: +24V -> probe red; probe black -> GP26 via 1k,
        #         and probe black -> 150R (0.1%) -> GND. 100nF GP26 -> GND.
        "id": "barrel",
        "name": "",                # entity names come out as "Level", "Volume", ... under the device name
        "type": "loop",
        "adc_pin": 26,
        "shunt_ohms": 150.0,
        "range_mm": 1000,          # probe full scale (0-1 m)
        "probe_offset_mm": 0,      # probe sensing face height above barrel floor - set when installed
        "litres_per_mm": 0.26,     # standard 220 L drum, ~58 cm inside diameter
        "capacity_l": 220,
        # Two-point calibration - fill in after a bucket test, like the storage probe.
        # Until then the nominal 4-20 mA = 0-1000 mm is used.
        # "cal_zero_ma": 4.00,     # reading with probe dry / zero depth
        # "cal_ref_ma": 8.00,      # reading at a known depth
        # "cal_ref_mm": 250,       # that known depth, tip of probe to water surface
    },
    # Float sender example (resistance divider: 3V3 -> r_ref -> pin -> sender -> GND)
    # {"id": "gather1", "name": "Gathering tank 1", "type": "float", "adc_pin": 27,
    #  "r_ref": 220, "capacity_l": 1000, "cal": [(10, 1000), (100, 500), (180, 0)]},
]

# ---- Pump ----
PUMP_PIN            = 15        # GP15 -> MOSFET PWM input (10k pull-down to GND holds it off)
PUMP_OUTPUT_ENABLED = True      # GP15 is driven - the pump WILL run once the MOSFET and pump are wired
STORAGE_TOPIC       = "rainwater/storage/state"   # main storage level, published by the workshop2 Pico
STORAGE_STALE_S     = 300       # storage reading older than this = unknown
TIMEOUT_COOLDOWN_S  = 1800      # after hitting max run time, Auto waits this long before retrying
STOP_PAUSE_S        = 1800      # after the Stop button, Auto waits this long before starting again
CONTROL_S           = 2         # how often the barrel is measured for pump control
SETTINGS_FILE       = "settings.json"
MODES               = ("Auto", "Off", "Manual")
DEFAULT_SETTINGS    = {"mode": "Off", "start_pct": 80, "stop_pct": 20,
                       "storage_full_pct": 95, "max_run_min": 15}
LIMITS              = {"start_pct": (10, 100), "stop_pct": (5, 90),
                       "storage_full_pct": (50, 100), "max_run_min": (1, 60)}
# ----------------------------------------

UID = ubinascii.hexlify(machine.unique_id()).decode()
BASE = NODE_ID
AVAIL = f"{BASE}/status"
network.country("GB")            # allow UK 2.4 GHz channels 12-13
wlan = network.WLAN(network.STA_IF)

def wifi_connect(wdt=None):
    wlan.active(True)
    wlan.config(pm=0xa11140)       # no power-save: fewer drop-outs on mains power
    if not wlan.isconnected():
        known = dict(WIFI_NETWORKS)
        try:
            seen = sorted(wlan.scan(), key=lambda n: n[3], reverse=True)  # strongest first
        except OSError:
            seen = []
        if wdt: wdt.feed()
        ssid = next((n[0].decode() for n in seen if n[0].decode() in known),
                    WIFI_NETWORKS[0][0])
        print("Wi-Fi: joining", ssid)
        wlan.connect(ssid, known[ssid])
        for _ in range(30):
            if wlan.isconnected():
                break
            if wdt: wdt.feed()
            time.sleep(0.5)
        print("Wi-Fi:", "connected" if wlan.isconnected() else "FAILED", wlan.ifconfig()[0],
              "status", wlan.status())
    return wlan.isconnected()

def read_raw(adc, samples=64):
    vals = sorted(adc.read_u16() for _ in range(samples))
    q = samples // 4
    return sum(vals[q:-q]) / (samples - 2 * q)       # trimmed mean

def interp(cal, x):
    pts = sorted(cal)
    if x <= pts[0][0]:  return pts[0][1]
    if x >= pts[-1][0]: return pts[-1][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

def measure(t, adc):
    """Returns dict of readings; values None on sensor fault."""
    raw = read_raw(adc)
    if t["type"] == "loop":
        ma = raw / 65535 * ADC_VREF / t["shunt_ohms"] * 1000
        if ma < 3.5 or ma > 21.0:                    # broken wire / probe fault
            return {"litres": None, "percent": None, "depth_mm": None,
                    "diag": round(ma, 2), "fault": "ON"}
        if "cal_ref_mm" in t:                        # two-point calibration
            z, r = t["cal_zero_ma"], t["cal_ref_ma"]
            above = (ma - z) / (r - z) * t["cal_ref_mm"]
        else:                                        # nominal 4-20 mA over range_mm
            above = (ma - 4) / 16 * t["range_mm"]
        raw_depth = above + t["probe_offset_mm"]     # unclamped, can go negative (diagnostic)
        depth = max(0.0, above) + t["probe_offset_mm"]
        litres = min(depth * t["litres_per_mm"], t["capacity_l"])
        diag = round(ma, 2)
    else:
        if raw >= 65000:
            return {"litres": None, "percent": None, "depth_mm": None,
                    "diag": None, "fault": "ON"}
        ohms = t["r_ref"] * raw / (65535 - raw)
        litres = interp(t["cal"], ohms)
        depth = raw_depth = None
        diag = round(ohms, 1)
    return {"litres": round(litres),
            "percent": round(100 * litres / t["capacity_l"], 1),
            "depth_mm": None if depth is None else round(depth),
            "depth_raw_mm": None if raw_depth is None else round(raw_depth, 1),
            "diag": diag, "fault": "OFF"}

_temp_adc = ADC(4)                     # RP2350 on-chip temperature sensor
def read_temp_c():
    v = read_raw(_temp_adc) / 65535 * ADC_VREF
    return round(27 - (v - 0.706) / 0.001721, 1)

def publish_discovery(c):
    dev = {"identifiers": [f"{NODE_ID}_{UID}"], "name": DEVICE_NAME,
           "manufacturer": "DIY", "model": "Pico 2 W"}
    base_cfg = {"availability_topic": AVAIL, "device": dev}
    for t in TANKS:
        nm = lambda s: ((t["name"] + " " + s).strip()[:1].upper() + (t["name"] + " " + s).strip()[1:])
        state = f"{BASE}/{t['id']}/state"
        diag_unit = "mA" if t["type"] == "loop" else "Ω"
        sensors = [
            ("litres",   nm("volume"), "L",  {"device_class": "volume_storage", "state_class": "measurement"}),
            ("percent",  nm("level"),  "%",  {"state_class": "measurement"}),
            ("diag",     nm("sensor signal"), diag_unit, {"entity_category": "diagnostic", "state_class": "measurement"}),
        ]
        if t["type"] == "loop":
            sensors.append(("depth_mm", nm("depth"), "mm",
                            {"device_class": "distance", "state_class": "measurement"}))
            sensors.append(("depth_raw_mm", nm("raw depth"), "mm",
                            {"device_class": "distance", "state_class": "measurement",
                             "entity_category": "diagnostic"}))
        for key, name, unit, extra in sensors:
            cfg = dict(base_cfg, name=name, unique_id=f"{NODE_ID}_{UID}_{t['id']}_{key}",
                       state_topic=state, value_template=f"{{{{ value_json.{key} }}}}",
                       unit_of_measurement=unit, **extra)
            c.publish(f"homeassistant/sensor/{NODE_ID}_{UID}/{t['id']}_{key}/config",
                      json.dumps(cfg), retain=True)
        fault = dict(base_cfg, name=nm("sensor fault"),
                     unique_id=f"{NODE_ID}_{UID}_{t['id']}_fault", state_topic=state,
                     value_template="{{ value_json.fault }}", device_class="problem",
                     entity_category="diagnostic")
        c.publish(f"homeassistant/binary_sensor/{NODE_ID}_{UID}/{t['id']}_fault/config",
                  json.dumps(fault), retain=True)
    rssi = dict(base_cfg, name="Wi-Fi signal", unique_id=f"{NODE_ID}_{UID}_rssi",
                state_topic=f"{BASE}/rssi", unit_of_measurement="dBm",
                device_class="signal_strength", entity_category="diagnostic",
                state_class="measurement")
    c.publish(f"homeassistant/sensor/{NODE_ID}_{UID}/rssi/config", json.dumps(rssi), retain=True)
    # Remote diagnostics - so connection problems can be seen without a USB cable
    for key, name, extra in (
        ("last_error",  "Last error",   {"icon": "mdi:alert-circle-outline"}),
        ("reset_cause", "Reset cause",  {"icon": "mdi:restart"}),
        ("uptime_min",  "Uptime",       {"unit_of_measurement": "min", "state_class": "measurement",
                                         "icon": "mdi:timer-outline"}),
        ("reconnects",  "Reconnects",   {"state_class": "measurement", "icon": "mdi:connection"}),
    ):
        cfg = dict(base_cfg, name=name, unique_id=f"{NODE_ID}_{UID}_{key}", state_topic=DIAG_TOPIC,
                   value_template=f"{{{{ value_json.{key} }}}}", entity_category="diagnostic", **extra)
        c.publish(f"homeassistant/sensor/{NODE_ID}_{UID}/{key}/config", json.dumps(cfg), retain=True)
    temp = dict(base_cfg, name="Enclosure temperature", unique_id=f"{NODE_ID}_{UID}_temp",
                state_topic=f"{BASE}/temp", unit_of_measurement="°C",
                device_class="temperature", entity_category="diagnostic",
                state_class="measurement")
    c.publish(f"homeassistant/sensor/{NODE_ID}_{UID}/temp/config", json.dumps(temp), retain=True)
    ver = dict(base_cfg, name="Firmware version", unique_id=f"{NODE_ID}_{UID}_version",
               state_topic=VERSION_TOPIC, entity_category="diagnostic", icon="mdi:chip")
    c.publish(f"homeassistant/sensor/{NODE_ID}_{UID}/version/config", json.dumps(ver), retain=True)
    upd = dict(base_cfg, name="Update status", unique_id=f"{NODE_ID}_{UID}_ota",
               state_topic=OTA_STATUS, entity_category="diagnostic", icon="mdi:update")
    c.publish(f"homeassistant/sensor/{NODE_ID}_{UID}/ota/config", json.dumps(upd), retain=True)
    publish_pump_discovery(c, base_cfg)

def publish_pump_discovery(c, base_cfg):
    """HA controls (mode, levels, buttons) and pump sensors. Controls send to BASE/set/...
       retained, so the Pico gets the latest settings again after a reboot."""
    pre = f"homeassistant/{{}}/{NODE_ID}_{UID}/{{}}/config"
    sel = dict(base_cfg, name="Pump mode", unique_id=f"{NODE_ID}_{UID}_mode", icon="mdi:pump",
               command_topic=f"{BASE}/set/mode", state_topic=SETTINGS_TOPIC,
               value_template="{{ value_json.mode }}", options=list(MODES), retain=True)
    c.publish(pre.format("select", "mode"), json.dumps(sel), retain=True)
    for key, name, unit, icon in (
        ("start_pct",        "Pump start level",      "%",   "mdi:arrow-collapse-up"),
        ("stop_pct",         "Pump stop level",       "%",   "mdi:arrow-collapse-down"),
        ("storage_full_pct", "Storage full cut-off",  "%",   "mdi:water-alert"),
        ("max_run_min",      "Pump max run time",     "min", "mdi:timer-alert-outline"),
    ):
        lo, hi = LIMITS[key]
        num = dict(base_cfg, name=name, unique_id=f"{NODE_ID}_{UID}_{key}", icon=icon,
                   command_topic=f"{BASE}/set/{key}", state_topic=SETTINGS_TOPIC,
                   value_template=f"{{{{ value_json.{key} }}}}", min=lo, max=hi, step=1,
                   unit_of_measurement=unit, mode="box", retain=True, entity_category="config")
        c.publish(pre.format("number", key), json.dumps(num), retain=True)
    for key, name, icon in (("pump_now", "Pump now", "mdi:play"), ("stop", "Stop pump", "mdi:stop")):
        btn = dict(base_cfg, name=name, unique_id=f"{NODE_ID}_{UID}_{key}", icon=icon,
                   command_topic=f"{BASE}/cmd/{key}", payload_press="PRESS")
        c.publish(pre.format("button", key), json.dumps(btn), retain=True)
    run = dict(base_cfg, name="Pump running", unique_id=f"{NODE_ID}_{UID}_pump_on",
               state_topic=PUMP_TOPIC, value_template="{{ value_json.pump }}", device_class="running")
    c.publish(pre.format("binary_sensor", "pump_on"), json.dumps(run), retain=True)
    for key, name, extra in (
        ("status",      "Pump status",          {"icon": "mdi:information-outline"}),
        ("last_run_s",  "Pump last run",        {"unit_of_measurement": "s", "icon": "mdi:timer-outline"}),
        ("runs",        "Pump runs since boot", {"state_class": "measurement", "icon": "mdi:counter",
                                                 "entity_category": "diagnostic"}),
        ("storage_pct", "Storage level seen",   {"unit_of_measurement": "%", "icon": "mdi:water",
                                                 "entity_category": "diagnostic"}),
    ):
        cfg = dict(base_cfg, name=name, unique_id=f"{NODE_ID}_{UID}_pump_{key}", state_topic=PUMP_TOPIC,
                   value_template=f"{{{{ value_json.{key} }}}}", **extra)
        c.publish(pre.format("sensor", "pump_" + key), json.dumps(cfg), retain=True)

STALE_RESET_S   = 600            # reboot if nothing published successfully for 10 min
REDISCOVER_S    = 900            # re-send HA discovery every 15 min (covers HA restarts)
OTA_CHECK_S     = 3600           # check for a newer main.py every hour
OTA_FIRST_S     = 120            # ...and 2 min after each boot
OTA_STATUS      = f"{BASE}/ota/status"
VERSION_TOPIC   = f"{BASE}/version"
DIAG_TOPIC      = f"{BASE}/diag"
ERR_FILE        = "last_error.txt"   # survives reboots, so a crash's cause can be reported after
SETTINGS_TOPIC  = f"{BASE}/settings"
PUMP_TOPIC      = f"{BASE}/pump"
HA_STATUS_TOPIC = "homeassistant/status"

# ---------------- Pump control ----------------
# All safety rules are enforced here on the Pico, whatever HA asks for:
#   - stop at the stop level (dry-run protection), never start at or below it
#   - stop after the max run time
#   - never pump into full storage; Auto also needs a fresh storage reading
#   - stop if the barrel sensor reports a fault
_pump_pin = Pin(PUMP_PIN, Pin.OUT, value=0)

def _now():
    return time.ticks_ms()

def _age_s(t):
    return None if t is None else time.ticks_diff(_now(), t) / 1000

class Pump:
    def __init__(self):
        self.s = dict(DEFAULT_SETTINGS)
        try:
            saved = json.loads(open(SETTINGS_FILE).read())
            for k in self.s:
                if k in saved:
                    self.s[k] = saved[k]
        except (OSError, ValueError):
            pass
        self.on = False
        self.manual_run = False
        self.started = None
        self.manual_req = False
        self.stop_req = False
        self.cooldown_from = None
        self.paused_from = None        # Stop button pressed - Auto paused
        self.storage_pct = None
        self.storage_at = None
        self.last_run_s = 0
        self.runs = 0
        self.status = "Starting"
        self.settings_dirty = True     # publish settings to HA
        self.state_dirty = True        # publish pump state to HA

    def _save(self):
        try:
            with open(SETTINGS_FILE, "w") as f:
                f.write(json.dumps(self.s))
        except OSError:
            pass

    def set(self, key, raw):
        """Apply a setting from HA. Invalid values are rejected and HA is told the real value."""
        try:
            if key == "mode":
                v = raw.strip()
                if v not in MODES:
                    raise ValueError
            else:
                v = int(round(float(raw)))
                lo, hi = LIMITS[key]
                if not lo <= v <= hi:
                    raise ValueError
                start = v if key == "start_pct" else self.s["start_pct"]
                stop = v if key == "stop_pct" else self.s["stop_pct"]
                if stop >= start:
                    raise ValueError
        except (ValueError, KeyError):
            print("Setting rejected:", key, raw)
            self.settings_dirty = True
            return
        if key == "mode":
            self.paused_from = None
        if self.s.get(key) != v:
            self.s[key] = v
            self._save()
            print("Setting:", key, "=", v)
        self.settings_dirty = True

    def storage_msg(self, msg):
        try:
            p = json.loads(msg).get("percent")
        except ValueError:
            return
        if p is None:                  # storage sensor fault - treat as unknown
            self.storage_pct, self.storage_at = None, None
        else:
            self.storage_pct, self.storage_at = float(p), _now()

    def _storage_fresh(self):
        a = _age_s(self.storage_at)
        return a is not None and a < STORAGE_STALE_S

    def _switch(self, on, status, manual=False):
        if on and not self.on:
            self.started, self.manual_run = _now(), manual
            self.runs += 1
        if not on and self.on:
            self.last_run_s = int(_age_s(self.started) or 0)
        self.on = on
        self._set_status(status)

    def _set_status(self, status):
        if status != self.status:
            self.status = status
            self.state_dirty = True
            print("Pump:", status)

    def update(self, barrel):
        """Called every CONTROL_S with the latest barrel reading. Decides on/off."""
        s = self.s
        lvl = barrel.get("percent")
        fault = barrel.get("fault") == "ON" or lvl is None
        fresh = self._storage_fresh()
        full = fresh and self.storage_pct >= s["storage_full_pct"]
        stop_req, self.stop_req = self.stop_req, False
        if self.on:
            run_s = _age_s(self.started)
            if s["mode"] == "Off":
                self._switch(False, "Stopped - mode set to Off")
            elif stop_req:
                self._switch(False, "Stopped by user")
                self.paused_from = _now()
            elif fault:
                self._switch(False, "Stopped - barrel sensor fault")
            elif lvl <= s["stop_pct"]:
                self._switch(False, "Stopped - barrel down to %d%%" % int(s["stop_pct"]))
            elif full:
                self._switch(False, "Stopped - storage full")
            elif not fresh and not self.manual_run:
                self._switch(False, "Stopped - no storage reading")
            elif run_s >= s["max_run_min"] * 60:
                self._switch(False, "Stopped - max run time reached, check pump and hose")
                self.cooldown_from = _now()
            else:
                # run time goes in its own field (run_s) so the status text doesn't change every pass
                self._set_status("Pumping" if PUMP_OUTPUT_ENABLED else "Would pump - output disabled")
        else:
            manual_req, self.manual_req = self.manual_req, False
            if stop_req:
                self.paused_from = _now()               # Stop pressed while idle also pauses Auto
            cooling = (self.cooldown_from is not None
                       and _age_s(self.cooldown_from) < TIMEOUT_COOLDOWN_S)
            paused = (self.paused_from is not None
                      and _age_s(self.paused_from) < STOP_PAUSE_S)
            if s["mode"] == "Off":
                self._set_status("Off")
            elif fault:
                self._set_status("Blocked - barrel sensor fault")
            elif full:
                self._set_status("Blocked - storage full (%d%%)" % int(self.storage_pct))
            elif manual_req:
                if lvl <= s["stop_pct"]:
                    self._set_status("Blocked - barrel below stop level")
                else:
                    self.cooldown_from = self.paused_from = None   # a manual run clears pause/cool-down
                    self._switch(True, "Manual run started", manual=True)
            elif s["mode"] == "Manual":
                self._set_status("Manual - press Pump now")
            elif lvl >= s["start_pct"]:
                if not fresh:
                    self._set_status("Blocked - no storage reading")
                elif paused:
                    self._set_status("Paused by Stop button - Auto resumes in %d min"
                                     % (1 + (STOP_PAUSE_S - _age_s(self.paused_from)) // 60))
                elif cooling:
                    self._set_status("Waiting - cool-down after max run time")
                else:
                    self._switch(True, "Auto run started")
            else:
                self._set_status("Waiting - barrel %d%%, starts at %d%%" % (int(lvl), s["start_pct"]))
        _pump_pin.value(1 if (self.on and PUMP_OUTPUT_ENABLED) else 0)

    def state(self):
        return {"pump": "ON" if self.on else "OFF", "status": self.status,
                "run_s": int(_age_s(self.started) or 0) if self.on else 0,
                "last_run_s": self.last_run_s, "runs": self.runs,
                "storage_pct": None if self.storage_pct is None else round(self.storage_pct, 1)}

pump = Pump()
_rediscover = False
_msg_count = 0

def on_msg(topic, msg):
    global _rediscover, _msg_count
    _msg_count += 1
    t = topic.decode()
    m = msg.decode()
    if t == STORAGE_TOPIC:
        pump.storage_msg(m)
    elif t.startswith(BASE + "/set/"):
        pump.set(t[len(BASE) + 5:], m)
    elif t == BASE + "/cmd/pump_now":
        pump.manual_req = True
    elif t == BASE + "/cmd/stop":
        pump.stop_req = True
    elif t == HA_STATUS_TOPIC and m == "online":
        _rediscover = True

def read_messages(client, wdt):
    """Handle any waiting MQTT messages (non-blocking), up to 10 per pass.
       Over TLS an empty non-blocking read can show up as OSError(-1) - treat that as
       'no message'. A real drop makes the next publish fail and reconnect as normal."""
    for _ in range(10):
        before = _msg_count
        try:
            client.check_msg()
        except OSError as e:
            if e.args[0] != -1:
                raise
            break
        finally:
            client.sock.setblocking(True)
        wdt.feed()
        if _msg_count == before:
            break

def _load_err():
    try:
        return open(ERR_FILE).read()
    except OSError:
        return "none"

def _save_err(text):
    if text != _diag["last_error"]:          # only write flash when it changes
        _diag["last_error"] = text
        try:
            with open(ERR_FILE, "w") as f:
                f.write(text)
        except OSError:
            pass

_causes = {getattr(machine, n, -99): n for n in ("PWRON_RESET", "WDT_RESET", "HARD_RESET", "SOFT_RESET")}
_diag = {"last_error": _load_err(),
         "reset_cause": _causes.get(machine.reset_cause(), str(machine.reset_cause())),
         "reconnects": 0}

def check_update(client, wdt):
    """Pull-based update. secrets.OTA_MANIFEST is a URL to a small JSON file:
       {"version": "2026-11-02a", "url": "https://.../main.py", "sha256": "optional"}
       If its version differs from VERSION, install it (with automatic rollback)."""
    manifest = getattr(secrets, "OTA_MANIFEST", None)
    if not manifest:
        return
    def status(s):
        print("OTA:", s)
        client.publish(OTA_STATUS, s, retain=True)
    try:
        import requests
        token = getattr(secrets, "OTA_TOKEN", None)
        wdt.feed()
        r = requests.get(manifest, headers={"Authorization": "token " + token} if token else {})
        try:
            if r.status_code != 200:
                raise ValueError("manifest HTTP %d" % r.status_code)
            m = r.json()
        finally:
            r.close()
        wdt.feed()
        if not str(m.get("version", "")).startswith(UPDATE_PREFIX):
            status("ignored manifest for another node (" + str(m.get("version")) + ")")
            return
        if m.get("version") == VERSION:
            status("up to date (" + VERSION + ")")
            return
        status("downloading " + str(m.get("version")))
        ota.install(m["url"], m.get("sha256"), token, feed=wdt.feed, status=status)
    except Exception as e:
        status("check failed: " + repr(e) + " - still running " + VERSION)

def mqtt_connect(wdt=None):
    feed = wdt.feed if wdt else (lambda: None)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.verify_mode = ssl.CERT_NONE    # encrypted, but server cert not verified (Pico has no clock/CA store)
    c = MQTTClient(f"{NODE_ID}_{UID}", MQTT_HOST, port=MQTT_PORT, user=MQTT_USER,
                   password=MQTT_PASS, keepalive=60, ssl=ctx)
    c.set_last_will(AVAIL, "offline", retain=True)
    c.set_callback(on_msg)
    c.connect()
    feed()
    for t in (BASE + "/set/#", BASE + "/cmd/#", STORAGE_TOPIC, HA_STATUS_TOPIC):
        c.subscribe(t)
    feed()
    c.publish(AVAIL, "online", retain=True)
    publish_discovery(c)
    feed()
    c.publish(VERSION_TOPIC, VERSION, retain=True)
    pump.settings_dirty = pump.state_dirty = True
    if ota.take_rollback_notice():
        c.publish(OTA_STATUS, "update failed to start - rolled back to " + VERSION, retain=True)
    print("MQTT: connected to", MQTT_HOST, "- discovery sent")
    return c

def main():
    global _rediscover
    print("Version", VERSION)
    print("Starting in 5 s - press Ctrl-C now to get the REPL (watchdog not yet running)")
    time.sleep(5)
    adcs = {t["id"]: ADC(t["adc_pin"]) for t in TANKS}
    boot = time.ticks_ms()
    while not wifi_connect():
        if time.ticks_diff(time.ticks_ms(), boot) > STALE_RESET_S * 1000:
            print("No Wi-Fi for 10 min after boot - rebooting")
            machine.reset()
        time.sleep(5)
    wdt = WDT(timeout=8000)
    client = None
    last = time.ticks_add(time.ticks_ms(), -PUBLISH_S * 1000)
    last_ok = time.ticks_ms()            # last successful publish
    last_disc = time.ticks_ms()          # last discovery sent
    next_ota = time.ticks_add(time.ticks_ms(), OTA_FIRST_S * 1000)
    last_ctl = time.ticks_add(time.ticks_ms(), -CONTROL_S * 1000)
    barrel = {"percent": None, "fault": "ON"}
    tank = TANKS[0]
    mqtt_retry = time.ticks_ms()         # after an error, wait before reconnecting (control keeps running)
    while True:
        wdt.feed()
        # Pump control runs every CONTROL_S whether or not Wi-Fi/MQTT/HA are up
        if time.ticks_diff(time.ticks_ms(), last_ctl) >= CONTROL_S * 1000:
            last_ctl = time.ticks_ms()
            try:
                barrel = measure(tank, adcs[tank["id"]])
                pump.update(barrel)
            except Exception as e:
                _pump_pin.value(0)
                print("Control error:", repr(e))
        # Self-heal: if errors keep looping without a hang, the watchdog never fires - so reboot here
        if time.ticks_diff(time.ticks_ms(), last_ok) > STALE_RESET_S * 1000:
            print("Nothing published for 10 min - rebooting")
            _save_err("self-reboot: nothing published for 10 min")
            machine.reset()
        if time.ticks_diff(time.ticks_ms(), mqtt_retry) < 0:
            time.sleep(0.5)
            continue
        try:
            if not wlan.isconnected():
                client = None
                wifi_connect(wdt)
            if client is None:
                client = mqtt_connect(wdt)
                last_disc = time.ticks_ms()
                _diag["reconnects"] += 1
            read_messages(client, wdt)       # settings / buttons from HA, storage level
            if _rediscover or time.ticks_diff(time.ticks_ms(), last_disc) > REDISCOVER_S * 1000:
                publish_discovery(client)
                _rediscover = False
                last_disc = time.ticks_ms()
            if pump.settings_dirty:
                client.publish(SETTINGS_TOPIC, json.dumps(pump.s), retain=True)
                pump.settings_dirty = False
            # Faster updates while pumping, so HA follows the level closely
            interval = 10 if pump.on else PUBLISH_S
            if pump.state_dirty or time.ticks_diff(time.ticks_ms(), last) >= interval * 1000:
                pump.state_dirty = False
                last = time.ticks_ms()
                reading = json.dumps(barrel)
                client.publish(f"{BASE}/{tank['id']}/state", reading)
                client.publish(PUMP_TOPIC, json.dumps(pump.state()), retain=True)
                print(tank["id"], reading, "|", pump.status)
                client.publish(f"{BASE}/rssi", str(wlan.status("rssi")))
                client.publish(f"{BASE}/temp", str(read_temp_c()))
                # Re-assert "online" every cycle: a stale "offline" from a dropped connection
                # can then only ever last one cycle
                client.publish(AVAIL, "online", retain=True)
                _diag["uptime_min"] = time.ticks_diff(time.ticks_ms(), boot) // 60000
                client.publish(DIAG_TOPIC, json.dumps(_diag), retain=True)
                last_ok = time.ticks_ms()
                if ota.confirm():            # first good publish after an update
                    client.publish(OTA_STATUS, "updated to " + VERSION, retain=True)
            if time.ticks_diff(time.ticks_ms(), next_ota) >= 0 and not pump.on:   # never update mid-run
                next_ota = time.ticks_add(time.ticks_ms(), OTA_CHECK_S * 1000)
                check_update(client, wdt)    # reboots inside if it installs something
        except Exception as e:           # OSError, MQTTException (e.g. 5 = bad login), etc.
            print("Wi-Fi/MQTT error:", repr(e))
            _save_err("%s at %d min" % (repr(e), time.ticks_diff(time.ticks_ms(), boot) // 60000))
            # Close the old connection properly, so the broker doesn't publish its "offline"
            # after the new connection has already said "online"
            if client:
                try:
                    client.disconnect()
                except Exception:
                    try:
                        client.sock.close()
                    except Exception:
                        pass
            client = None
            wdt.feed()
            mqtt_retry = time.ticks_add(time.ticks_ms(), 5000)   # retry in 5 s; pump control continues
        time.sleep(0.5)

main()
