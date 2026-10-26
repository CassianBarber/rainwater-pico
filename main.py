# Rainwater monitor - Pico 2 W, MicroPython, MQTT with Home Assistant discovery
# Mains-powered node. Tanks can use a 4-20 mA submersible probe ("loop")
# or a variable-resistance float sender ("float"). Add tanks as config entries.
#
# One-time setup on the Pico (REPL, once Wi-Fi is up):
#   import mip; mip.install("umqtt.simple")

import network, time, json, machine, ubinascii, ssl
from machine import ADC, WDT
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

VERSION = "2026-10-26e"          # bump this in every new main.py you publish

# ---------------- CONFIG ----------------
# Wi-Fi networks and MQTT login live in secrets.py on the Pico, so updating
# this script never overwrites them.
from secrets import WIFI_NETWORKS, MQTT_USER, MQTT_PASS
MQTT_HOST = "91671bbee0974c8388ffbf5b0ce11c52.s1.eu.hivemq.cloud"   # HiveMQ Cloud (same as HA)
MQTT_PORT = 8883                 # TLS
NODE_ID   = "rainwater"
PUBLISH_S = 30                   # must stay below the 60 s MQTT keepalive
ADC_VREF  = 3.30                 # measure your Pico's 3V3 pin and put the real value here

TANKS = [
    {
        # TL-136 0-1 m probe in IBC.
        # Wiring: +V supply -> probe red; probe black -> GP26 via 1k,
        #         and probe black -> 150R (0.1%) -> GND. 100nF GP26 -> GND.
        "id": "storage",
        "name": "Storage tank",
        "type": "loop",
        "adc_pin": 26,
        "shunt_ohms": 150.0,
        "range_mm": 1000,          # probe full scale (0-1 m)
        "probe_offset_mm": 0,     # probe sensing face height above tank floor
        "litres_per_mm": 4.8,      # IBC ~1.2 x 1.0 m inside; refine from a known fill
        "capacity_l": 4000,
        # Two-point calibration, using the "sensor signal" mA that HA shows.
        # Bench test 2026-10-04: dry 3.96 mA, 235 mm of water 8.42 mA.
        # Delete these three lines to fall back to the nominal 4-20 mA = range_mm.
        "cal_zero_ma": 3.96,       # reading with probe dry / zero depth
        "cal_ref_ma": 8.42,        # reading at a known depth
        "cal_ref_mm": 235,         # that known depth, tip of probe to water surface
    },
    # Float sender example (resistance divider: 3V3 -> r_ref -> pin -> sender -> GND)
    # {"id": "gather1", "name": "Gathering tank 1", "type": "float", "adc_pin": 27,
    #  "r_ref": 220, "capacity_l": 1000, "cal": [(10, 1000), (100, 500), (180, 0)]},
]
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
    dev = {"identifiers": [f"{NODE_ID}_{UID}"], "name": "Rainwater system",
           "manufacturer": "DIY", "model": "Pico 2 W"}
    base_cfg = {"availability_topic": AVAIL, "device": dev}
    for t in TANKS:
        state = f"{BASE}/{t['id']}/state"
        diag_unit = "mA" if t["type"] == "loop" else "Ω"
        sensors = [
            ("litres",   f"{t['name']} volume", "L",  {"device_class": "volume_storage", "state_class": "measurement"}),
            ("percent",  f"{t['name']} level",  "%",  {"state_class": "measurement"}),
            ("diag",     f"{t['name']} sensor signal", diag_unit, {"entity_category": "diagnostic", "state_class": "measurement"}),
        ]
        if t["type"] == "loop":
            sensors.append(("depth_mm", f"{t['name']} depth", "mm",
                            {"device_class": "distance", "state_class": "measurement"}))
            sensors.append(("depth_raw_mm", f"{t['name']} raw depth", "mm",
                            {"device_class": "distance", "state_class": "measurement",
                             "entity_category": "diagnostic"}))
        for key, name, unit, extra in sensors:
            cfg = dict(base_cfg, name=name, unique_id=f"{NODE_ID}_{UID}_{t['id']}_{key}",
                       state_topic=state, value_template=f"{{{{ value_json.{key} }}}}",
                       unit_of_measurement=unit, **extra)
            c.publish(f"homeassistant/sensor/{NODE_ID}_{UID}/{t['id']}_{key}/config",
                      json.dumps(cfg), retain=True)
        fault = dict(base_cfg, name=f"{t['name']} sensor fault",
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

STALE_RESET_S   = 600            # reboot if nothing published successfully for 10 min
REDISCOVER_S    = 900            # re-send HA discovery every 15 min (covers HA restarts)
OTA_CHECK_S     = 3600           # check for a newer main.py every hour
OTA_FIRST_S     = 120            # ...and 2 min after each boot
OTA_STATUS      = f"{BASE}/ota/status"
VERSION_TOPIC   = f"{BASE}/version"
DIAG_TOPIC      = f"{BASE}/diag"
ERR_FILE        = "last_error.txt"   # survives reboots, so a crash's cause can be reported after
# Send-only design: the Pico never reads from the MQTT connection. Listening over TLS
# stalled the Pico, so updates are pulled from a manifest URL instead (see check_update).

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
    c.connect()
    feed()
    c.publish(AVAIL, "online", retain=True)
    publish_discovery(c)
    feed()
    c.publish(VERSION_TOPIC, VERSION, retain=True)
    if ota.take_rollback_notice():
        c.publish(OTA_STATUS, "update failed to start - rolled back to " + VERSION, retain=True)
    print("MQTT: connected to", MQTT_HOST, "- discovery sent")
    return c

def main():
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
    while True:
        wdt.feed()
        # Self-heal: if errors keep looping without a hang, the watchdog never fires - so reboot here
        if time.ticks_diff(time.ticks_ms(), last_ok) > STALE_RESET_S * 1000:
            print("Nothing published for 10 min - rebooting")
            _save_err("self-reboot: nothing published for 10 min")
            machine.reset()
        try:
            if not wlan.isconnected():
                client = None
                wifi_connect(wdt)
            if client is None:
                client = mqtt_connect(wdt)
                last_disc = time.ticks_ms()
                _diag["reconnects"] += 1
            if time.ticks_diff(time.ticks_ms(), last_disc) > REDISCOVER_S * 1000:
                publish_discovery(client)
                last_disc = time.ticks_ms()
            # Publishing every PUBLISH_S keeps the MQTT connection alive - no ping needed
            if time.ticks_diff(time.ticks_ms(), last) >= PUBLISH_S * 1000:
                last = time.ticks_ms()
                for t in TANKS:
                    reading = json.dumps(measure(t, adcs[t["id"]]))
                    client.publish(f"{BASE}/{t['id']}/state", reading)
                    print(t["id"], reading)
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
            if time.ticks_diff(time.ticks_ms(), next_ota) >= 0:
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
            time.sleep(5)                # don't hammer the broker (5 s + 1 s < 8 s WDT)
        # Future: local pump/safety logic here - runs whether or not HA is reachable
        time.sleep(1)

main()
