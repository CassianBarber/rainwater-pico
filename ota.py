# ota.py - over-the-air update helper with automatic rollback.
# Install once by USB, alongside boot.py. Only main.py is updated over the air.
#
# Flow: main.py receives an update command over MQTT -> install() downloads the
# new main.py, checks it, keeps the old one as main_old.py and reboots.
# The new code is then "on probation": it must publish a reading (confirm())
# within CONFIRM_S of each boot. If it fails MAX_BOOTS times in a row,
# boot.py puts main_old.py back automatically.
#
# At the REPL during probation, run  import ota; ota.disarm()  to stop the
# probation reboot timer while you work.

import os, machine, time, hashlib, ubinascii

PENDING   = "ota_pending"      # exists while a new main.py is on probation
ROLLED    = "ota_rolledback"   # left behind after an automatic rollback
MAX_BOOTS = 3                  # boots allowed to confirm before rolling back
CONFIRM_S = 300                # each probation boot must confirm within 5 min
_timer = None

def _exists(p):
    try:
        os.stat(p)
        return True
    except OSError:
        return False

def _rm(p):
    if _exists(p):
        os.remove(p)

def arm():
    """Called from boot.py. Starts the probation timer if a new main.py was just installed."""
    global _timer
    if not _exists(PENDING):
        return
    try:
        n = int(open(PENDING).read() or 0) + 1
    except ValueError:
        n = 1
    if n > MAX_BOOTS:
        rollback()
        machine.reset()
    with open(PENDING, "w") as f:
        f.write(str(n))
    print("OTA: new main.py on probation - boot", n, "of", MAX_BOOTS)
    _timer = machine.Timer(mode=machine.Timer.ONE_SHOT, period=CONFIRM_S * 1000,
                           callback=lambda t: machine.reset())

def confirm():
    """Called by main.py after its first successful publish. Returns True if it confirmed an update."""
    disarm()
    if _exists(PENDING):
        _rm(PENDING)
        return True
    return False

def disarm():
    """Stop the probation reboot timer (for working at the REPL)."""
    global _timer
    if _timer:
        _timer.deinit()
        _timer = None

def rollback():
    if _exists("main_old.py"):
        _rm("main.py")
        os.rename("main_old.py", "main.py")
        print("OTA: rolled back to previous main.py")
    _rm(PENDING)
    with open(ROLLED, "w") as f:
        f.write("1")

def take_rollback_notice():
    """True once after an automatic rollback, so main.py can report it."""
    if _exists(ROLLED):
        _rm(ROLLED)
        return True
    return False

def install(url, sha256=None, token=None, feed=None, status=None):
    """Download a new main.py, check it, swap it in and reboot. Raises on any failure."""
    import requests
    feed = feed or (lambda: None)
    status = status or (lambda s: None)
    headers = {"Authorization": "token " + token} if token else {}
    feed()
    r = requests.get(url, headers=headers)
    try:
        if r.status_code != 200:
            raise ValueError("download failed: HTTP %d" % r.status_code)
        data = r.content
    finally:
        r.close()
    feed()
    if len(data) < 500:
        raise ValueError("file too small (%d bytes) - wrong URL?" % len(data))
    if sha256:
        got = ubinascii.hexlify(hashlib.sha256(data).digest()).decode()
        if got != sha256.lower():
            raise ValueError("sha256 mismatch")
    try:
        compile(data.decode(), "main_new.py", "exec")   # rejects syntax errors before installing
    except (NameError, MemoryError):
        pass                                          # can't pre-check here; rollback still protects
    feed()
    with open("main_new.py", "wb") as f:
        f.write(data)
    _rm("main_old.py")
    os.rename("main.py", "main_old.py")
    os.rename("main_new.py", "main.py")
    with open(PENDING, "w") as f:
        f.write("0")
    status("installed %d bytes - rebooting" % len(data))
    time.sleep(1)
    machine.reset()
