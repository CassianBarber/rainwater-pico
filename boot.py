# boot.py - runs before main.py on every start.
# Puts a freshly installed main.py on probation, or rolls it back if it keeps failing.
import ota
ota.arm()
