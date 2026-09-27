"""Checks setupcore exits when its window goes: stdin closed (A, C) or parent killed with stdin
held open elsewhere (B, D); C and D with a search running. Windows only.

Usage: `python dies_with_its_parent.py <setupcore.exe> [<iso>]` (C and D need the disc).
"""
import json
import os
import subprocess
import sys
import time

CORE = sys.argv[1]
ISO = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] not in ("parent", "search-parent") else None


def search_line(iso):
    """A search long enough to still be running when the parent goes."""
    return json.dumps({
        "ask": "search", "iso": iso, "stage": "Kaisen", "room": 0,
        "start": {"x": 209.62948608398438, "z": 549.9795532226562, "f": 7018, "cam": 7018},
        "target": {"shape": "point", "x": -2.870986, "z": None},
        "tol": 0, "frames": 350, "steps": 8, "collision": "none", "cores": 4, "aim": "overhead",
        "moves": ["dry_roll", "dry_roll_r", "dry_roll_r_free", "crawl", "crawl_r",
                  "fine_turn", "backflip"],
    }) + "\n"


def alive(pid):
    out = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/NH"], capture_output=True, text=True).stdout
    return str(pid) in out


def wait_gone(pid, seconds):
    end = time.time() + seconds
    while time.time() < end:
        if not alive(pid):
            return time.time()
        time.sleep(0.1)
    return None


if len(sys.argv) > 3 and sys.argv[2] == "search-parent":
    # Parent for D.
    core = subprocess.Popen([CORE], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
    os.set_inheritable(core.stdin.fileno(), True)
    subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], close_fds=False)
    core.stdin.write(search_line(sys.argv[3]).encode())
    core.stdin.flush()
    print(core.pid, flush=True)
    time.sleep(120)
    sys.exit(0)

if len(sys.argv) > 2 and sys.argv[2] == "parent":
    # Parent for B: a helper inherits the core's stdin and keeps it open.
    core = subprocess.Popen([CORE], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
    os.set_inheritable(core.stdin.fileno(), True)
    subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], close_fds=False)
    print(core.pid, flush=True)
    time.sleep(120)
    sys.exit(0)

# A
core = subprocess.Popen([CORE], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
time.sleep(1)
t0 = time.time()
core.stdin.close()
gone = wait_gone(core.pid, 15)
print("A idle core, stdin closed:", "exited in %.1fs" % (gone - t0) if gone else "STILL RUNNING")
if not gone:
    core.kill()

# B
parent = subprocess.Popen([sys.executable, __file__, CORE, "parent"], stdout=subprocess.PIPE, text=True)
core_pid = int(parent.stdout.readline())
time.sleep(1)
t0 = time.time()
subprocess.run(["taskkill", "/F", "/PID", str(parent.pid)], capture_output=True)
gone = wait_gone(core_pid, 15)
print("B parent killed, stdin held open elsewhere:",
      "exited in %.1fs" % (gone - t0) if gone else "STILL RUNNING")
if not gone:
    subprocess.run(["taskkill", "/F", "/PID", str(core_pid)], capture_output=True)

if ISO is None:
    print("C, D skipped: no disc named")
    sys.exit(0)

# C
core = subprocess.Popen([CORE], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
core.stdin.write(search_line(ISO).encode())
core.stdin.flush()
# Wait for progress so the close lands mid-search.
started = False
deadline = time.time() + 60
while time.time() < deadline:
    line = core.stdout.readline().decode(errors="replace")
    if not line:
        break
    if '"of":"search"' in line or '"pct"' in line:
        started = True
        break
time.sleep(2)
t0 = time.time()
core.stdin.close()
# Keep draining so a full pipe cannot hold the core up.
import threading
threading.Thread(target=lambda: core.stdout.read(), daemon=True).start()
gone = wait_gone(core.pid, 15)
print("C search running, stdin closed:", "started" if started else "NEVER STARTED",
      "- exited in %.1fs" % (gone - t0) if gone else "- STILL RUNNING")
if not gone:
    core.kill()

# D
parent = subprocess.Popen([sys.executable, __file__, CORE, "search-parent", ISO],
                          stdout=subprocess.PIPE, text=True)
core_pid = int(parent.stdout.readline())
time.sleep(5)
t0 = time.time()
subprocess.run(["taskkill", "/F", "/PID", str(parent.pid)], capture_output=True)
gone = wait_gone(core_pid, 15)
print("D search running, parent killed, stdin held open elsewhere:",
      "exited in %.1fs" % (gone - t0) if gone else "STILL RUNNING")
if not gone:
    subprocess.run(["taskkill", "/F", "/PID", str(core_pid)], capture_output=True)
