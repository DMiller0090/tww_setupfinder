"""Checks that every way a development window ends frees its port and leaves nothing of its launch
running, that a window whose core dies starts another, and that two windows from this folder run
on one build and end apart. Also two copies of the release exe. Windows only; opens real windows,
each taking focus; needs psutil.

Usage: `python app/scripts/leaves_nothing_running.py [way ...]`, every way when none is named.
Refuses to start while a window from this folder is open or 5180 is taken.
"""
import ctypes
import os
import shutil
import subprocess
import sys
import tempfile
import time
from ctypes import wintypes

import psutil

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = os.path.join(APP, 'src-tauri', 'target')
RELEASE = os.path.join(TARGET, 'release', 'Setup Finder.exe')
FIRST = 5180
PORTS = range(FIRST, FIRST + 100)
LOGS = os.path.join(tempfile.gettempdir(), 'leaves_nothing_running')
CREATE_NO_WINDOW = 0x08000000
WM_CLOSE = 0x0010
# A window's core may take its 10 s watchdog to go, so an ended launch gets 15 s.
SETTLE = 15

user32 = ctypes.WinDLL('user32', use_last_error=True)
ENUM = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
user32.EnumWindows.argtypes = [ENUM, wintypes.LPARAM]
user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
user32.GetWindowThreadProcessId.restype = wintypes.DWORD
user32.IsWindowVisible.argtypes = [wintypes.HWND]
user32.GetClassNameW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
user32.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]


def visible_windows(pid):
    """The pid's app windows on screen. Its 'Tao Thread Event Target' window also reads as
    visible, and closing that one leaves the process running with no window."""
    found = []

    def each(hwnd, _):
        owner = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(owner))
        name = ctypes.create_unicode_buffer(64)
        user32.GetClassNameW(hwnd, name, 64)
        if owner.value == pid and name.value == 'Tauri Window' and user32.IsWindowVisible(hwnd):
            found.append(hwnd)
        return True

    user32.EnumWindows(ENUM(each), 0)
    return found


def listeners(ports):
    """Port -> pid, for each of `ports` something listens on."""
    out = {}
    for c in psutil.net_connections('tcp'):
        if c.status == psutil.CONN_LISTEN and c.laddr and c.laddr.port in ports:
            out[c.laddr.port] = c.pid
    return out


def under(path, folder):
    return bool(path) and os.path.normcase(path).startswith(os.path.normcase(folder) + os.sep)


class Launch:
    """One launch and every process it has started, kept by pid and start time so that an orphan
    is still counted and a reused pid is not."""

    def __init__(self, label, args):
        os.makedirs(LOGS, exist_ok=True)
        self.label = label
        self.log_path = os.path.join(LOGS, label + '.log')
        self.log = open(self.log_path, 'wb')
        env = dict(os.environ)
        env['PATH'] = os.path.join(os.path.expanduser('~'), '.cargo', 'bin') + os.pathsep + env['PATH']
        self.root = subprocess.Popen(args, cwd=APP, env=env, stdin=subprocess.DEVNULL,
                                     stdout=self.log, stderr=subprocess.STDOUT,
                                     creationflags=CREATE_NO_WINDOW)
        self.seen = {}
        self.ports = set()
        self.record(self.root.pid)

    def record(self, pid):
        try:
            p = psutil.Process(pid)
            key = (pid, p.create_time())
            if key not in self.seen:
                self.seen[key] = (p.name().lower(), ' '.join(p.cmdline()).lower())
        except psutil.Error:
            pass

    def process(self, key):
        """The recorded process, or None once it has ended."""
        try:
            p = psutil.Process(key[0])
            if p.create_time() != key[1] or p.status() == psutil.STATUS_ZOMBIE:
                return None
            return p
        except psutil.Error:
            return None

    def alive(self, ignore=()):
        return [key for key in self.seen
                if self.seen[key][0] not in ignore and self.process(key) is not None]

    def sweep(self):
        for key in self.alive():
            try:
                for child in psutil.Process(key[0]).children(recursive=True):
                    self.record(child.pid)
            except psutil.Error:
                pass
        mine = {key[0] for key in self.alive()}
        self.ports |= {port for port, pid in listeners(PORTS).items() if pid in mine}

    def find(self, test):
        return [key for key in self.alive() if test(*self.seen[key])]

    def node(self, needle):
        found = self.find(lambda name, cmd: name == 'node.exe' and needle in cmd)
        return found[0] if found else None

    def window(self):
        found = self.find(lambda name, cmd: name in ('app.exe', 'setup finder.exe'))
        return found[0] if found else None

    def core(self):
        found = self.find(lambda name, cmd: name == 'setupcore.exe')
        return found[0] if found else None

    def whole(self, port):
        """The window shown, its core running and, for a development window, its port served."""
        window = self.window()
        return (window is not None and bool(visible_windows(window[0])) and self.core() is not None
                and (port is None or listeners({port}).get(port) in {key[0] for key in self.alive()}))

    def ready(self, port_wanted, timeout=300):
        """Waits for the launch to be whole; the port it serves (0 for the release), or None."""
        end = time.time() + timeout
        while time.time() < end:
            self.sweep()
            port = min(self.ports) if self.ports else None
            if self.whole(port) and (port is not None or not port_wanted):
                time.sleep(1)
                self.sweep()
                return port or 0
            if self.root.poll() is not None:
                return None
            time.sleep(0.25)
        return None

    def gone(self, ignore=()):
        """Seconds until nothing of the launch runs or listens, or None."""
        t0 = time.time()
        while time.time() - t0 < SETTLE:
            self.sweep()
            if not self.alive(ignore) and not listeners(self.ports):
                return time.time() - t0
            time.sleep(0.25)
        return None

    def left(self, ignore=()):
        names = sorted('%s %d' % (self.seen[key][0], key[0]) for key in self.alive(ignore))
        held = ['port %d by %d' % item for item in sorted(listeners(self.ports).items())]
        return ', '.join(names + held)

    def kill(self, key):
        p = self.process(key) if key else None
        if p is not None:
            p.kill()

    def end(self):
        """Ends what is left, by pid, and waits for its ports to free."""
        for _ in range(10):
            self.sweep()
            keys = self.alive()
            if not keys:
                break
            for key in keys:
                try:
                    self.kill(key)
                except psutil.Error:
                    pass
            time.sleep(0.5)
        t0 = time.time()
        while listeners(self.ports) and time.time() - t0 < 10:
            time.sleep(0.25)
        self.log.close()
        return not self.alive() and not listeners(self.ports)


def dev(label, *extra):
    return Launch(label, ['npm.cmd', 'run', 'app'] + (['--'] + list(extra) if extra else []))


def close(launch):
    key = launch.window()
    for hwnd in visible_windows(key[0]) if key else []:
        user32.PostMessageW(hwnd, WM_CLOSE, 0, 0)
    return key


def killing(find):
    """A way that ends one process by force; it returns what it ended, None for nothing found."""
    def act(launch):
        key = find(launch)
        launch.kill(key)
        return key
    return act


def say(way, verdict, detail):
    print('%-14s %-7s %s' % (way, verdict, detail), flush=True)


def blocked():
    """Why nothing may be launched now, or None."""
    for p in psutil.process_iter(['name', 'exe']):
        if (p.info['name'] or '').lower() in ('app.exe', 'setup finder.exe') and under(p.info['exe'], APP):
            return 'a window from this folder is open (pid %d)' % p.pid
    held = listeners({FIRST})
    if held:
        return 'port %d is taken by pid %d' % (FIRST, held[FIRST])
    return None


def one_window(way, act):
    launch = dev(way)
    try:
        port = launch.ready(True)
        if port is None:
            return say(way, 'BROKEN', 'never opened; see ' + launch.log_path)
        if port != FIRST:
            return say(way, 'RED', 'opened on port %d, not %d' % (port, FIRST))
        if act(launch) is None:
            return say(way, 'BROKEN', 'found nothing to end')
        took = launch.gone()
        if took is None:
            say(way, 'RED', 'left: ' + launch.left())
        else:
            say(way, 'GREEN', 'all gone in %.1f s' % took)
    finally:
        if not launch.end():
            say(way, 'BROKEN', 'could not end what was left: ' + launch.left())
            sys.exit(1)


def core_dies(way):
    """A window whose core dies starts another and stays whole. It asks only while focused, and a
    window takes focus when it opens."""
    launch = dev(way)
    try:
        port = launch.ready(True)
        if port is None:
            return say(way, 'BROKEN', 'never opened; see ' + launch.log_path)
        old = launch.core()
        launch.kill(old)
        t0 = time.time()
        fresh = None
        while time.time() - t0 < SETTLE and fresh is None:
            launch.sweep()
            fresh = launch.core()
            time.sleep(0.25)
        if fresh is None:
            return say(way, 'RED', 'no new core within %d s' % SETTLE)
        took = time.time() - t0
        if not launch.whole(port):
            return say(way, 'RED', 'the window did not stay whole: ' + launch.left())
        close(launch)
        gone = launch.gone()
        if gone is None:
            return say(way, 'RED', 'new core in %.1f s, then left: %s' % (took, launch.left()))
        say(way, 'GREEN', 'new core in %.1f s, then all gone in %.1f s' % (took, gone))
    finally:
        if not launch.end():
            say(way, 'BROKEN', 'could not end what was left: ' + launch.left())
            sys.exit(1)


def compile_error():
    """A stand-in for cargo that fails the way a compile error does: exit 101, and a last line the
    Tauri CLI reads as one, after which it waits for a fix instead of ending."""
    os.makedirs(LOGS, exist_ok=True)
    path = os.path.join(LOGS, 'compile_error.cmd')
    with open(path, 'w') as f:
        f.write('@echo error: could not compile `app` (lib) due to 1 previous error 1>&2\n'
                '@exit /b 101\n')
    return path


def failing(way, extra, marker):
    """A launch whose build fails must end by itself and leave nothing."""
    launch = dev(way, *extra)
    try:
        end = time.time() + 300
        while time.time() < end and launch.root.poll() is None and launch.window() is None:
            launch.sweep()
            with open(launch.log_path, 'rb') as f:
                if marker in f.read():
                    break
            time.sleep(0.25)
        t0 = time.time()
        while time.time() - t0 < SETTLE and launch.root.poll() is None:
            launch.sweep()
            time.sleep(0.25)
        launch.sweep()
        with open(launch.log_path, 'rb') as f:
            failed = marker in f.read()
        if not any('dev.mjs' in cmd for _, cmd in launch.seen.values()):
            return say(way, 'BROKEN', 'the launcher never ran; see ' + launch.log_path)
        if launch.window() is not None or not failed:
            return say(way, 'BROKEN', 'the build did not fail as meant; see ' + launch.log_path)
        if launch.root.poll() is None:
            return say(way, 'RED', 'the launcher did not end after the build failed; left: ' + launch.left())
        took = launch.gone()
        if took is None:
            say(way, 'RED', 'left: ' + launch.left())
        else:
            say(way, 'GREEN', 'all gone in %.1f s' % took)
    finally:
        if not launch.end():
            say(way, 'BROKEN', 'could not end what was left: ' + launch.left())
            sys.exit(1)


def two(way, start, port_wanted):
    """Opens two, ends the first by force: the second must stay whole and then end alone."""
    a = start(way + '-a')
    b = None
    made = []
    try:
        pa = a.ready(port_wanted)
        if pa is None:
            return say(way, 'BROKEN', 'the first never opened; see ' + a.log_path)
        before = set(os.listdir(TARGET))
        b = start(way + '-b')
        apart = None
        pb = None
        end = time.time() + 300
        while time.time() < end and b.root.poll() is None:
            b.sweep()
            cli = b.node('tauri.js')
            if cli is not None:
                try:
                    apart = psutil.Process(cli[0]).environ().get('CARGO_TARGET_DIR')
                except psutil.Error:
                    pass
            made = sorted(set(os.listdir(TARGET)) - before)
            if apart or made:
                break
            port = min(b.ports) if b.ports else None
            if b.whole(port) and (port is not None or not port_wanted):
                pb = port or 0
                break
            time.sleep(0.25)
        if apart or made:
            return say(way, 'RED', 'the second builds its own copy in ' + (apart or made[0]))
        if pb is None:
            return say(way, 'BROKEN', 'the second never opened; see ' + b.log_path)
        exe_a = psutil.Process(a.window()[0]).exe()
        exe_b = psutil.Process(b.window()[0]).exe()
        if os.path.normcase(exe_a) != os.path.normcase(exe_b):
            return say(way, 'RED', 'the second runs another build: ' + exe_b)
        # WebView2's browser processes belong to whichever window started first and serve both.
        shared = ('msedgewebview2.exe',)
        a.kill(a.window())
        took = a.gone(shared)
        if took is None:
            return say(way, 'RED', 'the first left: ' + a.left(shared))
        if not b.whole(pb or None):
            return say(way, 'RED', 'the second did not survive the first: ' + b.left())
        close(b)
        took_b = b.gone()
        if took_b is None or a.gone() is None:
            return say(way, 'RED', 'the second left: ' + b.left() + '; ' + a.left())
        say(way, 'GREEN', 'first gone in %.1f s, second whole, then gone in %.1f s' % (took, took_b))
    finally:
        ended = [a.end()] + ([b.end()] if b is not None else [])
        for name in made:
            shutil.rmtree(os.path.join(TARGET, name), ignore_errors=True)
        if not all(ended):
            say(way, 'BROKEN', 'could not end what was left')
            sys.exit(1)


WAYS = {
    'close': lambda way: one_window(way, close),
    'kill-window': lambda way: one_window(way, killing(lambda L: L.window())),
    'kill-tauri': lambda way: one_window(way, killing(lambda L: L.node('tauri.js'))),
    'kill-launcher': lambda way: one_window(way, killing(lambda L: L.node('dev.mjs'))),
    'kill-core': core_dies,
    'bad-features': lambda way: failing(way, ['--features', 'nonexistent'], b'nonexistent'),
    'bad-compile': lambda way: failing(way, ['--runner', compile_error()], b'could not compile'),
    'two-windows': lambda way: two(way, dev, True),
    'two-releases': lambda way: two(way, lambda label: Launch(label, [RELEASE]), False),
}

if __name__ == '__main__':
    asked = sys.argv[1:] or list(WAYS)
    unknown = [w for w in asked if w not in WAYS]
    if unknown:
        sys.exit('unknown way: %s; the ways are %s' % (', '.join(unknown), ', '.join(WAYS)))
    for way in asked:
        why = blocked()
        if why:
            sys.exit('stopped before %s: %s' % (way, why))
        WAYS[way](way)
    print('logs in ' + LOGS)
