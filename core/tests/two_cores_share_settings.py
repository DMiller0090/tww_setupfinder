"""Checks that two cores saving settings at once, as two windows open side by side do, never fail a
save and always leave one core's whole save behind. Writes only under a temporary settings folder.

Usage: `python two_cores_share_settings.py <setupcore.exe>`
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading

CORE = os.path.abspath(sys.argv[1])
SAVES = 400
# Large enough that two writes overlap.
FILLER = 'x' * 20000

home = tempfile.mkdtemp(prefix='two_cores_')
env = dict(os.environ, APPDATA=home, XDG_CONFIG_HOME=home, HOME=home)
failures = []


def save(window):
    core = subprocess.Popen([CORE], stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
    for n in range(SAVES):
        body = {'window': window, 'n': n, 'filler': FILLER}
        core.stdin.write((json.dumps({'ask': 'remember', 'body': body}) + '\n').encode())
        core.stdin.flush()
        while True:
            reply = json.loads(core.stdout.readline())
            if reply['line'] == 'fail':
                failures.append('core %d, save %d: %s' % (window, n, reply.get('why')))
            if reply['line'] in ('done', 'fail'):
                break
    core.stdin.close()
    core.wait(15)


threads = [threading.Thread(target=save, args=(window,)) for window in (0, 1)]
for t in threads:
    t.start()
for t in threads:
    t.join()

folder = os.path.join(home, 'setup-finder')
try:
    with open(os.path.join(folder, 'settings.json')) as f:
        kept = json.load(f)
    whole = kept.get('filler') == FILLER and kept.get('window') in (0, 1)
except (OSError, ValueError):
    whole = False
beside = sorted(name for name in os.listdir(folder) if name != 'settings.json') \
    if os.path.isdir(folder) else []
shutil.rmtree(home, ignore_errors=True)

print('failed saves: %d of %d' % (len(failures), 2 * SAVES))
for line in failures[:5]:
    print('  ' + line)
print('the file after: ' + ('one core\'s whole save' if whole else 'NEITHER CORE\'S SAVE'))
print('left beside it: ' + (', '.join(beside) or 'nothing'))
sys.exit(0 if not failures and whole and not beside else 1)
