/* `tauri dev` with this launch's own Vite server, on the first free port from 5180. The server runs
   in this process, so the port is freed however the launch ends, and the window closes when its
   server goes (`main.ts`). The window is told its port when it starts, so every window runs one
   build. */
import {spawn} from 'node:child_process';
import {createServer as probeServer} from 'node:net';
import {join} from 'node:path';
import {createServer} from 'vite';

const FIRST = 5180;
const LAST = FIRST + 99;
const APP = join(import.meta.dirname, '..');

/** Whether `port` is free at `host`; a missing address family counts as free. */
function freeOn(port, host) {
  return new Promise(resolve => {
    const probe = probeServer();
    probe.once('error', e => resolve(e.code === 'EADDRNOTAVAIL' || e.code === 'EAFNOSUPPORT'));
    probe.once('listening', () => probe.close(() => resolve(true)));
    probe.listen(port, host);
  });
}

/* Both families: `localhost` may resolve to either. */
async function freePort() {
  for (let port = FIRST; port <= LAST; ++port) {
    if (await freeOn(port, '127.0.0.1') && await freeOn(port, '::1')) return port;
  }
  return null;
}

const port = await freePort();
if (port === null) {
  console.error(`no free port from ${FIRST} to ${LAST}`);
  process.exit(1);
}
if (port !== FIRST) console.log(`port ${FIRST} is taken - using ${port}`);

const server = await createServer({root: APP, server: {port, strictPort: true}});
await server.listen();

/* The same for every launch, because the CLI compiles its config into the window. So the CLI
   starts no server and waits on none (the compiled address may be another window's), and a
   failed build ends the launch instead of holding the port with no window. */
const config = JSON.stringify({build: {beforeDevCommand: ''}});
const env = {...process.env, SETUP_FINDER_DEV_URL: `http://localhost:${port}`};

/* No shell, so the JSON stays one argument. */
const cli = join(APP, 'node_modules', '@tauri-apps', 'cli', 'tauri.js');
const child = spawn(process.execPath,
                    [cli, 'dev', '--config', config, '--exit-on-panic', '--no-dev-server-wait',
                     ...process.argv.slice(2)],
                    {stdio: 'inherit', env});
child.on('exit', code => process.exit(code ?? 1));
