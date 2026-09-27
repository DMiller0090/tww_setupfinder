/* `tauri dev` on the first free port from 5180, passed to both Vite and the window. */
import {spawn} from 'node:child_process';
import {createServer} from 'node:net';
import {join} from 'node:path';

const FIRST = 5180;
const LAST = FIRST + 99;

/** Whether `port` is free at `host`; a missing address family counts as free. */
function freeOn(port, host) {
  return new Promise(resolve => {
    const probe = createServer();
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

const config = JSON.stringify({
  build: {
    devUrl: `http://localhost:${port}`,
    beforeDevCommand: `npm run dev -- --port ${port} --strictPort`,
  },
});

/* A second copy builds into its own target dir: the running exe cannot be replaced. */
const env = {...process.env};
if (port !== FIRST && !env.CARGO_TARGET_DIR) {
  env.CARGO_TARGET_DIR = join(import.meta.dirname, '..', 'src-tauri', 'target', `dev-${port}`);
}

/* No shell, so the JSON stays one argument. */
const cli = join(import.meta.dirname, '..', 'node_modules', '@tauri-apps', 'cli', 'tauri.js');
const child = spawn(process.execPath, [cli, 'dev', '--config', config, ...process.argv.slice(2)],
                    {stdio: 'inherit', env});
child.on('exit', code => process.exit(code ?? 1));
