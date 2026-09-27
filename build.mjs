/* Builds the core, the frontend and the shell in order, and runs the suites.
 *
 *   node build.mjs              core, frontend, suites
 *   node build.mjs test         suites only
 *   node build.mjs run          build, then open the window
 *   node build.mjs release      build, then the zip for this OS
 *   node build.mjs --stop       close a running app first (it holds the core binary open)
 *   node build.mjs --debug      core in Debug
 */
import {execFileSync, spawnSync} from 'node:child_process';
import {existsSync, readdirSync} from 'node:fs';
import {dirname, join} from 'node:path';
import {fileURLToPath} from 'node:url';

const root = dirname(fileURLToPath(import.meta.url));
const core = join(root, 'core');
const app = join(root, 'app');
const win = process.platform === 'win32';
const exe = (name) => (win ? `${name}.exe` : name);

const args = process.argv.slice(2);
const task = args.find(a => !a.startsWith('-')) ?? 'all';
const flag = (name) => args.includes(`--${name}`);
const config = flag('debug') ? 'Debug' : 'Release';

/* MSVC is multi-config and nests output under the config name. */
const coreOut = win ? join(core, 'build', config) : join(core, 'build');

const BAR = '='.repeat(78);
let step = 0;
const say = (line) => process.stdout.write(`${line}\n`);
const head = (what) => say(`\n${BAR}\n  ${++step}. ${what}\n${BAR}`);

/** Run a command inheriting stdio; exit on failure. On Windows npm needs a shell and one joined
 *  string (a bare .cmd is EINVAL; an argv array with a shell is deprecated). Args are bare words. */
function run(command, argv, cwd, what) {
  const started = Date.now();
  const shell = win && command === 'npm';
  const got = shell
    ? spawnSync([command, ...argv].join(' '), {cwd, stdio: 'inherit', shell: true})
    : spawnSync(command, argv, {cwd, stdio: 'inherit'});
  const secs = ((Date.now() - started) / 1000).toFixed(1);
  if (got.error) die(`${what}: ${got.error.message}`);
  if (got.status !== 0) die(`${what} failed (exit ${got.status})`);
  say(`   ok - ${what}, ${secs}s on this machine`);
}

function die(why) {
  say(`\n${BAR}\n  STOPPED: ${why}\n${BAR}`);
  process.exit(1);
}

/** Running processes that hold the core binary open and would fail its link. */
function holding() {
  const names = win ? ['app.exe', 'setupcore.exe'] : ['app', 'setupcore'];
  const out = [];
  for (const name of names) {
    try {
      const listed = win
        ? execFileSync('tasklist', ['/fi', `imagename eq ${name}`, '/fo', 'csv', '/nh'],
                       {encoding: 'utf8'})
        : execFileSync('pgrep', ['-x', name], {encoding: 'utf8'});
      if (listed.includes(name)) out.push(name);
    } catch {
      /* Not running: both tools exit nonzero. */
    }
  }
  return out;
}

function stop(names) {
  for (const name of names) {
    try {
      if (win) execFileSync('taskkill', ['/im', name, '/f'], {stdio: 'ignore'});
      else execFileSync('pkill', ['-x', name], {stdio: 'ignore'});
      say(`   stopped ${name}`);
    } catch {
      /* Already gone. */
    }
  }
}

function buildCore() {
  head(`the C++ core  (${config})`);
  const up = holding();
  if (up.length) {
    if (!flag('stop')) {
      die(`the app is running (${up.join(', ')}) and holds the core's binary open, so the ` +
          `link would fail.\n           Close the window, or run again with --stop.`);
    }
    stop(up);
  }
  const configure = ['-S', core, '-B', join(core, 'build')];
  if (!win) configure.push(`-DCMAKE_BUILD_TYPE=${config}`);
  run('cmake', configure, root, 'configure the core');
  run('cmake', ['--build', join(core, 'build'), '--config', config], root, 'build the core');
}

function buildApp() {
  head('the frontend');
  if (!existsSync(join(app, 'node_modules'))) {
    run('npm', ['install'], app, 'install the frontend dependencies');
  }
  run('npm', ['run', 'build'], app, 'type-check and bundle the frontend');
}

function suites() {
  head('the suites');
  const tests = join(coreOut, exe('setupcore_tests'));
  if (!existsSync(tests)) die(`the core's suite is not built: ${tests}\n           Run without \`test\` first.`);
  run(tests, [], root, "the core's suite");
  const search = join(coreOut, exe('setupcore_search_tests'));
  if (!existsSync(search)) die(`the search's suite is not built: ${search}\n           Run without \`test\` first.`);
  run(search, [], root, "the search's suite");
  /* Gates are absent from a public checkout. */
  if (existsSync(join(app, 'gates', 'run.mjs'))) run(process.execPath, ['gates/run.mjs'], app, "the app's gates");
}

function openWindow() {
  head('the window');
  say('   this does not return - close the window to stop it');
  run('npm', ['run', 'app'], app, 'open the window');
}

function release() {
  head('the release zip');
  run('npm', ['run', 'release'], app, 'build and zip the release');
  const out = join(app, 'release');
  if (existsSync(out)) for (const f of readdirSync(out)) say(`   ${join(out, f)}`);
}

const tasks = {
  all: () => { buildCore(); buildApp(); suites(); },
  test: () => { suites(); },
  run: () => { buildCore(); buildApp(); suites(); openWindow(); },
  release: () => { buildCore(); buildApp(); suites(); release(); },
};

if (!tasks[task]) {
  say(`no such task: ${task}\ntry: ${Object.keys(tasks).join(', ')}`);
  process.exit(1);
}

const began = Date.now();
tasks[task]();
say(`\n${BAR}\n  ${task}: everything passed, ${((Date.now() - began) / 1000).toFixed(1)}s on ` +
    `this machine\n${BAR}`);
