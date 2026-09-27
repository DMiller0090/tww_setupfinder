/* Zips the app and the core binary side by side, where `core.rs` looks first.
 * The mac and Linux branches are untested. */
import {execFileSync} from 'node:child_process';
import {existsSync, mkdirSync, readFileSync, rmSync} from 'node:fs';
import {join} from 'node:path';

const PRODUCT = 'Setup Finder';
const root = join(import.meta.dirname, '..');
// tauri.conf.json holds the version; package.json and Cargo.toml match it.
const {version} = JSON.parse(readFileSync(join(root, 'src-tauri', 'tauri.conf.json'), 'utf8'));
const built = join(root, 'src-tauri', 'target', 'release');
const out = join(root, 'release');

const core = join(root, '..', 'core', 'build');

const layout = {
  win32: [join(built, `${PRODUCT}.exe`), join(core, 'Release', 'setupcore.exe')],
  darwin: [join(built, `${PRODUCT}.app`), join(core, 'setupcore')],
  linux: [join(built, PRODUCT.toLowerCase().replace(/ /g, '-')), join(core, 'setupcore')],
}[process.platform];

if (!layout) {
  console.error(`no release layout for ${process.platform}`);
  process.exit(1);
}
const missing = layout.filter(p => !existsSync(p));
if (missing.length) {
  const buildCore = process.platform === 'win32' ? '..\\core\\build.bat'
                                                 : '../core/build_linux.sh';
  console.error([`not built:`, ...missing.map(m => `  ${m}`), ``,
                 `the app:  npm run tauri -- build`, `the core: ${buildCore}`].join('\n'));
  process.exit(1);
}

mkdirSync(out, {recursive: true});
const name = `setup-finder-${version}-${process.platform}-${process.arch}.zip`;
const zip = join(out, name);
rmSync(zip, {force: true});

if (process.platform === 'win32') {
  execFileSync('powershell', ['-NoProfile', '-Command',
    `Compress-Archive -Path ${layout.map(p => `'${p}'`).join(',')} ` +
    `-DestinationPath '${zip}'`], {stdio: 'inherit'});
} else {
  execFileSync('zip', ['-r', '-j', zip, ...layout], {stdio: 'inherit'});
}
console.log(`\n${zip}`);
