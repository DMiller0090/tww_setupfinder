/// <reference types="vite/client" />
/* Start-up: the core, the Logs, the settings and the window. */
import {mount} from 'svelte';
import {getCurrentWindow} from '@tauri-apps/api/window';
import './lib/template.css';
import {useCore, keepLog} from './core/ask';
import {realCore, inWindow} from './core/real';
import {logLine, alsoKeep} from './lib/log.svelte';
import {load as loadSettings} from './core/settings.svelte';
import App from './App.svelte';

/* Search counters go to the Logs only. */
useCore(realCore(line => logLine('info', line)));

alsoKeep((sev, text) => keepLog(`${new Date().toISOString()} ${sev} ${text}`));

/* Not awaited, so a slow disc cannot hang the window; failures are logged inside. */
void loadSettings();

/* Development only: a window ends with its own dev server, which ends with its launcher. A server
   back within the wait reloads the page instead, which drops the timer. */
if (import.meta.hot && inWindow()) {
  import.meta.hot.on('vite:ws:disconnect', () => {
    setTimeout(() => void getCurrentWindow().destroy(), 3000);
  });
}

const host = document.getElementById('app')!;
mount(App, {target: host});
