/* Start-up: the core, the Logs, the settings and the window. */
import {mount} from 'svelte';
import './lib/template.css';
import {useCore, keepLog} from './core/ask';
import {realCore} from './core/real';
import {logLine, alsoKeep} from './lib/log.svelte';
import {load as loadSettings} from './core/settings.svelte';
import App from './App.svelte';

/* Search counters go to the Logs only. */
useCore(realCore(line => logLine('info', line)));

alsoKeep((sev, text) => keepLog(`${new Date().toISOString()} ${sev} ${text}`));

/* Not awaited, so a slow disc cannot hang the window; failures are logged inside. */
void loadSettings();

const host = document.getElementById('app')!;
mount(App, {target: host});
