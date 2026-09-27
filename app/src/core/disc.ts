/* Choosing a disc through the shell: a webview file input never gives the path the core needs. */
import {open} from '@tauri-apps/plugin-dialog';
import {t} from '../lib/strings';
import {inWindow} from './real';

export const canChoose = (): boolean => inWindow();

/** The chosen disc's path, or `null` when the window was closed without choosing one. */
export async function chooseDisc(): Promise<string | null> {
  const got = await open({multiple: false, directory: false,
                          filters: [{name: t.gameFile, extensions: ['iso']}]});
  return typeof got === 'string' ? got : null;
}
