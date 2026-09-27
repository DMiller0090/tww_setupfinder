import {defineConfig} from 'vite';
import {svelte} from '@sveltejs/vite-plugin-svelte';

/* Static files only; the client must not assume a server. */
export default defineConfig({
  plugins: [svelte()],
  base: './',
  server: {
    port: 5180,
    strictPort: true,
    /* The watcher dies with EBUSY on files the Rust build writes under `src-tauri/target`. */
    watch: {ignored: ['**/src-tauri/**']},
  },
});
