<!-- Confirm dialog for a destructive action. -->
<script lang="ts">
  import {t} from '../../lib/strings';
  import {place} from '../../lib/log.svelte';

  let {open = $bindable(false), title = '', line = '', yes = '', confirm}:
    {open: boolean; title?: string; line?: string; yes?: string; confirm: () => void} = $props();

  let win = $state<HTMLDialogElement | null>(null);
  $effect(() => {
    if (!win) return;
    if (open && !win.open) { win.showModal(); place(); }
    if (!open && win.open) win.close();
  });
</script>

<dialog id="askWin" class="ask" aria-labelledby="askTitle" bind:this={win}
        onclose={() => { open = false; setTimeout(place, 0); }}><div class="win">
  <div class="askbody">
    <svg class="warn" viewBox="0 0 24 24" width="28" height="28" role="img"
         aria-label={t.warning} fill="none" stroke="currentColor" stroke-width="1.8"
         stroke-linecap="round" stroke-linejoin="round">
      <path d="M12 3.6 1.9 20.4h20.2L12 3.6Z"></path>
      <path d="M12 10v4.6"></path><path d="M12 17.6h.01"></path></svg>
    <div><h2 id="askTitle">{title}</h2><p id="askLine">{line}</p></div>
  </div>
  <footer><span class="sp"></span>
    <button type="button" onclick={() => open = false}>{t.cancel}</button>
    <button type="button" class="danger"
            onclick={() => { open = false; confirm(); }}>{yes || t.clear}</button></footer>
</div></dialog>
