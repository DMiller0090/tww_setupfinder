<!-- Floating messages over the window; each is also kept in the logs. A note that stays is never
     also a routing button: no control inside a control. -->
<script lang="ts">
  import {t} from '../lib/strings';
  import {notes, close, place} from '../lib/log.svelte';

  let {logs}: {logs: () => void} = $props();

  /* Focus and select the named control so typing replaces the refused value. */
  function go(id: string | null): void {
    if (!id) return;
    const at = document.getElementById(id) as HTMLInputElement | null;
    setTimeout(() => {
      at?.focus();
      at?.select?.();
      at?.scrollIntoView({block: 'nearest'});
    }, 0);
  }
</script>

<div class="msgs" role="log" aria-live="polite" aria-label={t.messages}>
  {#each notes as note (note.key)}
    {#if note.opensLogs}
      <button type="button" class="msg" data-sev={note.sev} data-route title={t.goToIt}
              onclick={() => { logs(); close(note.key); }}>
        <span class="msev">{note.sev}</span><span class="mt">{note.text}</span></button>
    {:else if note.reach}
      <button type="button" class="msg" data-sev={note.sev} data-route title={t.goToIt}
              onclick={() => { go(note.reach); close(note.key); }}>
        <span class="msev">{note.sev}</span><span class="mt">{note.text}</span></button>
    {:else}
      <div class="msg" data-sev={note.sev} data-stay={note.stay ? '' : undefined}>
        <span class="msev">{note.sev}</span><span class="mt">{note.text}</span>
        {#if note.stay}
          <button type="button" class="ghost icon mclose" aria-label={t.close}
                  onclick={() => close(note.key)}>✕</button>
        {/if}
      </div>
    {/if}
  {/each}
</div>

<svelte:window onresize={place} />
