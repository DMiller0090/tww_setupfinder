<!-- Emulators. Opens only when more than one is running. -->
<script lang="ts">
  import {t} from '../../lib/strings';
  import {place} from '../../lib/log.svelte';
  import {offered} from '../../core/live.svelte';
  import type {Emulator} from '../../core/fixtures';

  let {open = $bindable(false), take}:
    {open: boolean; take: (e: Emulator) => void} = $props();

  let win = $state<HTMLDialogElement | null>(null);
  let pid = $state<number | null>(null);

  $effect(() => {
    if (!win) return;
    if (open && !win.open) { pid = null; win.showModal(); place(); }
    if (!open && win.open) win.close();
  });

  const rows = $derived(offered());
  const chosen = $derived(rows.find(e => e.pid === pid && e.reads) ?? null);

  function go(e: Emulator): void {
    open = false;
    take(e);
  }
</script>

<dialog id="instWin" class="opts" bind:this={win}
        onclose={() => { open = false; setTimeout(place, 0); }}><div class="win">
  <header><h2>{t.emulators}</h2><span class="sp"></span>
    <button type="button" class="ghost icon" aria-label={t.close}
            onclick={() => open = false}>✕</button></header>
  <div class="rows">
    <table>
      <colgroup><col><col style="width:5.5rem"><col style="width:5.5rem"></colgroup>
      <thead><tr><th scope="col">{t.game}</th><th scope="col" class="n">{t.processCol}</th>
        <th scope="col">{t.stateCol}</th></tr></thead>
      <tbody>
        {#each rows as e (e.pid)}
          <!-- An unreadable game is listed but not a control: no focus, handlers or cursor. -->
          {#if !e.reads}
            <!-- `data-over` borrows the Moves window's muted, struck-through row style. -->
            <tr data-over="true" aria-disabled="true"><td>{e.game}</td><td class="n">{e.pid}</td>
              <td>{e.paused ? t.pausedWord : t.runningWord}</td></tr>
          {:else}
            <!-- svelte-ignore a11y_click_events_have_key_events
                 It has them: the row takes focus and Enter or Space chooses it. -->
            <tr tabindex="0" aria-selected={pid === e.pid} style="cursor:pointer"
                onclick={() => pid = e.pid}
                ondblclick={() => go(e)}
                onkeydown={(ev) => {
                  if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); pid = e.pid; }
                }}>
              <td>{e.game}</td><td class="n">{e.pid}</td>
              <td>{e.paused ? t.pausedWord : t.runningWord}</td></tr>
          {/if}
        {/each}
      </tbody>
    </table>
  </div>
  <footer>
    <span class="fact">{t.runningCount(rows.length)}</span>
    <span class="sp"></span>
    <button type="button" onclick={() => open = false}>{t.cancel}</button>
    <button type="button" class="on" disabled={!chosen}
            title={chosen ? undefined : t.chooseOneFirst}
            onclick={() => chosen && go(chosen)}>{t.connect}</button></footer>
</div></dialog>
