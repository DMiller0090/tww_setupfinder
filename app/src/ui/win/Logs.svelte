<!-- Logs, newest first. -->
<script lang="ts">
  import {t} from '../../lib/strings';
  import {log, place} from '../../lib/log.svelte';

  let {open = $bindable(false)}: {open: boolean} = $props();
  let win = $state<HTMLDialogElement | null>(null);
  let copied = $state(false);

  const clock = new Intl.DateTimeFormat(undefined,
    {hour: '2-digit', minute: '2-digit', second: '2-digit'});

  $effect(() => {
    if (!win) return;
    if (open && !win.open) { win.showModal(); place(); }
    if (!open && win.open) win.close();
  });

  const rows = $derived(log.slice().reverse());

  async function copy(): Promise<void> {
    const all = log.map(e => `${clock.format(e.at)}\t${e.sev}\t${e.text}`).join('\n');
    try {
      await navigator.clipboard.writeText(all);
      copied = true;
      setTimeout(() => { copied = false; }, 1200);
    } catch { copied = false; }
  }
</script>

<dialog id="logsWin" class="big" bind:this={win}
        onclose={() => { open = false; setTimeout(place, 0); }}><div class="win">
  <header><h2>{t.logs}</h2>
    <span class="fact">{log.length === 1 ? t.oneLine : t.lines(log.length)}</span>
    <span class="copied" style:opacity={copied ? 1 : 0}>{t.copied}</span>
    <button type="button" class="ghost icon" aria-label={t.copy} onclick={copy}><svg
      viewBox="0 0 16 16" width="13" height="13" aria-hidden="true" fill="none"
      stroke="currentColor" stroke-width="1.4" stroke-linecap="round"
      stroke-linejoin="round"><rect x="5.75" y="5.75" width="7.5" height="8.5"
      rx="1.5"></rect><path d="M10.25 3.25H4.5a1.25 1.25 0 0 0-1.25 1.25v6.25"></path
      ></svg></button><span class="sp"></span>
    <button type="button" class="ghost icon" aria-label={t.close}
            onclick={() => open = false}>✕</button></header>
  <div class="rows">
    {#if !log.length}
      <div class="empty" title={t.logsEmptyWhy}><b>{t.stateEmpty}</b></div>
    {:else}
      <table class="logtab">
        <colgroup><col style="width:5.5rem"><col style="width:5rem"><col></colgroup>
        <thead><tr><th scope="col">{t.time}</th><th scope="col">{t.type}</th>
          <th scope="col">{t.details}</th></tr></thead>
        <tbody>
          {#each rows as e (e.at.getTime() + e.text)}
            <tr><td class="t">{clock.format(e.at)}</td>
              <td class="msg-sev" data-sev={e.sev}>{e.sev}</td>
              <td class="w">{e.text}</td></tr>
          {/each}
        </tbody>
      </table>
    {/if}
  </div>
  <footer><span class="sp"></span>
    <button type="button" class="on" onclick={() => open = false}>{t.done}</button></footer>
</div></dialog>
