<!-- A List file's preview. X, Z and Facing each choose a column; the chosen ones are lit. -->
<script lang="ts">
  import {t} from '../../lib/strings';
  import {place} from '../../lib/log.svelte';
  import {give, rows, takes, ROLES, type List, type Role} from '../../core/list';

  let {open = $bindable(false), draft = $bindable(), take}:
    {open: boolean; draft: List; take: (list: List) => void} = $props();

  let win = $state<HTMLDialogElement | null>(null);

  $effect(() => {
    if (!win) return;
    if (open && !win.open) { win.showModal(); place(); }
    if (!open && win.open) win.close();
  });

  /* Only the top of the file is drawn; every row imports. */
  const SHOWN = 50;
  const body = $derived(draft.cells.slice(draft.head ? 1 : 0, (draft.head ? 1 : 0) + SHOWN));
  const width = $derived(Math.max(0, ...draft.cells.slice(0, SHOWN + 1).map(r => r.length)));
  const columns = $derived(Array.from({length: width}, (_, i) => i));
  const ready = $derived(rows(draft).length > 0);

  const LABEL: Record<Role, string> = {x: t.x, z: t.z, f: t.facing};
  /* A column is named by the file: its header, or with none its first value. */
  const nameOf = (c: number): string => draft.cells[0]?.[c] ?? '';
  const chosen = (c: number): boolean => ROLES.some(r => draft[r] === c);
  /* The template's selected-row colour, used for a chosen column. */
  const lit = (c: number): string | null => chosen(c) ? '#12314f' : null;
</script>

<dialog id="listWin" class="big" bind:this={win}
        onclose={() => { open = false; setTimeout(place, 0); }}><div class="win">
  <header><h2>{t.list}</h2><span class="sp"></span>
    <button type="button" class="ghost icon" aria-label={t.close}
            onclick={() => open = false}>✕</button></header>
  {#if draft.cells.length}
    <div class="filters">
      <!-- A select is as wide as its longest option, and a file's header can be a sentence. -->
      {#each ROLES as r (r)}
        <span style="display:flex;align-items:center;gap:.375rem;flex:none;margin-right:.5rem">
        <label for={'list-' + r}>{LABEL[r]}</label>
        <select id={'list-' + r} value={draft[r]} style="width:10rem"
                onchange={(e) => {
                  const c = Number(e.currentTarget.value);
                  if (c >= 0) give(draft, c, r); else draft[r] = -1;
                }}>
          <option value={-1}>{t.dash}</option>
          {#each columns as c (c)}<option value={c}>{nameOf(c)}</option>{/each}
        </select>
        </span>
      {/each}
    </div>
  {/if}
  <div class="rows">
    {#if !draft.cells.length}
      <div class="empty"><b>{t.stateEmpty}</b></div>
    {:else}
      <table>
        {#if draft.head}
          <thead><tr>{#each columns as c (c)}
            <th scope="col" style:background={lit(c)}
                style="text-transform:none;letter-spacing:0;font:400 .6875rem var(--mono);
                       overflow:hidden;text-overflow:ellipsis;white-space:nowrap"
              >{nameOf(c)}</th>
          {/each}</tr></thead>
        {/if}
        <tbody style="font:400 .6875rem var(--mono)">
          {#each body as r, i (i)}
            <tr style:opacity={takes(draft, r) ? null : '.42'}>
              {#each columns as c (c)}<td style:background={lit(c)}>{r[c] ?? ''}</td>{/each}</tr>
          {/each}
        </tbody>
      </table>
    {/if}
  </div>
  <footer><span class="sp"></span>
    <button type="button" onclick={() => open = false}>{t.cancel}</button>
    <button type="button" class="on" disabled={!ready}
            onclick={() => { take(draft); open = false; }}>{t.importWord}</button></footer>
</div></dialog>
