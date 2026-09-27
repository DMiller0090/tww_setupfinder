<!-- Stage and room chooser. -->
<script lang="ts">
  import {t} from '../../lib/strings';
  import {place} from '../../lib/log.svelte';
  import type {RoomIndex} from '../../core/fixtures';

  let {open = $bindable(false), rooms, picked, choose}:
    {open: boolean; rooms: RoomIndex[]; picked: RoomIndex | null;
     choose: (it: RoomIndex) => void} = $props();

  let win = $state<HTMLDialogElement | null>(null);
  let q = $state('');
  let stage = $state<string | null>(null);

  $effect(() => {
    if (!win) return;
    if (open && !win.open) { win.showModal(); place(); }
    if (!open && win.open) win.close();
  });

  const stages = $derived.by(() => {
    const by = new Map<string, RoomIndex[]>();
    for (const r of rooms) {
      const got = by.get(r.stage);
      if (got) got.push(r); else by.set(r.stage, [r]);
    }
    return [...by.entries()].map(([name, its]) => ({name, its}));
  });
  const shown = $derived(stages.filter(s =>
    !q.trim() || s.name.toLowerCase().includes(q.trim().toLowerCase())));
  const on = $derived(stage ?? picked?.stage ?? stages[0]?.name ?? null);
  const its = $derived(stages.find(s => s.name === on)?.its ?? []);
</script>

<dialog id="stageWin" class="big" bind:this={win}
        onclose={() => { open = false; setTimeout(place, 0); }}><div class="win">
  <header>
    <h2>{t.stage}</h2>
    <span class="sp"></span>
    <label class="hid" for="stageSearch">{t.search}</label>
    <input type="search" id="stageSearch" bind:value={q} placeholder={t.search}
           style="max-width:15rem">
    <button type="button" class="ghost icon" aria-label={t.close}
            onclick={() => open = false}>✕</button>
  </header>
  <div class="split">
    <div class="list">
      {#each shown as s (s.name)}
        <button type="button" class="listrow" aria-current={on === s.name}
                onclick={() => stage = s.name}>
          <span class="n">{s.name}</span><span class="c">{s.its.length}</span></button>
      {:else}
        <div class="empty"><b>{t.noMatch}</b></div>
      {/each}
    </div>
    <div class="movelist"><div class="rows">
      <table>
        <colgroup><col style="width:4rem"><col><col style="width:5rem"></colgroup>
        <thead><tr><th scope="col" class="n">{t.room}</th><th scope="col">{t.stage}</th>
          <th scope="col" class="n">{t.wallsCol}</th></tr></thead>
        <tbody>
          {#each its as r (r.stage + r.room)}
            <!-- svelte-ignore a11y_click_events_have_key_events
                 It has them: the row takes focus and Enter or Space chooses it. -->
            <tr style="cursor:pointer" tabindex="0" aria-current={picked?.stage === r.stage &&
                  picked?.room === r.room}
                onclick={() => { choose(r); open = false; }}
                onkeydown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); choose(r); open = false; }
                }}>
              <td class="n">{r.room}</td><td>{r.stage}</td>
              <td class="n">{r.counts.wall.toLocaleString()}</td></tr>
          {/each}
        </tbody>
      </table>
    </div></div>
  </div>
  <footer>
    <span class="fact">{on ? t.stageRooms(on, its.length) : t.nothingChosen}</span>
    <span class="sp"></span>
    <button type="button" class="on" onclick={() => open = false}>{t.done}</button>
  </footer>
</div></dialog>
