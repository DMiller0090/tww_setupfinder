<!-- Moves: types on the left, that type's moves on the right. `Show All` is every chosen move in
     search order. A type's checkbox sits outside its button, since a checkbox in a button cannot
     be checked. Moves over the presses cap stay listed, struck out. -->
<script lang="ts">
  import {t} from '../../lib/strings';
  import {place, say} from '../../lib/log.svelte';
  import {TYPES, COMBO_MAX, type Move, type MoveType} from '../../core/moves';
  import {ALL, catalog, chosen, inType, shownIn, swordType, typeLive, weigh, capped, total}
    from '../../core/catalog.svelte';
  import {keep} from '../../core/settings.svelte';

  /* Every catalogue change is saved through here. */
  function changed(): void {
    keep();
  }

  let {open = $bindable(false)}: {open: boolean} = $props();

  let win = $state<HTMLDialogElement | null>(null);
  let type = $state<string>(ALL);
  let q = $state('');
  let fMin = $state(''), fMax = $state('');
  let sortBy = $state<'frames' | 'name'>('frames');
  let capBox = $state(String(COMBO_MAX));

  $effect(() => {
    if (!win) return;
    if (open && !win.open) { win.showModal(); place(); }
    if (!open && win.open) win.close();
  });

  const isAll = $derived(type === ALL);
  const T = $derived(TYPES.find(x => x.k === type) ?? null);

  const passes = (m: Move): boolean => {
    const needle = q.trim().toLowerCase();
    const lo = parseFloat(fMin), hi = parseFloat(fMax);
    return (!needle || m.name.toLowerCase().includes(needle)) &&
      (!Number.isFinite(lo) || m.frames >= lo) &&
      (!Number.isFinite(hi) || m.frames <= hi);
  };

  /** Whether the drawn rows need a presses column. */
  const counts = (ms: Move[]): boolean => ms.some(m => m.presses);

  /* Show All is grouped by type; a type is a flat sorted list. */
  const groups = $derived.by(() => {
    if (!isAll) return [];
    return TYPES.map(x => ({T: x, rows: chosen(x).filter(passes)}))
      .filter(g => typeLive(g.T) > 0 && g.rows.length);
  });
  const rows = $derived.by(() => {
    if (isAll || !T) return [];
    const out = shownIn(T).filter(passes);
    return out.slice().sort((a, b) =>
      sortBy === 'frames' ? a.frames - b.frames : a.name.localeCompare(b.name));
  });
  const counted = $derived(isAll
    ? counts(groups.flatMap(g => g.rows))
    : counts(rows));
  const pressesRow = $derived(!isAll && !!T && shownIn(T).some(m => m.presses));

  function swordTo(to: 'away' | 'out' | 'both'): void {
    catalog.sword = to;
    changed();
    /* Leave a type the filter emptied. */
    if (T && typeLive(T) === 0) {
      const first = TYPES.find(x => typeLive(x) > 0);
      type = first ? first.k : ALL;
    }
  }

  function allTo(on: boolean): void {
    if (!T) return;
    for (const m of inType(T)) m.on = on;
    changed();
  }

  function setFrames(m: Move, raw: string): void {
    const v = parseInt(raw, 10);
    if (!Number.isFinite(v) || v < 1) {
      say('error', t.needsWholeFrames(m.name));
      return;
    }
    m.frames = v;
    changed();
  }

  function setCap(raw: string): void {
    const v = parseInt(raw, 10);
    if (!Number.isFinite(v) || v < 1 || v > COMBO_MAX) {
      say('error', t.pressesRange(COMBO_MAX));
      capBox = String(catalog.cap);
      return;
    }
    catalog.cap = v;
    changed();
  }

  const typeOn = (x: MoveType): number => weigh(chosen(x));
</script>

<dialog id="movesWin" class="big" bind:this={win}
        onclose={() => { open = false; setTimeout(place, 0); }}><div class="win">
  <header>
    <h2>{t.moves}</h2>
    <span class="cap" id="swordCap">{t.sword}</span>
    <span class="seg" role="group" aria-labelledby="swordCap">
      <button type="button" aria-pressed={catalog.sword === 'away'}
              onclick={() => swordTo('away')}>{t.swordAway}</button>
      <button type="button" aria-pressed={catalog.sword === 'out'}
              onclick={() => swordTo('out')}>{t.swordOut}</button>
      <button type="button" aria-pressed={catalog.sword === 'both'}
              onclick={() => swordTo('both')}>{t.swordBoth}</button>
    </span>
    <span class="sp"></span>
    <label class="hid" for="allSearch">{t.search}</label>
    <input type="search" id="allSearch" bind:value={q} placeholder={t.search}
           style="max-width:15rem">
    <button type="button" class="ghost icon" aria-label={t.close}
            onclick={() => open = false}>✕</button>
  </header>
  <div class="split">
    <div class="list">
      <div class="typerow all">
        <button type="button" class="listrow" aria-current={isAll} onclick={() => type = ALL}>
          <span class="n">{t.showAll}</span><span class="c">{total().toLocaleString()}</span>
        </button>
      </div>
      {#each [[t.sword, true], [t.noSword, false]] as const as [said, needs] (said)}
        <div class="band">{said}</div>
        {#each TYPES.filter(x => swordType(x) === needs) as x (x.k)}
          {@const live = typeLive(x)}
          {@const on = typeOn(x)}
          <div class="typerow">
            <input type="checkbox" checked={Boolean(live && on)}
                   indeterminate={Boolean(live && on && on < live)} disabled={live === 0}
                   aria-label={x.n}
                   onchange={(e) => {
                     const to = (e.currentTarget as HTMLInputElement).checked;
                     for (const m of inType(x)) m.on = to;
                     changed();
                   }}>
            <button type="button" class="listrow" data-empty={live === 0} disabled={live === 0}
                    aria-current={type === x.k} onclick={() => type = x.k}>
              <span class="n">{x.n}</span>
              <span class="c">{live ? `${on.toLocaleString()}/${live.toLocaleString()}` : t.none}</span>
            </button>
          </div>
        {/each}
      {/each}
    </div>

    <div class="movelist">
      <div class="filters">
        <span class="cap">{t.frames}</span>
        <label class="hid" for="fMin">{t.fewestFrames}</label>
        <input type="number" id="fMin" bind:value={fMin} placeholder={t.min}>
        <label class="hid" for="fMax">{t.mostFrames}</label>
        <input type="number" id="fMax" bind:value={fMax} placeholder={t.max}>
        <span class="cap">{t.sort}</span>
        <label class="hid" for="sortBy">{t.sort}</label>
        <select id="sortBy" bind:value={sortBy} disabled={isAll}
                title={isAll ? t.searchDecidesOrder : undefined}>
          <option value="frames">{t.frames}</option><option value="name">{t.name}</option></select>
        <span class="num" hidden={!pressesRow}><label for="presses">{t.presses}</label>
          <input type="number" id="presses" min="1" max={COMBO_MAX} bind:value={capBox}
                 onchange={() => setCap(capBox)}></span>
        <span class="sp"></span>
        <button type="button" disabled={isAll} title={isAll ? t.chooseTypeFirst : undefined}
                onclick={() => allTo(true)}>{t.all}</button>
        <button type="button" disabled={isAll} title={isAll ? t.chooseTypeFirst : undefined}
                onclick={() => allTo(false)}>{t.none}</button>
      </div>
      <div class="rows">
        {#if isAll && !groups.length}
          <div class="empty" title={t.noMovesChosen}><b>{t.stateEmpty}</b></div>
        {:else if !isAll && !rows.length}
          <div class="empty"><b>{t.noMatch}</b></div>
        {:else}
          <table class="movetab">
            <colgroup><col style="width:2rem"><col style="width:1px">
              {#if counted}<col style="width:1px">{/if}<col></colgroup>
            <thead><tr><th scope="col"><span class="hid">{t.chosenCol}</span></th>
              <th scope="col">{t.move}</th>
              {#if counted}<th scope="col" class="n">{t.presses}</th>{/if}
              <th scope="col" class="n">{t.frames}</th></tr></thead>
            <tbody>
              {#if isAll}
                {#each groups as g (g.T.k)}
                  <tr class="grp"><td colspan={counted ? 4 : 3}>{g.T.n}
                    <b>{weigh(g.rows).toLocaleString()}</b></td></tr>
                  {#each g.rows as m (m.id)}
                    {@render row(m)}
                  {/each}
                {/each}
              {:else}
                {#each rows as m (m.id)}
                  {@render row(m)}
                {/each}
              {/if}
            </tbody>
          </table>
        {/if}
      </div>
    </div>
  </div>
  <footer>
    <span class="fact">{t.movesCount(total())}</span>
    <span class="sp"></span>
    <button type="button" class="on" onclick={() => open = false}>{t.done}</button>
  </footer>
</div></dialog>

{#snippet row(m: Move)}
  {@const out = capped(m)}
  <tr data-over={out ? 'true' : undefined}>
    <td><input type="checkbox" bind:checked={m.on} disabled={out} aria-label={m.name}
               onchange={changed}></td>
    <td title={m.name}>{m.name}</td>
    {#if counted}<td class="n">{m.presses ?? ''}</td>{/if}
    <td class="n"><input type="number" value={m.frames} min="1" disabled={out || m.fixed}
                         aria-label={t.framesOf(m.name)}
                         onchange={(e) => setFrames(m, (e.currentTarget as HTMLInputElement).value)}></td>
  </tr>
{/snippet}
