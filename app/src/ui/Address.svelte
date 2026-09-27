<!-- Address shape: three axes of four hex bytes; a blank byte is null. -->
<script lang="ts">
  import {t} from '../lib/strings';
  import {say} from '../lib/log.svelte';
  import {AXES, hex2, spanOf} from '../core/address';
  import {settings as store, keep} from '../core/settings.svelte';

  let {settings = store}: {settings?: typeof store} = $props();

  /* The focused box shows the raw draft, not the padded byte: padding while typing would turn `5`
     into `05` and maxlength would block the second character. */
  let caret: string | null = $state(null);
  let draft = $state('');
  const key = (ax: number, i: number): string => `${ax}:${i}`;
  const settled = (b: number | null): string => b === null ? '' : hex2(b);
  const shown = (ax: number, i: number, b: number | null): string =>
    caret === key(ax, i) ? draft : settled(b);

  function typed(ax: number, i: number, box: HTMLInputElement): void {
    const raw = box.value.trim();
    draft = box.value;
    if (raw === '') {
      settings.addr[ax][i] = null;
    } else if (/^[0-9a-fA-F]{1,2}$/.test(raw)) {
      settings.addr[ax][i] = parseInt(raw, 16);
    } else {
      say('error', t.notHex(AXES[ax] + (i + 1)));
      const had = settings.addr[ax][i];
      draft = settled(had);
      box.value = draft;
      return;
    }
    keep();
  }

  const values = $derived(settings.addr.map((bytes, ax) => {
    const s = spanOf(bytes);
    if ('exact' in s) return [[AXES[ax], String(s.exact)]];
    if ('none' in s) return [[AXES[ax], t.noNumber]];
    return [[`${AXES[ax]} ${t.minWord}`, String(s.lo)], [`${AXES[ax]} ${t.maxWord}`, String(s.hi)]];
  }));
</script>

<div id="shapeAddr">
  {#each settings.addr as bytes, ax (ax)}
    <div class="ax">
      {#each values[ax] as [tag, value] (tag)}
        <div class="v"><b>{tag}</b><span translate="no">{value}</span></div>
      {/each}
      <div class="bytes">
        {#each bytes as b, i (i)}
          <input type="text" maxlength="2" spellcheck="false" autocomplete="off" translate="no"
                 class={b === null ? 'blank' : ''}
                 title={b === null ? t.blankByte : undefined}
                 aria-label={AXES[ax] + (i + 1)}
                 value={shown(ax, i, b)}
                 oninput={(e) => typed(ax, i, e.currentTarget as HTMLInputElement)}
                 onfocus={(e) => {
                   caret = key(ax, i);
                   draft = settled(settings.addr[ax][i]);
                   (e.currentTarget as HTMLInputElement).select();
                 }}
                 onmouseup={(e) => e.preventDefault()}
                 onblur={() => (caret = null)}
                 onchange={() => (draft = settled(settings.addr[ax][i]))}>
        {/each}
      </div>
    </div>
  {/each}
</div>
