<!-- Options: machine and disc settings. Values live in `core/settings.svelte.ts`, saved as changed. -->
<script lang="ts">
  import {t} from '../../lib/strings';
  import {place, say} from '../../lib/log.svelte';
  import {canChoose} from '../../core/disc';
  import {live} from '../../core/live.svelte';
  import {settings, keep, tookHex} from '../../core/settings.svelte';

  /* The disc path is taken on `change`, not bound: reading a disc walks every room on it. */
  let {open = $bindable(false), browse, took, askTo}:
    {open: boolean; browse: () => void; took: () => void;
     askTo: (title: string, line: string, yes: string, run: () => void) => void} =
    $props();

  let win = $state<HTMLDialogElement | null>(null);
  let info = $state('');
  let looking = $state(false);
  /* Placeholder sizes: the core does not report storage yet. `as string` widens the const literal. */
  let planBytes = $state(t.planBytes as string), solBytes = $state(t.solBytes as string);


  $effect(() => {
    if (!win) return;
    if (open && !win.open) { win.showModal(); place(); keepCores(); }
    if (!open && win.open) win.close();
  });

  /* The foot line describes the row under the cursor or caret. */
  const tell = (what: string) => (): void => { info = what; };
  const clear = (): void => { info = ''; };

  /* Empty or negative falls back to 4; no ceiling. */
  function keepRange(): void {
    const want = Number(settings.checkRange);
    settings.checkRange = !(want >= 0) ? 4 : want;
    keep();
  }

  /* Clamp to 1..machine cores; min/max only bind the arrows. Also run on open, since the saved
     value and the core count load in a race at start-up. */
  function keepCores(): void {
    const want = Math.floor(Number(settings.cores));
    settings.cores = !(want >= 1) ? 1 : Math.min(want, live.cores);
    keep();
  }
</script>

<dialog id="optWin" class="opts" bind:this={win}
        onclose={() => { open = false; setTimeout(place, 0); }}>
  <div class="win">
  <header><h2>{t.options}</h2><span class="sp"></span>
    <button type="button" class="ghost icon" aria-label={t.close}
            onclick={() => open = false}>✕</button></header>
  <div class="set">
    <div>
      <h3>{t.machine}</h3>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoGame)} onpointerleave={clear}
           onfocusin={tell(t.infoGame)} onfocusout={clear}>
        <label for="isoPath">{t.game}</label>
        <input type="text" id="isoPath" value={settings.iso} placeholder={t.noFile}
               autocomplete="off" spellcheck="false" translate="no"
               onchange={(e) => {
                 settings.iso = (e.currentTarget as HTMLInputElement).value.trim();
                 keep();
                 took();
               }}>
        <!-- Disabled where no native file dialog is available. -->
        <button type="button" class="mini" disabled={!canChoose()}
                onclick={browse}>{t.browse}</button></div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoCores)} onpointerleave={clear}
           onfocusin={tell(t.infoCores)} onfocusout={clear}>
        <label for="lCo">{t.cores}</label>
        <input type="number" id="lCo" min="1" max={live.cores}
               bind:value={settings.cores} onchange={keepCores}><span class="fact"
          >{t.ofCores(live.cores)}</span></div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={clear} onpointerleave={clear}
           onfocusin={clear} onfocusout={clear}>
        <label for="lCr">{t.checkRange}</label>
        <input type="number" id="lCr" min="0" step="any"
               bind:value={settings.checkRange} onchange={keepRange}></div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoCameraChecks)} onpointerleave={clear}
           onfocusin={tell(t.infoCameraChecks)} onfocusout={clear}>
        <span class="check"><input type="checkbox" id="cameraChecks"
            bind:checked={settings.cameraChecks} onchange={keep}>
          <label for="cameraChecks">{t.cameraChecks}</label></span></div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoHex)} onpointerleave={clear}
           onfocusin={tell(t.infoHex)} onfocusout={clear}>
        <span class="check"><input type="checkbox" id="hex" bind:checked={settings.hex} onchange={tookHex}>
          <label for="hex">{t.hex}</label></span></div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoKeepMoves)} onpointerleave={clear}
           onfocusin={tell(t.infoKeepMoves)} onfocusout={clear}>
        <span class="check"><input type="checkbox" id="keepMoves" bind:checked={settings.keepMoves} onchange={keep}>
          <label for="keepMoves">{t.saveMoves}</label></span></div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoRestore)} onpointerleave={clear}
           onfocusin={tell(t.infoRestore)} onfocusout={clear}>
        <span class="check"><input type="checkbox" id="restore" bind:checked={settings.restore} onchange={keep}>
          <label for="restore">{t.saveSession}</label></span></div>
    </div>
    <div>
      <h3>{t.disc}</h3>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoPlans)} onpointerleave={clear}
           onfocusin={tell(t.infoPlans)} onfocusout={clear}>
        <span class="check"><input type="checkbox" id="keepPlans" bind:checked={settings.keepPlans}
            onchange={() => { keep();
              say(settings.keepPlans ? 'info' : 'warning',
                  settings.keepPlans ? t.plansKept : t.plansNotKept); }}>
          <label for="keepPlans">{t.plans}</label></span>
        <span class="sp"></span>
        <span class="fact"><b>{planBytes}</b></span>
        <button type="button"
                onclick={() => askTo(t.clearPlans, t.cannotUndo, t.clear,
                  () => { planBytes = t.noBytes; say('info', t.plansCleared); })}
          >{t.clear}</button>
      </div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f" onpointerenter={tell(t.infoSolutions)} onpointerleave={clear}
           onfocusin={tell(t.infoSolutions)} onfocusout={clear}>
        <span class="check" style="color:var(--muted)">{t.solutions}</span>
        <span class="sp"></span>
        <span class="fact"><b>{solBytes}</b></span>
        <button type="button"
                onclick={() => askTo(t.clearSolutions, t.cannotUndo, t.clear,
                  () => { solBytes = t.noBytes; say('info', t.solutionsCleared); })}
          >{t.clear}</button>
      </div>
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="f apart" onpointerenter={tell(t.infoUpdates)} onpointerleave={clear}
           onfocusin={tell(t.infoUpdates)} onfocusout={clear}>
        <label for="checkUpd">{t.updates}</label>
        <span class="fact">{t.version}</span>
        <span class="sp"></span>
        <button type="button" class="mini" id="checkUpd" disabled={looking}
                title={looking ? t.looking : undefined}
                onclick={() => {
                  looking = true;
                  setTimeout(() => { looking = false; say('info', t.noNewerVersion); }, 700);
                }}>{t.check}</button></div>
    </div>
  </div>
  <footer>
    <span class="info">{info}</span>
    <span class="sp"></span>
    <button type="button" class="on" onclick={() => open = false}>{t.done}</button>
  </footer>
</div></dialog>
