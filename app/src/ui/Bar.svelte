<!-- Top bar. -->
<script lang="ts">
  import {t} from '../lib/strings';
  let {moveCount, where, connected = false, busy = false, canConnect = true, canStage = true,
       canAttach = false, stage, options, moves, conn, attach}:
    {moveCount: number; where: string; connected?: boolean; busy?: boolean;
     canConnect?: boolean; canStage?: boolean; canAttach?: boolean; stage: () => void;
     options: () => void; moves: () => void; conn: () => void; attach: () => void} = $props();
</script>

<header class="bar">
  <button type="button" onclick={moves}
    >{t.moves}<b class="count">{moveCount.toLocaleString()}</b></button>
  <button type="button" onclick={options}>{t.options}</button>
  <span class="sp"></span>
  <!-- Shortcut to the Options disc row; gone once a disc is attached. -->
  {#if canAttach}
    <button type="button" class="attach" onclick={attach}>{t.attachIso}</button>
  {/if}
  <!-- Disabled with no game or disc: there are no rooms to list. -->
  <button type="button" class="stagebtn" disabled={!canStage}
          onclick={stage} title={canStage ? t.chooseStage : undefined}>{where}</button>
  <!-- Connect/disconnect toggle; disabled without a supported game. -->
  <button type="button" disabled={busy || !canConnect}
          title={busy ? t.working : undefined}
          onclick={conn}>{connected ? t.disconnect : t.connect}</button>
</header>
