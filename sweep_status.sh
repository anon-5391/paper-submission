#!/bin/bash
# sweep_status.sh — quick live status for the 8-way parameter sweep
# (disc_dia, backbone_outer_dia, offset, inner_dia x tendon_count 3/4).
#
# One-shot:   bash sweep_status.sh
# Continuous: bash sweep_status.sh --watch [interval_seconds]   (default 30s;
#             refreshes in place, Ctrl+C to stop; auto-stops once the sweep
#             itself finishes)
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

print_status() {
    echo "================================================================"
    echo "  SWEEP STATUS — $(date '+%Y-%m-%d %H:%M:%S')"
    echo "================================================================"

    echo
    echo "── processes ──"
    n_drivers=$(pgrep -fc "sweep_cli.py --type cad" 2>/dev/null); n_drivers=${n_drivers:-0}
    n_workers=$(pgrep -fc "runSofa" 2>/dev/null); n_workers=${n_workers:-0}
    echo "  sweep_cli.py drivers : $n_drivers  (expect 8 while running, 0 when all done)"
    echo "  runSofa workers      : $n_workers"
    uptime | sed 's/^/  /'

    echo
    echo "── designs meshed ──"
    n_mesh=$(ls vtk/{RD,BD,OF,CD}_*_t[34].vtk 2>/dev/null | wc -l)
    echo "  $n_mesh / 24 designs have a VTK mesh"
    missing=$(comm -23 \
      <(for p in RD_26 RD_30 RD_36 BD_22 BD_25 BD_28 OF_1 OF_2 OF_3 CD_11 CD_14 CD_17; do
          for t in t3 t4; do echo "${p}_${t}"; done; done | sort) \
      <(ls vtk/*_t[34].vtk 2>/dev/null | sed -E 's#vtk/(.*)\.vtk#\1#' | sort))
    [ -n "$missing" ] && { echo "  not yet meshed (or failed):"; echo "$missing" | sed 's/^/    /'; }

    echo
    echo "── pulls completed (settled vs timed-out) by design ──"
    printf "  %-10s %6s %10s %10s\n" "design" "target" "settled" "timeout"
    total_target=0; total_settled=0; total_timeout=0
    for f in results/{RD,BD,OF,CD}_*_t[34].csv; do
      [ -f "$f" ] || continue
      did=$(basename "$f" .csv)
      ncab=3; [[ "$did" == *_t4 ]] && ncab=4
      target=$((ncab * 10))
      settled=$(awk -F',' 'NR>1 && $4!=0 && $6==1' "$f" | wc -l)
      timeout=$(awk -F',' 'NR>1 && $4!=0 && $6==0' "$f" | wc -l)
      printf "  %-10s %6d %10d %10d\n" "$did" "$target" "$settled" "$timeout"
      total_target=$((total_target+target))
      total_settled=$((total_settled+settled))
      total_timeout=$((total_timeout+timeout))
    done

    echo
    echo "── totals ──"
    done_n=$((total_settled+total_timeout))
    echo "  $done_n / $total_target pulls done  (settled=$total_settled, timeout=$total_timeout)"
    if [ "$total_target" -gt 0 ]; then
      pct=$((done_n * 100 / total_target))
      echo "  ${pct}% complete"
    fi
    if [ "$n_drivers" -eq 0 ] && [ "$n_workers" -eq 0 ]; then
      echo "  *** SWEEP FINISHED (no drivers or workers running) ***"
    fi
}

if [ "${1:-}" = "--watch" ]; then
    interval="${2:-30}"
    while true; do
        [ -t 1 ] && clear
        print_status
        if [ "$n_drivers" -eq 0 ] && [ "$n_workers" -eq 0 ]; then
            echo
            echo "  (sweep finished — stopping refresh)"
            break
        fi
        echo
        echo "  refreshing every ${interval}s — Ctrl+C to stop"
        sleep "$interval"
    done
    exit 0
fi

print_status
