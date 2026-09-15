# -*- coding: utf-8 -*-
import Sofa.Core
import csv
import os
import time
import numpy as np

# Optional real-time freeze after each settled log, purely for capturing a
# screenshot of the rendered GUI at that exact pose (paper figures etc). 0 by
# default -> zero behavior change for headless/batch runs.
SNAPSHOT_PAUSE_S = float(os.environ.get("TDCR_SNAPSHOT_PAUSE_S", "0"))

from pipeline_config import (
    CABLE_VALUE_TYPE,
    CABLE_PULL_LEVELS,
    REST_CAPTURE_FRAME,
    SCENE_FORCE_PER_NEWTON,
    SETTLE_TIP_THRESHOLD_MM,
    SETTLE_CONSECUTIVE_STEPS,
    SETTLE_MIN_STEPS,
    SETTLE_MAX_STEPS_PULL,
    SPINE_SEGMENTS,
)
from bending_utils import (
    SpineTracker,
    bend_angle_deg,
    bend_plane_normal,
    curvature_constancy,
)


class CableController(Sofa.Core.Controller):
    """
    TDCR spine-point collector — single-cable mode.

    Fresh SOFA process per cable (TDCR_CABLE_INDEX selects which one, default 0).
    Pull that cable at each tension level, settle, log, exit. No release phase.

    Each row logs the raw rest + deformed spine points plus a bend_angle_deg;
    other bend metrics can be derived offline from the same points.
    """

    def __init__(self, *args, cables, soft_body, design_name,
                 tendon_radius=1.0, robot_length=None, backbone_length=None,
                 backbone_axis=None, tension_levels=None, output_csv=None,
                 notch_positions=None, spine_probe=None, **kwargs):
        kwargs.setdefault("name", "CableController")
        super().__init__(*args, **kwargs)

        self.cables        = cables
        self.soft_body     = soft_body
        self.design_name   = design_name
        self.backbone_axis = backbone_axis
        self.output_csv    = output_csv
        # Notch (flex-hinge) Y-positions from the CAD's dims.json sidecar, if any;
        # the spine is sampled there. None falls back to an evenly-spaced default.
        self.notch_positions = notch_positions
        # Nominal backbone_length from the sweep args, recorded per-row so a
        # stale/duplicated file is self-evident. Falls back to measured robot_length.
        self.backbone_length = backbone_length if backbone_length is not None else robot_length

        self.levels = list(tension_levels) if tension_levels else list(CABLE_PULL_LEVELS)

        self.spine         = SpineTracker(soft_body, n_segments=SPINE_SEGMENTS,
                                          probe=spine_probe)
        self.rest_captured = False
        self.frame_count   = 0

        self.seq_idx    = 0
        self.phase_step = 0
        self.prev_tip   = None
        self.streak     = 0
        self.last_delta = 0.0
        self.done       = False

        # TDCR_CABLE_INDEX selects which cable this process pulls (default 0).
        cable_env = os.environ.get("TDCR_CABLE_INDEX")
        self.cable_idx = int(cable_env) if cable_env is not None else 0

        self.sequence = list(self.levels)

        # CSV setup is deferred to _open_csv() — the header has one column per
        # spine point, and the point count is only known after capture_rest.
        self._fieldnames = None
        self._csv_file   = None
        self._csv_writer = None

        print(f"[CTRL] {design_name} | single-cable mode  cable={self.cable_idx+1}  "
              f"{len(self.levels)} tension(s) = {len(self.sequence)} row(s)")
        if self.output_csv:
            print(f"[CTRL] log → {self.output_csv}")

    # ── CSV schema ──────────────────────────────────────────────────────────────

    def _build_fieldnames(self, n_spine):
        """Column order: metadata, then flattened rest spine, then deformed spine.
        Point index i runs base(0) → tip(n-1)."""
        cols = ["design", "backbone_length", "cable",
                "tension_newton", "displacement_mm", "settled", "n_spine",
                "tip_x", "tip_y", "tip_z",
                "bend_angle_deg",
                # constant-curvature diagnostics (kappa_cv, cc_rmse_norm ~0 for a circular arc)
                "arc_length_mm", "radius_mm", "kappa_mean_permm",
                "kappa_cv", "cc_rmse_mm", "cc_rmse_norm",
                "plane_nx", "plane_ny", "plane_nz",
                "true_base_x", "true_base_y", "true_base_z",
                "true_tip_x", "true_tip_y", "true_tip_z"]
        for i in range(n_spine):
            cols += [f"rest_x{i}", f"rest_y{i}", f"rest_z{i}"]
        for i in range(n_spine):
            cols += [f"def_x{i}", f"def_y{i}", f"def_z{i}"]
        return cols

    def _open_csv(self):
        """Open the CSV and write the header once the spine count is known."""
        if not self.output_csv:
            return
        self._fieldnames = self._build_fieldnames(self.spine.n_spine)
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        # Drop this process's own (design, cable, tension) rows so a re-run
        # overwrites them; rows from other designs/cables/tensions are kept.
        self._prune_own_rows()
        self._csv_file = open(self.output_csv, "a", newline="")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=self._fieldnames)
        if self._csv_file.tell() == 0:
            self._csv_writer.writeheader()
            self._csv_file.flush()

    # ── helpers ───────────────────────────────────────────────────────────────

    def _prune_own_rows(self):
        """Remove this process's own rows (same design + cable + each pull level)
        from an existing CSV so a re-run replaces its output. No-op if the file
        doesn't exist.

        Match key depends on CABLE_VALUE_TYPE: "force" matches tension_newton,
        "displacement" matches displacement_mm on proximity (a hard constraint
        can be force-logged tens of mm short of its target)."""
        path = self.output_csv
        if not path or not os.path.exists(path) or os.path.getsize(path) == 0:
            return

        # An old CSV whose header predates a column change can't be appended to —
        # discard it and let _open_csv write a fresh one.
        with open(path, newline="") as f:
            existing_header = next(csv.reader(f), [])
        if self._fieldnames and existing_header and existing_header != list(self._fieldnames):
            print(f"[CTRL] {os.path.basename(path)}: schema changed — rewriting from scratch")
            os.remove(path)
            return

        my_cable = str(self.cable_idx + 1)
        if CABLE_VALUE_TYPE == "displacement":
            match_col  = "displacement_mm"
            my_targets = list(self.levels)
            match_tol  = 5.0
        else:
            match_col  = "tension_newton"
            my_targets = [t / SCENE_FORCE_PER_NEWTON for t in self.levels]
            match_tol  = 1e-4
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames or self._fieldnames
            kept = []
            for row in reader:
                mine = (row.get("design") == self.design_name and
                        row.get("cable") == my_cable)
                try:
                    val = float(row.get(match_col, "nan"))
                    # our own rows: the zero-load anchor + this run's pull levels
                    same = mine and (abs(val) <= 1e-9 or
                                     any(abs(val - t) <= match_tol for t in my_targets))
                except (ValueError, TypeError):
                    same = False
                if not same:
                    kept.append(row)
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(kept)

    def _tip(self):
        """Deformed tip = last spine centroid — used only for settle detection."""
        return tuple(float(v) for v in self.spine.tip_point())

    def _reset_settle(self):
        self.phase_step = 0
        self.prev_tip   = None
        self.streak     = 0

    def _check_settled(self, tip):
        """Advance streak; return (settled, delta_mm)."""
        delta = 0.0
        if self.prev_tip is not None:
            delta = sum((a - b) ** 2 for a, b in zip(tip, self.prev_tip)) ** 0.5
            self.streak = self.streak + 1 if delta < SETTLE_TIP_THRESHOLD_MM else 0
        self.prev_tip    = tip
        self.phase_step += 1
        settled = (self.phase_step >= SETTLE_MIN_STEPS and
                   self.streak     >= SETTLE_CONSECUTIVE_STEPS)
        return settled, delta

    def _log(self, constraint, settled=True):
        """Log a pull: read the resulting force / displacement off the cable
        constraint and write one row with the current deformed spine.
        `settled` is False when the step budget ran out before the tip
        genuinely quieted down (see onAnimateBeginEvent) — the row is still
        written ("logging anyway"), just flagged as lower-confidence."""
        force = float(constraint.force.value)
        disp  = float(constraint.displacement.value)
        self._write_row(force / SCENE_FORCE_PER_NEWTON, disp, settled=settled)

    def _log_zero(self):
        """Zero-load anchor row (tension 0), written once after rest capture: the
        rest spine is logged as the deformed spine so every tip trajectory starts
        at (0, 0) and bend_angle_deg is 0. No extra solver steps, so trivially
        "settled" (there's no transient to wait out)."""
        self._write_row(0.0, 0.0,
                        deformed=self.spine.rest_spine,
                        true_tip=self.spine.true_tip, settled=True)

    def _write_row(self, newton, disp, deformed=None, true_tip=None, settled=True):
        """One CSV row: metadata + rest + deformed spine + bend/curvature metrics.
        deformed/true_tip default to the live spine; _log_zero passes the rest one."""
        if self._csv_writer is None:
            return

        rest     = self.spine.rest_spine
        deformed = self.spine.current_spine() if deformed is None else deformed
        true_base = self.spine.true_base
        true_tip  = self.spine.current_true_tip() if true_tip is None else true_tip

        nx, ny, nz = bend_plane_normal(rest, deformed)
        tip = deformed[-1]

        # constant-curvature check; "" for fields undefined on a straight spine
        cc = curvature_constancy(deformed)
        _fin = lambda v, n: round(float(v), n) if np.isfinite(v) else ""

        row = {
            "design":          self.design_name,
            "backbone_length": self.backbone_length,
            "cable":           self.cable_idx + 1,
            "tension_newton":  round(newton, 6),
            "displacement_mm": round(disp, 4),
            "settled":         int(bool(settled)),
            "n_spine":         self.spine.n_spine,
            "tip_x":           round(float(tip[0]), 4),
            "tip_y":           round(float(tip[1]), 4),
            "tip_z":           round(float(tip[2]), 4),
            "bend_angle_deg":  round(bend_angle_deg(rest, deformed, true_base, true_tip), 4),
            "arc_length_mm":   _fin(cc["arc_length_mm"], 4),
            "radius_mm":       _fin(cc["radius_mm"], 4),
            "kappa_mean_permm": _fin(cc["kappa_mean_permm"], 8),
            "kappa_cv":        _fin(cc["kappa_cv"], 6),
            "cc_rmse_mm":      _fin(cc["cc_rmse_mm"], 5),
            "cc_rmse_norm":    _fin(cc["cc_rmse_norm"], 6),
            "plane_nx":        round(nx, 6),
            "plane_ny":        round(ny, 6),
            "plane_nz":        round(nz, 6),
            "true_base_x":     round(float(true_base[0]), 4),
            "true_base_y":     round(float(true_base[1]), 4),
            "true_base_z":     round(float(true_base[2]), 4),
            "true_tip_x":      round(float(true_tip[0]), 4),
            "true_tip_y":      round(float(true_tip[1]), 4),
            "true_tip_z":      round(float(true_tip[2]), 4),
        }
        for i, (x, y, z) in enumerate(rest):
            row[f"rest_x{i}"] = round(float(x), 4)
            row[f"rest_y{i}"] = round(float(y), 4)
            row[f"rest_z{i}"] = round(float(z), 4)
        for i, (x, y, z) in enumerate(deformed):
            row[f"def_x{i}"] = round(float(x), 4)
            row[f"def_y{i}"] = round(float(y), 4)
            row[f"def_z{i}"] = round(float(z), 4)

        self._csv_writer.writerow(row)
        self._csv_file.flush()
        tx, ty, tz = deformed[-1]
        cv  = cc["kappa_cv"]
        rn  = cc["cc_rmse_norm"]
        cc_txt = (f"CV_k={cv:.3f}  RMSE/L={rn:.4f}  R={cc['radius_mm']:.1f}mm"
                  if np.isfinite(cv) else "straight")
        print(f"[CTRL] LOGGED  cable={self.cable_idx+1}  {newton:.4g} N  "
              f"disp={disp:.3f} mm  spine_pts={self.spine.n_spine}  "
              f"tip=({tx:.2f}, {ty:.2f}, {tz:.2f})  [{cc_txt}]")

    def _close_csv(self):
        if self._csv_file:
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None

    # ── simulation loop ───────────────────────────────────────────────────────

    def onAnimateBeginEvent(self, dt):
        if self.done:
            return

        self.frame_count += 1
        if self.frame_count < REST_CAPTURE_FRAME:
            return

        if not self.rest_captured:
            self.spine.capture_rest(backbone_axis=self.backbone_axis,
                                     station_positions=self.notch_positions)
            self._open_csv()   # header depends on the captured spine-point count
            self.rest_captured = True
            self._log_zero()   # (0,0) tension-0 anchor row, before any pull
            if SNAPSHOT_PAUSE_S > 0:
                print(f"[CTRL] SNAPSHOT-READY  0 N  (frozen {SNAPSHOT_PAUSE_S:.1f}s)")
                time.sleep(SNAPSHOT_PAUSE_S)

        if self.seq_idx >= len(self.sequence):
            self._close_csv()
            print(f"[CTRL] All measurements done — {self.design_name}")
            self.done = True
            os._exit(0)

        target = self.sequence[self.seq_idx]
        constraint = self.cables[self.cable_idx].CableConstraint

        # First step of a new phase: set the pull target and reset tracking.
        if self.phase_step == 0:
            if CABLE_VALUE_TYPE == "displacement":
                print(f"[CTRL] PULL   cable={self.cable_idx+1}  {target:.4g} mm")
            else:
                print(f"[CTRL] PULL   cable={self.cable_idx+1}  "
                      f"{target / SCENE_FORCE_PER_NEWTON:.4g} N")
            constraint.value = [target]
            self._reset_settle()

        tip = self._tip()
        settled, delta = self._check_settled(tip)
        self.last_delta = delta
        timed_out = self.phase_step >= SETTLE_MAX_STEPS_PULL

        if settled or timed_out:
            if timed_out and not settled:
                print(f"[CTRL] TIMEOUT cable={self.cable_idx+1}  "
                      f"phase_step={self.phase_step}  PULL — logging anyway")
            self._log(constraint, settled=settled)
            if SNAPSHOT_PAUSE_S > 0:
                force_n = target / SCENE_FORCE_PER_NEWTON if CABLE_VALUE_TYPE != "displacement" else float("nan")
                print(f"[CTRL] SNAPSHOT-READY  {force_n:.4g} N  (frozen {SNAPSHOT_PAUSE_S:.1f}s)")
                time.sleep(SNAPSHOT_PAUSE_S)
            self.seq_idx += 1
            self._reset_settle()
