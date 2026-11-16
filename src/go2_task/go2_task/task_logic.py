"""Task-layer logic without ROS (spec 6, 7.4): mission files and the per-waypoint rules (progress, hold, recovery ladders,
budget). Pure functions / small classes, tested offline.

Whether the dog CAN get to a waypoint is not decided here any more: mot_nav's own planner answers (service /mot_nav_check,
checker.hpp) - one judge for "can it get there" (P2). This file only knows what a task is and how a waypoint attempt is counted.

A waypoint is an area (arrived = body centre within radius_m). It may carry `path`: the taught polyline that leads to it
(map frame, [[x, y], ...]); mot_nav then follows that path instead of planning its own (teach routes).
"""
import math
from dataclasses import dataclass, asdict, field
from enum import Enum
from pathlib import Path
from typing import Optional

import yaml


class WpState(str, Enum):
    PENDING = 'PENDING'
    APPROACH = 'APPROACH'
    RECOVER = 'RECOVER'
    ARRIVED = 'ARRIVED'
    SKIPPED = 'SKIPPED'
    BAD = 'BAD'                     # failed the check


@dataclass
class Waypoint:
    """A waypoint is an area (spec 6.1): arrived = body centre within radius_m; yaw (rad, map) then only a turn."""
    name: str
    x: float
    y: float
    radius_m: float
    yaw: Optional[float] = None
    path: Optional[list] = None     # taught path to this waypoint [[x, y], ...]


_KEYS = {'name', 'x', 'y', 'yaw', 'radius_m', 'path'}


def load_mission(path, default_radius_m: float) -> list:
    data = yaml.safe_load(Path(path).read_text()) or {}
    raw = data.get('waypoints')
    if not isinstance(raw, list) or not raw:
        raise ValueError(f'{path}: needs a non-empty list `waypoints`')
    out = []
    for i, w in enumerate(raw):
        unknown = set(w) - _KEYS
        if unknown:
            raise ValueError(f'{path}: waypoint {i}: unknown keys {sorted(unknown)}')
        if 'x' not in w or 'y' not in w:
            raise ValueError(f'{path}: waypoint {i}: x and y are required')
        pts = w.get('path')
        if pts is not None:
            if not (isinstance(pts, list) and len(pts) >= 2 and all(isinstance(p, (list, tuple)) and len(p) == 2 for p in pts)):
                raise ValueError(f'{path}: waypoint {i}: path must be a list of at least 2 [x, y] points')
            pts = [[float(p[0]), float(p[1])] for p in pts]
        out.append(Waypoint(name=str(w.get('name', f'wp{i + 1}')), x=float(w['x']), y=float(w['y']),
                            radius_m=float(w.get('radius_m', default_radius_m)),
                            yaw=None if w.get('yaw') is None else float(w['yaw']), path=pts))
    return out


def save_mission(path, waypoints: list) -> None:
    rows = [{k: v for k, v in asdict(w).items() if v is not None} for w in waypoints]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(yaml.safe_dump({'waypoints': rows}, sort_keys=False, allow_unicode=True))


# ---------------- per-waypoint rules ----------------
class Progress:
    """No progress = the remaining distance did not improve by eps_m within window_s (turning in place is fine: the
    remaining distance along the path does not grow)."""

    def __init__(self, window_s, eps_m=0.1):
        self.window_s, self.eps_m = window_s, eps_m
        self.reset(None)

    def reset(self, now):
        self.best, self.since = math.inf, now

    def update(self, now, remaining_m):
        if self.since is None:
            self.since = now
        if remaining_m < self.best - self.eps_m:
            self.best, self.since = remaining_m, now
        return now - self.since < self.window_s       # True = still progressing


class Hold:
    """A decision needs its trigger continuously for hold_s, and hold_s must have passed since the last decision (P5)."""

    def __init__(self, hold_s):
        self.hold_s, self.since, self.last_decision = hold_s, None, -math.inf

    def update(self, now, trigger):
        if not trigger:
            self.since = None
            return False
        self.since = now if self.since is None else self.since
        if now - self.since >= self.hold_s and now - self.last_decision >= self.hold_s:
            self.last_decision, self.since = now, None
            return True
        return False


# Recovery ladders by WHY mot_nav / the progress check gave up (2026-10-26; before, one fixed ladder for every reason).
#   blocked       every sampled trajectory hits something close (evidence-map memory exists): backup, turn   [LOCAL_LADDERS]
#   no_path       the planner cannot get there (evidence may change): wait, another spot, back off
#   goal_blocked  the target area is closed off: another spot at once, wait for the evidence map in between
#   start_blocked the global planner cannot start here (that map has no voxel memory, a blind backup has no basis):
#                 turn, then relocate to a checked temporary spot nearby (never backup)                         [LOCAL_LADDERS]
# blocked / start_blocked are executed by go2_task.local_recovery (shared by every policy: actual-displacement judged); the
# other reasons below still use next_recovery() in the legacy single/teach policy.
#   no_progress   moving but not getting closer (a loop, a slippery floor): wait, back off, another spot, turn
#   no_nav        mot_nav is not answering: wait (nothing else helps)
LOCAL_LADDERS = {'blocked': ('backup', 'turn'), 'start_blocked': ('turn', 'relocate')}
LADDERS = {
    **LOCAL_LADDERS,
    'no_path': ('wait', 'alt_position', 'backup', 'turn'),
    'goal_blocked': ('alt_position', 'wait', 'alt_position'),
    'no_progress': ('wait', 'backup', 'alt_position', 'turn'),
    'no_nav': ('wait',),
}
# Single-point and taught-path tasks (2026-11-12, executed by go2_task.local_recovery like blocked/start_blocked):
#   no_path / no_progress  backup, then turn (judged by the actual displacement); exhausted -> SKIPPED (+ the one ROUND_RETRY)
#   goal_blocked           whenever it happens (at the start or on the way, no special case and no popup): up to 5 CHECKED
#                          alternative landing spots around the original target, each asked of /mot_nav_check from the dog's
#                          current position, like the final waypoint of a route (never for a taught path). No detour limit.
# The LADDERS entries above for no_path / goal_blocked / no_progress are no longer used by the executor for them.
SINGLE_LADDERS = {**LOCAL_LADDERS, 'no_path': ('backup', 'turn'), 'no_progress': ('backup', 'turn'),
                  'goal_blocked': ('alt_goal',)*5}
DEFAULT_LADDER = ('wait', 'backup', 'alt_position', 'turn')
# Reasons that mot_nav already held for a while (blocked_hold_s, plan_fail_hold_s): act after a short hold. no_progress is the
# slow one (the progress window plus a longer hold).
EXPLICIT_REASONS = frozenset(LADDERS) - {'no_progress'}


@dataclass
class WaypointRun:
    """Book-keeping of one waypoint attempt: budget (time while not paused) and tries (recovery steps)."""
    index: int
    budget_s: float
    max_tries: int
    state: WpState = WpState.PENDING
    used_s: float = 0.0
    tries: int = 0
    steps_by_reason: dict = field(default_factory=dict)
    tried_positions: list = field(default_factory=list)
    reason: str = ''
    # Single-point / taught-path tasks (2026-11-12): the budget is PROBLEM time, not the whole walk. trouble_s is a real clock
    # (non-paused sim time spent in trouble) that nothing resets: not a resumed goal, a finished recovery step, a replan, a new goal
    # id, ROUND_RETRY or the stage-one stop. At budget_s the dog stops once and continues; at 2*budget_s the point is abandoned.
    trouble_budget: bool = False
    trouble_s: float = 0.0
    stage_one_done: bool = False

    def tick(self, dt, paused, trouble=False):
        if not paused and self.state in (WpState.APPROACH, WpState.RECOVER):
            self.used_s += dt
            if trouble:
                self.trouble_s += dt

    def trouble_stage(self):
        """0 = within the first budget, 1 = first budget spent (stop once, continue), 2 = both spent (abandon)."""
        if not self.trouble_budget:
            return 0
        return 2 if self.trouble_s >= 2*self.budget_s else 1 if self.trouble_s >= self.budget_s and not self.stage_one_done else 0

    def out_of_budget(self):
        if self.trouble_budget:
            return self.tries >= self.max_tries or self.trouble_s >= 2*self.budget_s
        return self.used_s >= self.budget_s or self.tries >= self.max_tries

    def next_recovery(self, reason=''):
        ladder = LADDERS.get(reason, DEFAULT_LADDER)
        k = self.steps_by_reason.get(reason, 0)
        self.steps_by_reason[reason] = k + 1
        self.tries += 1
        return ladder[k % len(ladder)]
