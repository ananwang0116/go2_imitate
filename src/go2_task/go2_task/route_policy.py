"""Multi-waypoint policy helpers. No map interpretation, planner or motion publisher."""
import json
import math
import os
from datetime import datetime, timezone

from go2_common.events import run_dir


class SpatialProgress:
    """Visit-local spatial progress from localisation, independent of changing paths.

    Progress = the robot has been at least eps_m away from where it is NOW at some time in the
    last window_s (non-paused waypoint time). Creeping, turning in place and small oscillations
    stay inside the eps_m disc and earn nothing; any real displacement, including driving back
    out of a dead-end along the same corridor, counts. Large loops are not detected here: the
    per-waypoint budget bounds them. Normal turns have the full window to finish.
    """
    def __init__(self, window_s=20., eps_m=.4):
        self.window_s, self.eps_m = window_s, eps_m
        self.reset()

    def reset(self, _now=None):
        self.samples = []
        self.last_progress_s = 0.

    def update(self, used_s, xy):
        if not self.samples:
            self.last_progress_s = used_s
        self.samples.append((used_s, xy[0], xy[1]))
        keep = used_s-self.window_s-1.
        while len(self.samples) > 1 and self.samples[0][0] < keep:
            self.samples.pop(0)
        for t, x, y in reversed(self.samples):
            if math.hypot(xy[0]-x, xy[1]-y) >= self.eps_m:
                self.last_progress_s = max(self.last_progress_s, t)
                break
        return used_s - self.last_progress_s < self.window_s


class DetourGuard:
    """Refuse a big detour: the planner's cost-to-go may not rise far above the lowest value seen for this waypoint.

    A long route that exists from the start (doors, rooms) sets the baseline and is allowed; a route that BECOMES much
    longer (new obstacles) exceeds max(best*ratio, best+margin_m) for hold_s and is refused. Cost is the original
    planner's A* length (multi-waypoint: /mot_nav_check leg_m; single point: mot_nav remaining_m). Invalid costs
    (None, nan, inf, <=0) are ignored. No planner or map logic here; "no path / goal blocked" are separate failures.
    """
    def __init__(self, ratio=1.5, margin_m=2., hold_s=3.):
        self.ratio, self.margin_m, self.hold_s = ratio, margin_m, hold_s
        self.reset()

    def reset(self):
        self.best = math.inf
        self.since = None
        self.last = None

    def limit(self):
        return math.inf if not math.isfinite(self.best) else max(self.best*self.ratio, self.best+self.margin_m)

    def allows(self, cost):
        """Would this cost be acceptable now (for a candidate target)? No state change."""
        return not math.isfinite(self.best) or cost <= self.limit()

    def update(self, t, cost):
        """True = the cost has stayed above the limit for hold_s (detour too long)."""
        if cost is None or not math.isfinite(cost) or cost <= 0.:
            return False
        self.last = cost
        self.best = min(self.best, cost)
        if cost > self.limit():
            self.since = t if self.since is None else self.since
            return t-self.since >= self.hold_s
        self.since = None
        return False


def final_candidates(w, robot, max_offset=1.):
    """Geometric proposals only; EACH must be checked from the dog by mot_nav.

    Candidate centres stay within max_offset of the immutable original final
    waypoint. Rank short approaches with a small original-target distance penalty;
    this ranking makes no reachability claim (mot_nav is still the judge).
    """
    bearing = math.atan2(robot[1]-w.y, robot[0]-w.x)
    out = []
    for fraction in (.5, 1.):
        for k in (0, 1, -1, 2, -2, 3, -3, 4):
            angle = bearing + k * math.pi / 4
            out.append((w.x + max_offset*fraction*math.cos(angle),
                        w.y + max_offset*fraction*math.sin(angle)))
    return sorted(out, key=lambda xy: math.hypot(xy[0]-robot[0], xy[1]-robot[1])
                  + .25*math.hypot(xy[0]-w.x, xy[1]-w.y))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def backup_effect_m(start, now):
    """Actual distance moved straight backwards (along -heading at start) between two localisation poses (x, y, yaw)."""
    return -((now[0]-start[0])*math.cos(start[2]) + (now[1]-start[1])*math.sin(start[2]))


def turn_effect_rad(start, now):
    return wrap_angle(now[2]-start[2])


def relocation_candidates(robot, goal, distances=(.5, .8), count=8):
    """Temporary spots around the DOG (not around the goal). Geometry only: each one is still judged by /mot_nav_check
    from the dog's pose; the ranking prefers close spots with a small penalty for distance to the goal."""
    out = []
    for d in distances:
        for k in range(count):
            a = robot[2] + 2*math.pi*k/count
            out.append((robot[0]+d*math.cos(a), robot[1]+d*math.sin(a)))
    return sorted(out, key=lambda xy: math.hypot(xy[0]-robot[0], xy[1]-robot[1])
                  + .25*math.hypot(xy[0]-goal[0], xy[1]-goal[1]))


def append_route_audit(row):
    """Append one structured record to the owning run; no cross-session file."""
    directory = run_dir()
    if directory is None:
        return
    row = dict(wall_utc=datetime.now(timezone.utc).isoformat(), **row)
    line = (json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n').encode()
    fd = os.open(directory / 'route_decisions.jsonl', os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        os.write(fd, line)
    finally:
        os.close(fd)
