"""Observed-only 2D mapping and reachable frontier selection, without ROS.

No world/map file is read here. Unknown space is blocked in the navigation view.
The grid has fixed bounds so evidence and planner cell identities remain stable.
"""
import heapq
import math

import cv2
import numpy as np


class ObservedMap:
    def __init__(self, width=600, height=600, resolution=.05, origin=(-15., -15.)):
        self.w, self.h, self.res = width, height, resolution
        self.origin = np.asarray(origin, dtype=float)
        self.score = np.zeros((height, width), np.int16)
        self.seen = np.zeros((height, width), bool)
        self.rev = 0

    def reset(self):
        self.score[:]=0
        self.seen[:]=False
        self.rev+=1

    def cell(self, xy):
        return np.floor((np.asarray(xy) - self.origin) / self.res).astype(int)

    def inside(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def update(self, pose, angles, hits, free, body_half_size):
        x, y, yaw = pose
        start = self.cell((x, y))
        if not self.inside(*start):
            return False
        # A finite obstacle return proves the line before the hit clear at the
        # perception height band. Ground returns also prove their nearer cells clear.
        hits = np.asarray(hits)
        reach = np.maximum(np.where(np.isfinite(hits), np.minimum(hits, 6.)-self.res, 0.),
                           np.minimum(np.asarray(free), 6.)-self.res)
        reach = np.minimum(reach, np.where(np.isfinite(hits), hits-self.res, 6.))
        d = np.arange(0., 6., self.res*.7)[None, :]
        a = angles + yaw
        mask = d <= reach[:, None]
        fx = (x + np.cos(a)[:, None]*d)[mask]
        fy = (y + np.sin(a)[:, None]*d)[mask]
        def cells(xs, ys):
            ij = self.cell(np.column_stack((xs, ys)))
            keep = (ij[:, 0]>=0)&(ij[:, 0]<self.w)&(ij[:, 1]>=0)&(ij[:, 1]<self.h)
            return np.unique(ij[keep], axis=0)
        ij = cells(fx, fy)
        if len(ij):
            self.seen[ij[:, 1], ij[:, 0]] = True
            self.score[ij[:, 1], ij[:, 0]] = np.maximum(-8, self.score[ij[:, 1], ij[:, 0]]-2)
        hit = np.isfinite(hits)&(hits > .15)&(hits <= 6.)
        ij = cells(x+np.cos(a[hit])*hits[hit], y+np.sin(a[hit])*hits[hit])
        if len(ij):
            self.seen[ij[:, 1], ij[:, 0]] = True
            self.score[ij[:, 1], ij[:, 0]] = np.minimum(12, self.score[ij[:, 1], ij[:, 0]]+6)
        # Only the occupied body footprint is known at startup; no reveal of the world.
        yy, xx = np.ogrid[:self.h, :self.w]
        wx = self.origin[0]+(xx+.5)*self.res-x
        wy = self.origin[1]+(yy+.5)*self.res-y
        bx, by = np.cos(yaw)*wx+np.sin(yaw)*wy, -np.sin(yaw)*wx+np.cos(yaw)*wy
        body = (np.abs(bx)<=body_half_size[0])&(np.abs(by)<=body_half_size[1])
        self.seen[body] = True
        self.score[body] = -8
        self.rev += 1
        return True

    def grid(self):
        out = np.full((self.h, self.w), -1, np.int8)
        out[self.seen & (self.score <= 0)] = 0
        out[self.seen & (self.score > 0)] = 100
        return out


def navigation_grid(observed, evidence=None):
    """Known-free is the only traversable state; recent occupied evidence wins."""
    out = np.where(observed == 0, 0, 100).astype(np.int8)
    if evidence is not None and evidence.shape == out.shape:
        out[evidence >= 65] = 100
    return out


def reachable_frontiers(observed, nav, resolution, origin, pose, radius, blacklist=()):
    """Return ranked safe goals inside the robot's known reachable component.

    The candidate stands back from the free/unknown boundary by the footprint
    radius. Dijkstra uses 8 neighbours without diagonal corner cutting.
    """
    free = nav == 0
    clearance = cv2.distanceTransform(np.pad(free.astype(np.uint8), 1), cv2.DIST_L2, 5)[1:-1, 1:-1]*resolution
    safe = clearance > radius
    h, w = safe.shape
    si, sj = np.floor((np.asarray(pose[:2])-origin)/resolution).astype(int)
    if not (0<=si<w and 0<=sj<h and safe[sj, si]):
        return []
    unknown = (observed < 0).astype(np.uint8)
    near_unknown = cv2.dilate(unknown, np.ones((3, 3), np.uint8)).astype(bool)
    frontier = (observed == 0) & near_unknown
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier.astype(np.uint8), connectivity=8)
    dist = np.full((h, w), np.inf)
    dist[sj, si] = 0.
    q = [(0., sj, si)]
    while q:
        cost, j, i = heapq.heappop(q)
        if cost != dist[j, i]:
            continue
        for dj, di in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
            nj, ni = j+dj, i+di
            if not (0<=ni<w and 0<=nj<h and safe[nj, ni]):
                continue
            if di and dj and not (safe[j, ni] and safe[nj, i]):
                continue
            nc = cost + (math.sqrt(2) if di and dj else 1)*resolution
            if nc < dist[nj, ni]:
                dist[nj, ni] = nc
                heapq.heappush(q, (nc, nj, ni))
    goals = []
    reach = np.isfinite(dist)
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < 4:
            continue
        # Expand the frontier inward only to locate an approach point; never
        # turn these expanded cells into free navigation space.
        band = cv2.dilate((labels == label).astype(np.uint8),
                         np.ones((2*math.ceil((radius+.25)/resolution)+1,)*2, np.uint8)).astype(bool)
        js, ins = np.where(band & reach & (dist > .65))
        if not len(js):
            continue
        order = np.argsort(dist[js, ins])
        for k in order:
            xy = np.asarray(origin)+(np.array([ins[k], js[k]])+.5)*resolution
            if any(np.linalg.norm(xy-np.asarray(p)) < .65 for p in blacklist):
                continue
            gain = int(stats[label, cv2.CC_STAT_AREA])
            goals.append({'x': float(xy[0]), 'y': float(xy[1]), 'distance': float(dist[js[k],ins[k]]),
                          'gain': gain, 'score': float(gain*resolution/(1.+dist[js[k],ins[k]]))})
            break
    return sorted(goals, key=lambda x: -x['score'])


def known_goal(nav, resolution, origin, xy, radius):
    if not np.isfinite(xy).all():
        return False
    i, j = np.floor((np.asarray(xy)-origin)/resolution).astype(int)
    if not (0<=i<nav.shape[1] and 0<=j<nav.shape[0]):
        return False
    clearance = cv2.distanceTransform(np.pad((nav==0).astype(np.uint8), 1), cv2.DIST_L2, 5)[1:-1,1:-1]*resolution
    return bool(clearance[j,i] > radius)


class ClientLease:
    """A passive second tab cannot keep the active tab's mission alive."""
    def __init__(self, ttl=3.):
        self.ttl, self.owner, self.beats = ttl, None, {}

    def heartbeat(self, client, now):
        self.beats[client] = now
        self.beats = {k:v for k,v in self.beats.items() if now-v<=self.ttl}
        if len(self.beats)>32:
            oldest=min(self.beats,key=self.beats.get)
            self.beats.pop(oldest)

    def fresh(self, client, now):
        return client in self.beats and now-self.beats[client]<=self.ttl

    def valid(self, now):
        return self.fresh(self.owner, now)
