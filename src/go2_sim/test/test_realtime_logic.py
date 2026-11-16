import math
import numpy as np
from go2_sim.realtime_logic import ObservedMap, navigation_grid, reachable_frontiers, known_goal


def test_observed_map_starts_unknown_and_missing_returns_never_reveal_world():
    m=ObservedMap(100,100,.1,(-5.,-5.))
    assert (m.grid()==-1).all()
    m.update((0.,0.,0.),np.array([0.]),np.array([np.inf]),np.array([0.]),(.35,.2))
    assert m.grid()[50,70]==-1
    assert m.grid()[50,50]==0
    assert m.grid()[54,54]==-1  # not a fictitious circumscribed free disc


def test_mapping_hit_ray_and_clearing_use_correct_map_frame():
    m=ObservedMap(100,100,.1,(-5.,-5.))
    m.update((0.,0.,math.pi/2),np.array([0.]),np.array([2.]),np.array([0.]),(.35,.2))
    assert m.grid()[70,50]==100
    assert m.grid()[60,50]==0
    assert m.grid()[50,70]==-1
    for _ in range(4):
        m.update((0.,0.,math.pi/2),np.array([0.]),np.array([np.inf]),np.array([3.]),(.35,.2))
    assert m.grid()[70,50]==0


def test_realtime_navigation_blocks_unknown_and_recent_obstacles():
    observed=np.array([[-1,0,100],[0,0,-1]],np.int8)
    evidence=np.array([[-1,100,0],[-1,0,-1]],np.int8)
    result=navigation_grid(observed,evidence)
    assert result.tolist()==[[100,100,100],[0,0,100]]
    assert not (result[observed<0]==0).any()


def test_frontier_goals_are_in_reachable_known_space_and_ignore_other_rooms():
    m=np.full((100,140),-1,np.int8)
    m[10:90,10:70]=0
    m[10:90,90:130]=0  # disjoint observed room beyond unknown
    nav=navigation_grid(m)
    goals=reachable_frontiers(m,nav,.1,(0.,0.),(3.,5.,0.),.5)
    assert goals
    assert all(1.<g['x']<7. for g in goals)
    assert all(known_goal(nav,.1,(0.,0.),(g['x'],g['y']),.5) for g in goals)
    xy=[(g['x'],g['y']) for g in goals]
    again=reachable_frontiers(m,nav,.1,(0.,0.),(3.,5.,0.),.5,xy)
    assert all(math.hypot(g['x']-p[0],g['y']-p[1])>=.65 for g in again for p in xy)


def test_frontier_empty_in_closed_fully_observed_room_and_unsafe_start():
    m=np.full((40,40),100,np.int8);m[5:35,5:35]=0
    assert not reachable_frontiers(m,navigation_grid(m),.1,(0.,0.),(2.,2.,0.),.5)
    m[:]=-1;m[19:22,19:22]=0
    assert not reachable_frontiers(m,navigation_grid(m),.1,(0.,0.),(2.,2.,0.),.5)
    assert not known_goal(navigation_grid(m),.1,(0.,0.),(np.nan,1.),.5)


def test_only_active_clients_heartbeat_keeps_autonomy_alive():
    from go2_sim.realtime_logic import ClientLease
    lease=ClientLease()
    lease.heartbeat('operator',0.)
    lease.owner='operator'
    assert lease.valid(2.9)
    lease.heartbeat('observer',3.2)
    assert not lease.valid(3.2)
    assert lease.fresh('observer',3.2)
    lease.owner='observer'
    assert lease.valid(3.2)
    lease.owner=None
    assert not lease.valid(3.2)


def test_localisation_reset_discards_accumulation_and_advances_display_version():
    m=ObservedMap(100,100,.1,(-5.,-5.))
    m.update((0.,0.,0.),np.array([0.]),np.array([2.]),np.array([0.]),(.35,.2))
    rev=m.rev
    assert (m.grid()>=0).any()
    m.reset()
    assert m.rev>rev and (m.grid()==-1).all()
