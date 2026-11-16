"""Realtime profile of the existing evidence owner, isolated from fixed-map mode.

PerEvidence still owns observation scores, expiry and /per_evidence_map. This
simulation profile converts the current observed map + recent evidence to the
sole /per_nav_map: unknown is blocked; measured occupied cells stay blocked
until the online mapper observes clear rays. Saved-map nav uses the original
PerEvidence unchanged, including its passable prior-wall policy.
"""
import array
from go2_common.node import spin, EVENT_QOS
from go2_msgs.msg import Event
from go2_per.per_evidence import PerEvidence
from go2_sim.realtime_logic import navigation_grid


class RealtimeEvidence(PerEvidence):
    def __init__(self):
        super().__init__()
        self.create_subscription(Event,'/loc_event',self.on_loc_event,EVENT_QOS)

    def on_loc_event(self,m):
        if m.name=='JUMP' and self.grid is not None:
            self.grid.reset();self.obstacle.clear();self.free.clear()
            self.event('EVIDENCE_RESET',reason='localisation_jump')

    def publish(self):
        if self.grid is not None:
            self.grid.expire()
        if self.grid is None or not self.grid.changed:
            return
        self.grid.changed=False
        evidence=self.grid.evidence()
        nav=navigation_grid(self.prior,evidence)
        # /map already marks the actually occupied body footprint observed-free.
        # Do not clear unseen neighbouring space to make a route artificially fit.
        for pub,data in ((self.ev_pub,evidence),(self.nav_pub,nav)):
            m=type(self.map_msg)()
            m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
            m.info=self.map_msg.info;m.data=array.array('b',data.tobytes())
            pub.publish(m)

    def on_map(self,msg):
        super().on_map(msg)
        if self.grid is not None:
            self.grid.changed=True


def main():
    spin(RealtimeEvidence)
