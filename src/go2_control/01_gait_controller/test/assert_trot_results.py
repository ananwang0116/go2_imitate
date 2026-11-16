"""Independent physical acceptance from Gazebo model_states, not target joints."""
import json
import math
import statistics
import sys

with open(sys.argv[1]) as source:
    data = json.load(source)
rows = data['rows']
assert rows and rows[-1][0] >= 109, 'Incomplete physical run'
assert not any(row[-1] for row in data['status']), 'Controller fault recorded'
windows = [('forward_004',15,22,.04,0), ('forward_014',28,35,.14,0),
           ('turn_03',41,48,0,.3), ('turn_10',55,62,0,1),
           ('forward_after_turn',69,76,.14,0), ('stop',78,80,0,0),
           ('turn_negative',85,92,0,-1), ('forward_last',99,104,.14,0),
           ('timeout_stop',108,110,0,0)]
summary = {}
for label, start, end, target_v, target_yaw in windows:
    segment = [row for row in rows if start < row[0] < end]
    assert len(segment) > 50, f'{label}: missing actual Gazebo measurements'
    vx = statistics.mean(math.cos(row[6])*row[7]+math.sin(row[6])*row[8] for row in segment)
    vy = statistics.mean(-math.sin(row[6])*row[7]+math.cos(row[6])*row[8] for row in segment)
    yaw_delta = sum(math.atan2(math.sin(b[6]-a[6]),math.cos(b[6]-a[6])) for a,b in zip(segment,segment[1:]))
    yaw_rate = yaw_delta/(segment[-1][0]-segment[0][0])
    roll = max(abs(row[4]) for row in segment)*180/math.pi
    pitch = max(abs(row[5]) for row in segment)*180/math.pi
    height_pp = (max(row[3] for row in segment)-min(row[3] for row in segment))*1000
    summary[label] = dict(actual_body_v=vx,side_v=vy,yaw_rate=yaw_rate,
                          roll_peak_deg=roll,pitch_peak_deg=pitch,height_pp_mm=height_pp)
    assert max(roll,pitch) < 5, f'{label}: excessive tilt {roll}/{pitch}'
    assert height_pp < 10, f'{label}: excessive vertical oscillation {height_pp}'
    if target_v:
        assert abs(vx-target_v) < .2*abs(target_v), f'{label}: speed mismatch {vx}'
        assert abs(vy) < .01 and abs(yaw_rate) < .02, f'{label}: lateral/heading drift'
    elif target_yaw:
        assert abs(yaw_rate-target_yaw) < .15*abs(target_yaw), f'{label}: turn mismatch {yaw_rate}'
        assert abs(vx) < .02 and abs(vy) < .02, f'{label}: translation during turning'
    else:
        assert math.hypot(vx,vy) < .005 and abs(yaw_rate) < .01, f'{label}: did not stop'
print(json.dumps(summary,indent=2))
print('Physical acceptance passed: speed, heading, both turns, tilt, height, and stopping.')
