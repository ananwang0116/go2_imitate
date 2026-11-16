import math
import os
import time

import pytest
import yaml

from go2_web import web_logic as wl
from go2_web.stack_manager import StackManager

FAKE = ['bash', '-c', 'echo "[run] START $*"; trap "echo got-int; exit 0" INT; while true; do sleep 0.1; done', '--']


def mk_ws(tmp_path):
    (tmp_path / 'maps' / 'm').mkdir(parents=True)
    (tmp_path / 'maps' / 'm' / 'map.yaml').write_text('image: map.pgm\n')
    (tmp_path / 'runs').mkdir()
    return tmp_path


def wait(cond, s=5):
    end = time.time() + s
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_stack_start_stop_group(tmp_path):
    m = StackManager(mk_ws(tmp_path), launch_cmd=FAKE, kill_after_s=3)
    assert m.status()['state'] == 'idle'
    pid = m.start('nav', 'm', False)
    assert pid != os.getpgrp() and os.getpgid(pid) == pid           # its own session / group
    assert wait(lambda: 'mode:=nav' in ' '.join(m.tail()))
    text = ' '.join(m.tail())
    assert 'map:=m' in text and 'enable_motion:=false' in text and 'web:=false' in text
    assert m.status()['state'] == 'starting'
    with pytest.raises(PermissionError):
        m.start('map', '', False)
    assert m.stop()
    assert wait(lambda: not m.alive())
    st = m.status()
    assert st['state'] == 'idle' and not st['killed']
    assert not m.stop()


def test_stack_rejects_bad_input_and_reports_failure(tmp_path):
    m = StackManager(mk_ws(tmp_path), launch_cmd=['bash', '-c', 'echo CHECK FAIL network; exit 3', '--'])
    for args in (('xyz', '', False), ('nav', 'nope', False)):
        with pytest.raises(ValueError):
            m.start(*args)
    m.start('map', '', True)
    assert wait(lambda: not m.alive())
    st = m.status()
    assert st['state'] == 'failed' and st['returncode'] == 3 and any('CHECK FAIL' in ln for ln in st['tail'])


def test_stack_sigkill_escalation(tmp_path):
    stubborn = ['bash', '-c', 'trap "" INT; while true; do sleep 0.1; done', '--']
    m = StackManager(mk_ws(tmp_path), launch_cmd=stubborn, kill_after_s=0.5)
    m.start('map', '', False)
    time.sleep(0.3)
    m.stop()
    assert wait(lambda: not m.alive())
    assert m.status()['killed']


def test_route_rows_and_segments():
    path = [[i * 0.1, 0.0] for i in range(101)] + [[10.0, i * 0.1] for i in range(1, 51)]    # 10 m east, then 5 m north
    r = wl.route_rows('东侧走廊', 'm', path, [[5.0, 0.0]])
    assert r['length_m'] == pytest.approx(15.0, abs=0.2) and r['marks'] == [[5.0, 0.0]]
    segs = wl.route_segments(r['path'], r['marks'])
    assert [s['name'] for s in segs] == ['P1', 'end']
    assert segs[0]['path'][0] == [0.0, 0.0] and segs[0]['x'] == pytest.approx(5.0)
    assert segs[1]['path'][0] == segs[0]['path'][-1]                      # the segments join at the stop
    assert segs[1]['y'] == pytest.approx(5.0)
    # the dog is already at the corner: the mark behind it is dropped, the path starts there
    later = wl.route_segments(r['path'], r['marks'], from_xy=(10.0, 0.5))
    assert [s['name'] for s in later] == ['end'] and later[0]['path'][0][0] == pytest.approx(10.0)
    # a mark at the very start or the very end does not make an empty segment
    assert [s['name'] for s in wl.route_segments(r['path'], [[0.0, 0.0], [10.0, 5.0]])] == ['end']
    with pytest.raises(ValueError):
        wl.route_rows('a', 'm', [[0, 0]], [])
    with pytest.raises(ValueError):
        wl.route_rows('../a', 'm', path, [])
    rows = wl.mission_rows(segs)
    assert rows[0]['path'][0] == [0.0, 0.0] and 'yaw' not in rows[0]


def test_route_files(tmp_path):
    d = tmp_path / 'm' / 'routes'
    d.mkdir(parents=True)
    r = wl.route_rows('r1', 'm', [[0, 0], [1, 0], [2, 0]], [])
    (d / 'r1.yaml').write_text(yaml.safe_dump(r))
    assert wl.list_routes(tmp_path, 'm') == [{'name': 'r1', 'length_m': 2.0, 'marks': 0}]
    assert wl.read_route(tmp_path, 'm', 'r1')['path'][-1] == [2.0, 0.0]
    assert wl.list_routes(tmp_path, 'none') == []
