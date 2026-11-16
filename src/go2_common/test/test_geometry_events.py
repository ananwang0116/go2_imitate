"""Offline tests for the geometry nesting check and the event format. Run: colcon test --packages-select go2_common"""
from dataclasses import replace
from pathlib import Path

import pytest

from go2_common import geometry
from go2_common.events import format_event, format_value
from go2_common.names import EVENT_TOPICS, LAYERS, event_topic

GEOMETRY = Path(__file__).resolve().parents[2] / 'go2_bringup' / 'config' / 'geometry.yaml'


def test_shipped_geometry_nests():
    assert geometry.check(geometry.load(GEOMETRY)) == []


@pytest.mark.parametrize('change, word', [
    ({'avoid_gap_m': 0.15}, 'avoid_gap_m'),                 # < safe_stop 0.10 + sigma 0.10
    ({'waypoint_clearance_m': 0.25}, 'waypoint_clearance_m'),
    ({'arrive_radius_m': 0.05}, 'arrive_radius_m'),
    ({'brake_decel_mps2': 0.0}, 'brake_decel_mps2'),
])
def test_violations_are_named(change, word):
    g = replace(geometry.load(GEOMETRY), **change)
    problems = geometry.check(g)
    assert len(problems) == 1 and word in problems[0]


def test_unknown_or_missing_keys_rejected(tmp_path):
    bad = tmp_path / 'g.yaml'
    bad.write_text(GEOMETRY.read_text() + 'robot_radius: 0.4\n')
    with pytest.raises(ValueError, match='robot_radius'):
        geometry.load(bad)


def test_brake_distance():
    g = geometry.load(GEOMETRY)
    assert g.brake_distance_m(0.0) == 0.0
    assert g.brake_distance_m(-0.4) == pytest.approx(0.4 * g.brake_reaction_s + 0.16 / (2 * g.brake_decel_mps2))


def test_event_format():
    text = format_event('safe', 'LIMIT', {'dir': 'front', 'gap_m': 0.21, 'v_in': 0.45, 'v_out': 0.12})
    assert text == '[safe] LIMIT dir=front gap_m=0.21 v_in=0.45 v_out=0.12'
    assert format_event('task', 'SKIP', {'wp': 3, 'used_s': 30.0}) == '[task] SKIP wp=3 used_s=30.0'
    assert format_value(True) == 'true' and format_value('a b') == '"a b"'


@pytest.mark.parametrize('layer, name, fields', [
    ('nav', 'START', {}), ('safe', 'limit', {}), ('safe', 'LIMIT', {'Gap': 1})])
def test_event_format_rejects(layer, name, fields):
    with pytest.raises(ValueError):
        format_event(layer, name, fields)


def test_event_topics():
    assert event_topic('loc') == '/loc_event'
    assert len(EVENT_TOPICS) == len(LAYERS)
