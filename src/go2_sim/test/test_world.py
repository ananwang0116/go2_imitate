from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from go2_sim.world import spawn_pose, build_world


def test_spawn_rejects_insufficient_clearance():
    with pytest.raises(ValueError, match='clearance'):
        spawn_pose(np.zeros((3, 3), dtype=np.int8), .05, (0, 0), .5)


def test_supplied_map_world_is_self_contained(tmp_path):
    ws = Path(__file__).resolve().parents[3]
    path = tmp_path / 'world.sdf'
    info = build_world(ws/'maps/my_map/map.yaml', ws/'src/go2_bringup/config/geometry.yaml', path)
    world = ET.parse(path).getroot().find('world')
    assert 'custom.world' in info['world_source']
    assert world.find("model[@name='room']") is not None
    assert world.find("model[@name='go2']") is None  # spawned only after the Gazebo factory is ready
    robot = ET.parse(info['robot_description']).getroot()
    assert len(robot.findall("joint[@type='revolute']")) == 12
    assert robot.find("joint[@name='radar_joint']") is None
    mount = robot.find("joint[@name='mid360_joint']/origin")
    np.testing.assert_allclose([float(v) for v in mount.get('xyz').split()], [.2954, 0., .097])
    np.testing.assert_allclose([float(v) for v in mount.get('rpy').split()], np.radians([-1.92,-59.59,180.]))
    assert robot.find("joint[@name='mid360_joint']/parent").get('link') == 'base'
    sensor = robot.find("gazebo[@reference='mid360_link']/sensor")
    assert sensor.get('name') == 'mid360_angular_approximation'
    assert sensor.find('.//vertical/samples').text == '40'
    assert not world.findall('.//include')  # no Fuel/model downloads
    assert not robot.findall(".//plugin[@filename='libgazebo_ros_planar_move.so']")
    assert robot.find(".//sensor[@type='imu']") is not None
