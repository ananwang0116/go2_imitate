"""Import internship world/Go2 visuals and attach a correctly mounted Mid360 angular approximation."""
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from go2_per.map_logic import load_map


def map_data(path):
    meta = yaml.safe_load(Path(path).read_text())
    if abs(float(meta['origin'][2])) > 1e-8:
        raise ValueError('rotated map origins are not supported by the imported map loader')
    return load_map(path)


def spawn_pose(grid, resolution, origin, radius):
    # Distance transform includes unknown as blocked. Prefer a clear point near world zero.
    free = np.pad((grid == 0).astype(np.uint8), 1)
    distance = cv2.distanceTransform(free, cv2.DIST_L2, 5)[1:-1, 1:-1] * resolution
    ys, xs = np.where(distance >= radius)
    if not len(xs):
        raise ValueError('map has no known-free spawn with enough robot clearance')
    wx = origin[0] + (xs + .5) * resolution
    wy = origin[1] + (ys + .5) * resolution
    k = int(np.argmin(wx * wx + wy * wy))
    return float(wx[k]), float(wy[k])


def build_world(map_yaml, geometry_file, output):
    """Keep the imported physical world; prepare an articulated robot for staged spawning."""
    import xml.etree.ElementTree as ET
    import xacro
    from ament_index_python.packages import get_package_share_directory
    grid, res, origin = map_data(map_yaml)
    g = yaml.safe_load(Path(geometry_file).read_text())
    x, y = spawn_pose(grid, res, origin, math.hypot(g['body_half_length_m'], g['body_half_width_m']) + g['safe_stop_gap_m'])
    package = Path(get_package_share_directory('go2_sim'))
    description = Path(get_package_share_directory('go2_description'))
    controllers = Path(get_package_share_directory('go2_joint_tracking_controller'))/'config/controllers.yaml'
    source_world = package/'worlds/custom.world'
    tree = ET.parse(source_world); world = tree.getroot().find('world')
    for saved in world.findall('state'): world.remove(saved)
    if world.find("model[@name='go2']") is not None: raise ValueError('source world already contains go2')
    state_plugin = ET.SubElement(world, 'plugin', name='sim_state', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(state_plugin, 'ros'), 'namespace').text = '/sim'
    ET.SubElement(state_plugin, 'update_rate').text = '30'
    camera = world.find('gui/camera')
    if camera is not None: camera.find('pose').text = f'{x+5} {y-7} 7 0 .7 2.2'
    tree.write(output, encoding='utf-8', xml_declaration=True)

    robot_xml = xacro.process_file(str(description/'xacro/go2.urdf.xacro'),
                                  mappings={'controllers_file':str(controllers),'lidar_type':'ray'}).toxml()
    robot = ET.fromstring(robot_xml)
    radar = robot.find("link[@name='radar']"); radar.set('name','mid360_link')
    inertial = radar.find('inertial')
    inertial.find('mass').set('value','.265')
    for key in ['ixx','iyy','izz']: inertial.find('inertia').set(key,'.00015')
    joint = robot.find("joint[@name='radar_joint']"); joint.set('name','mid360_joint')
    joint.find('child').set('link','mid360_link')
    xyz = [g['mid360_x_m'],g['mid360_y_m'],g['mid360_z_m']]
    rpy = [math.radians(g[key]) for key in ['mid360_roll_deg','mid360_pitch_deg','mid360_yaw_deg']]
    joint.find('origin').set('xyz',' '.join(map(str,xyz)))
    joint.find('origin').set('rpy',' '.join(map(str,rpy)))
    visual = ET.SubElement(radar,'visual')
    ET.SubElement(ET.SubElement(visual,'geometry'),'cylinder',radius='.0325',length='.06')
    ET.SubElement(ET.SubElement(visual,'material',name='mid360_dark'),'color',rgba='.15 .2 .25 1')
    sensor_gazebo = robot.find("gazebo[@reference='radar']"); sensor_gazebo.set('reference','mid360_link')
    sensor = sensor_gazebo.find('sensor'); sensor.set('name','mid360_angular_approximation')
    sensor.find('update_rate').text='10'
    sensor.find('.//horizontal/samples').text='360'
    sensor.find('.//vertical/samples').text='40'
    sensor.find('.//vertical/min_angle').text=str(math.radians(-7))
    sensor.find('.//vertical/max_angle').text=str(math.radians(52))
    sensor.find('.//range/min').text='.1'; sensor.find('.//range/max').text='20'
    plugin=sensor.find('plugin'); plugin.find('ros/namespace').text='/sim'
    plugin.find('ros/remapping').text='~/out:=/sim/mid360_points'
    plugin.find('frame_name').text='mid360_link'
    # No planar-move plugin: all body motion now comes from joints and ground contacts.
    robot_xml = ET.tostring(robot,encoding='unicode').replace('package://go2_description/',description.as_uri()+'/')
    robot_file=Path(output).with_suffix('.urdf'); robot_file.write_text(robot_xml)
    return {'spawn_x':x,'spawn_y':y,'spawn_z':.285,'world_source':str(source_world),
            'robot_description':str(robot_file),'controllers':str(controllers),
            'map':str(map_yaml),'mid360_xyz':xyz,'mid360_rpy':rpy,
            'sensor_source':'gazebo_ray_and_body_imu','motion':'articulated_reference_trot',
            'lidar':'regular_ray_angular_approximation_not_livox_scan_pattern','web_cloud':'none'}
