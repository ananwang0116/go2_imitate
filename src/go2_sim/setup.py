from glob import glob
from setuptools import setup

setup(name='go2_sim', version='0.1.0', packages=['go2_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/go2_sim']),
                  ('share/go2_sim', ['package.xml']), ('share/go2_sim/launch', glob('launch/*.py')),
                  ('share/go2_sim/worlds', glob('worlds/*')), ('share/go2_sim/config', glob('config/*'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['sim_startup = go2_sim.startup:main', 'sim_bridge = go2_sim.bridge:main', 'sim_realtime = go2_sim.realtime:main',
          'sim_slam_inputs = go2_sim.slam_inputs:main', 'sim_slam_frames = go2_sim.slam_frames:main',
          'sim_online_mapper = go2_sim.online_mapping:main', 'sim_realtime_evidence = go2_sim.realtime_evidence:main']})
