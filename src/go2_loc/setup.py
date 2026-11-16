from glob import glob

from setuptools import setup

package_name = 'go2_loc'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'loc_pose = go2_loc.loc_pose:main',
        'loc_align = go2_loc.loc_align:main',
        'loc_map_check = go2_loc.loc_map_check:main',
    ]},
)
