from glob import glob

from setuptools import setup

package_name = 'go2_mot'
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
        'mot_cmd_mux = go2_mot.mot_cmd_mux:main',
        'mot_recover = go2_mot.mot_recover:main',
    ]},
)
