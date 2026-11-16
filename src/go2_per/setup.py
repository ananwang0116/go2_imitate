from glob import glob

from setuptools import setup

package_name = 'go2_per'
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
        'per_scan = go2_per.per_scan:main',
        'per_evidence = go2_per.per_evidence:main',
        'per_map = go2_per.per_map:main',
    ]},
)
