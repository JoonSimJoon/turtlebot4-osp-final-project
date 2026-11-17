from glob import glob
from setuptools import find_packages, setup

setup(
    name='tb4_mission',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tb4_mission']),
        ('share/tb4_mission', ['package.xml']),
        ('share/tb4_mission/launch', glob('launch/*.launch.py')),
        ('share/tb4_mission/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JoonSimJoon',
    maintainer_email='43689295+JoonSimJoon@users.noreply.github.com',
    description='TurtleBot4 autonomous SLAM and target finding mission',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'explorer = tb4_mission.explorer:main',
        'marker_detector = tb4_mission.marker_detector:main',
        'doctor = tb4_mission.doctor:main',
        'origin_anchor = tb4_mission.origin_anchor:main',
    ]},
)
