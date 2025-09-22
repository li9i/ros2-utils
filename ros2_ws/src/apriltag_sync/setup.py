from setuptools import setup

package_name = 'apriltag_sync'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
      ('share/' + package_name, ['package.xml']),
      ('share/' + package_name + '/launch', ['launch/apriltag_sync.launch.xml']),
      ('share/' + package_name + '/launch', ['launch/apriltag_sync_sfr_v2.launch.xml']),
      ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
      ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li9i',
    maintainer_email='oh@no.com',
    description='Synchronise image and camera info topics => needed by apriltag detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
      'console_scripts': [
        'apriltag_sync_node = apriltag_sync.apriltag_sync_node:main',
        ],
      },
    )
