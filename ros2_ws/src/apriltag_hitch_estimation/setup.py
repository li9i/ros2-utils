from setuptools import setup

package_name = 'apriltag_hitch_estimation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_array_pose_estimation.launch.xml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_hitch_estimation.launch.xml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_ros.launch.xml']),
        ('share/' + package_name + '/launch', ['launch/hitch_joint_estimation.launch.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li9i',
    maintainer_email='alefilot@auth.gr',
    description='A package that estimates the hitch joint state between a robot and a trailer by optical recognition of an array of april tags mounted at the front of the trailer by the rear camera of the robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_array_pose_estimation_node = apriltag_hitch_estimation.apriltag_array_pose_estimation_node:main',
            'hitch_joint_estimation_node = apriltag_hitch_estimation.hitch_joint_estimation_node:main',
        ],
    },
)
