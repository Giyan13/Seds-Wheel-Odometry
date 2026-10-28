from setuptools import setup

package_name = 'wheel_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wheel_odometry.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giyan Chand',
    maintainer_email='giyanchand13042007@gmail.com',
    description='Wheel odometry node for the SEDS Pakistan rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry = wheel_odometry.odometry_node:main',
            'fake_joint_states = wheel_odometry.fake_joint_states:main',
        ],
    },
)
