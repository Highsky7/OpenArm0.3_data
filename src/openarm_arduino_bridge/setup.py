from setuptools import setup
import sys

# Workaround for 'error: option --editable not recognized' with recent setuptools/colcon
# Also handles --build-directory which takes an argument
if '--editable' in sys.argv:
    sys.argv.remove('--editable')

if '--build-directory' in sys.argv:
    idx = sys.argv.index('--build-directory')
    sys.argv.pop(idx) # Remove flag
    if idx < len(sys.argv):
        sys.argv.pop(idx) # Remove value

package_name = 'openarm_arduino_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arduino_servo.launch.py', 'launch/bimanual_arduino_servo.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'], 
    zip_safe=True,
    maintainer='openarm',
    maintainer_email='openarm@example.com',
    description='Arduino bridge for OpenArm gripper control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_bridge = openarm_arduino_bridge.bridge_node:main',
            'bimanual_servo_bridge = openarm_arduino_bridge.bimanual_bridge_node:main',
        ],
    },
)
