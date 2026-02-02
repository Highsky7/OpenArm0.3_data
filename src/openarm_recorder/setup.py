from setuptools import find_packages, setup
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

package_name = 'openarm_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/record_waypoints.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mintlabskh',
    maintainer_email='mintlabskh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'recorder_node = openarm_recorder.recorder_node:main',
        ],
    },
)
