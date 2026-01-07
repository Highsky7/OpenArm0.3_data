from setuptools import find_packages, setup

package_name = 'openarm_dataset_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mintlabskh',
    maintainer_email='mintlabskh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'encoder_reader = openarm_dataset_node.encoder_reader:main',
        ],
    },
)
