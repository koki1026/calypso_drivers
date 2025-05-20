from setuptools import find_packages, setup

package_name = 'propeller_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (f'share/{package_name}/launch', ['launch/propeller_teleop_launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koki-22',
    maintainer_email='amakou2626@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_joy_node = propeller_teleop.teleop_joy_node:main',
        ],
    },
)
