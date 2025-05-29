from setuptools import find_packages, setup

package_name = 'lowrance_driver'

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
    maintainer='koki-22',
    maintainer_email='amakou2626@gmail.com',
    description='RTSP映像をROS 2でimage_rawとして配信するノード',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_image_publisher = lowrance_driver.rtsp_image_publisher:main',
        ],
    },
)
