from setuptools import find_packages, setup

package_name = 'my_audio_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'sounddevice',
    ],
    zip_safe=True,
    maintainer='hira',
    maintainer_email='akash.hira.c3@s.gifu-u.ac.jp',
    description='ROS2 node for real-time audio localization using an 8-microphone array',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
       
            'audio_localization_node  = my_audio_node.microphone_lake_testing:main',
            'microphone_lake_testing = my_audio_node.microphone_lake_testing:main', 
            'bag_to_wav_saver       = my_audio_node.bag_to_wav_saver:main',
            'bag_to_wav_converter   = my_audio_node.bag_to_wav_converter:main',
        ],
    },
)

