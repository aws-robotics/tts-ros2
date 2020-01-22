from setuptools import find_packages
from setuptools import setup

package_name = 'tts'

setup(
    name=package_name,
    version='2.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml', 'launch/tts.launch.py']),
    ],
    package_data={
        'tts': ['services/data/models/polly/2016-06-10/*.json', 'services/data/*.ogg'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    author='RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'TTS'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polly_server = tts.services.amazonpolly:main',
            'synthesizer_server = tts.services.synthesizer:main',
            'voicer = tts.scripts.voicer:main',
        ],
    },
)
