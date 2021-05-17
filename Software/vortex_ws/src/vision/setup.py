"""This is the image enhancement publisher_subscriber setup."""
from setuptools import setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vortex-co',
    maintainer_email='info@vortex-co.com',
    description=' vision package ',
    license='GNU GENERAL PUBLIC LICENSE',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'Enhancer = vision.image_enhancement_node:main',
        ],
    },
)
