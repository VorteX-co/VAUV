from setuptools import setup

package_name = 'motion_planning_py'

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
    maintainer='islam',
    maintainer_email='info@vortex-co.com',
    description='motion planning python package',
    license='GNU GENERAL PUBLIC LICENSE',
    entry_points={
        'console_scripts': [
        ],
    },
)
