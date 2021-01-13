from setuptools import setup

package_name = 'px4_py'

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
    maintainer='vortex',
    maintainer_email='info@vortex-co.com',
    description='px4_py',
    license='GNU GENERAL PUBLIC LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
 	"px4 =px4_py.px4_test:main"
        ],
    },
)
