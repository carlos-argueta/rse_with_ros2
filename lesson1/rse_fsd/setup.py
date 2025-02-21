from setuptools import find_packages, setup

package_name = 'rse_fsd'

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
    maintainer='carlos',
    maintainer_email='carlos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_center_node = rse_fsd.control_center_node:main',
            'dashcam_node = rse_fsd.dashcam_node:main',
        ],
    },
)
