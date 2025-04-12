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
    maintainer_email='kid.a.rgueta@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_node = rse_fsd.command_node:main',
            'camera_node = rse_fsd.camera_node:main',
            'camera_node_blocking = rse_fsd.camera_node_blocking:main',
            'class_counts_client = rse_fsd.class_counts_client:main',
            'class_counts_server = rse_fsd.class_counts_server:main',
            'find_object_server = rse_fsd.find_object_server:main',
            'find_object_client = rse_fsd.find_object_client:main',
        ],
    },
)
