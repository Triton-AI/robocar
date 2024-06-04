from setuptools import setup

package_name = 'oakd_camera_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'depthai', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 node for OakD camera',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oakd_camera_node = oakd_camera_node.oakd_camera_node:main'
        ],
    },
)
