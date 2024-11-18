from setuptools import setup

package_name = 'gps_to_global_coordinates'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'gps_to_global_coordinates.gps_to_global_coordinates_node',
    ],
    install_requires=['setuptools'],
    # Remove the data_files section entirely or comment it out if you don't have resource files
    # data_files=[
    #     ('share/ament_index/resource_index/ament_python', ['resource/' + package_name + '.resource']),
    # ],
    entry_points={
        'console_scripts': [
            'gps_to_global_coordinates_node = gps_to_global_coordinates.gps_to_global_coordinates_node:main',
        ],
    },
)

