from setuptools import find_packages, setup

package_name = 'rover_control_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='hasi',
    maintainer_email='hasi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_control = rover_control_test.rover_control:main',
            'keyboard_control = rover_control_test.rover_keyboard_control:main'
        ],
    },
)
