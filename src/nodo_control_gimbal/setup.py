from setuptools import find_packages, setup

package_name = 'nodo_control_gimbal'

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
    maintainer='alpha',
    maintainer_email='alpha@todo.todo',
    description='Nodo de control del gimbal',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_control_gimbal = nodo_control_gimbal.nodo_control_gimbal:main',
        ],
    },
)

