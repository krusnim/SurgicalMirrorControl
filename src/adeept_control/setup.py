from setuptools import find_packages, setup

package_name = 'adeept_control'

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
    maintainer='mel',
    maintainer_email='mel.krusniak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adeept_control = adeept_control.adeept_control:main',
            'adeept_kinematics = adeept_control.adeept_kinematics:main',
        ],
    },
)
