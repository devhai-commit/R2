import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'r2_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'models', 'rack'),
            glob('worlds/models/rack/model.*')),
        (os.path.join('share', package_name, 'worlds', 'models', 'rack', 'meshes'),
            glob('worlds/models/rack/meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf', 'components'), glob('urdf/components/*.xacro')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL') + glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devhai',
    maintainer_email='devnnhai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apply_kfs_layout = r2_sim.apply_kfs_layout:main',
            'navigator = r2_sim.navigator:main',
            'kfs_collector = r2_sim.kfs_collector:main',
            'r1_sim = r2_sim.r1_sim:main',
        ],
    },
)
