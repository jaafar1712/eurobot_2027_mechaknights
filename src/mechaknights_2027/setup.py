from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mechaknights_2027'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'eurobot_table'), glob('models/eurobot_table/*.sdf') + glob('models/eurobot_table/*.config')),
        (os.path.join('share', package_name, 'models', 'eurobot_table', 'meshes'), glob('models/eurobot_table/meshes/*')),
        (os.path.join('share', package_name, 'models', 'eurobot_stone'), glob('models/eurobot_stone/*.sdf') + glob('models/eurobot_stone/*.config')),
        (os.path.join('share', package_name, 'models', 'eurobot_stone', 'meshes'), glob('models/eurobot_stone/meshes/*')),
        (os.path.join('share', package_name, 'models', 'simple_robot'), glob('models/simple_robot/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Freddy Liendo',
    maintainer_email='liendomf@univ-smb.fr',
    description='MechaKnights Eurobot 2027 simulation and launch files',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            
        ],
    },
)