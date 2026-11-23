from glob import glob
from setuptools import setup,find_packages
setup(name='pointcloud_crop',version='0.1.0',description='PointCloud2 crop and robot self-return removal',python_requires='>=3.10',
      packages=find_packages(exclude=['test']),install_requires=['numpy'],extras_require={'test': ['pytest']},
      license='Apache-2.0',maintainer='PyLoT Robotics',maintainer_email='163533636+PyLoT-Robotics@users.noreply.github.com',
      data_files=[('share/ament_index/resource_index/packages',['resource/pointcloud_crop']),
                  ('share/pointcloud_crop',['package.xml']),('share/pointcloud_crop/config',glob('config/*')),
                  ('share/pointcloud_crop/launch',glob('launch/*.py'))],
      entry_points={'console_scripts':['ros2-pointcloud-crop = pointcloud_crop.cli:main','pointcloud_crop_node = pointcloud_crop.node:main']})
