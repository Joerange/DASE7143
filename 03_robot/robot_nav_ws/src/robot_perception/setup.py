from setuptools import setup
from glob import glob
setup(name='robot_perception', version='0.1.0', packages=['robot_perception'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_perception']),
                  ('share/robot_perception', ['package.xml']),
                  ('share/robot_perception/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='DASE7143', maintainer_email='dase7143@example.invalid',
      description='Event-driven RGB-D target perception and bounded semantic confirmation', license='Apache-2.0',
      entry_points={'console_scripts': ['perception_node = robot_perception.perception_node:main',
                                        'vlm_confirm_node = robot_perception.vlm_confirm_node:main']})
