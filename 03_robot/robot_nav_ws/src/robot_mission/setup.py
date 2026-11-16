from setuptools import setup
setup(name='robot_mission', version='0.1.0', packages=['robot_mission'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_mission']),
                  ('share/robot_mission', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='DASE7143', maintainer_email='dase7143@example.invalid',
      description='Bounded trash-bin search and docking mission', license='Apache-2.0',
      entry_points={'console_scripts': ['mission_node = robot_mission.node:main']})
