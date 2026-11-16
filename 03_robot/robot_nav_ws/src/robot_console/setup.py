from setuptools import setup
setup(name='robot_console',version='0.1.0',packages=['robot_console'],
      data_files=[('share/ament_index/resource_index/packages',['resource/robot_console']),('share/robot_console',['package.xml'])],
      install_requires=['setuptools'],zip_safe=True,maintainer='DASE7143',maintainer_email='dase7143@example.invalid',
      description='Local Jetson operator console with independent guarded authority',license='Apache-2.0',
      entry_points={'console_scripts':['console = robot_console.ui:main','control_node = robot_console.backend:main']})
