from setuptools import setup

setup(
    name='robot_safety', version='0.1.0', packages=['robot_safety'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/robot_safety']),
                ('share/robot_safety', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='DASE7143 project', maintainer_email='dase7143@example.invalid',
    description='Independent fail-closed Bunker command arbiter and watchdog.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['command_guard_node = robot_safety.command_guard_node:main']})
