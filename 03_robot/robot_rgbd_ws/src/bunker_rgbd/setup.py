from glob import glob
from setuptools import setup
setup(name='bunker_rgbd', version='0.1.0', packages=['bunker_rgbd'],
      data_files=[('share/ament_index/resource_index/packages',['resource/bunker_rgbd']),
                  ('share/bunker_rgbd',['package.xml']),
                  ('share/bunker_rgbd/launch',glob('launch/*.py')),
                  ('share/bunker_rgbd/config',glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='DASE7143',
      maintainer_email='robot@example.invalid', description='Standalone RGB-D visual docking',
      license='Apache-2.0', entry_points={'console_scripts':[
          'perception = bunker_rgbd.perception:main',
          'mission = bunker_rgbd.mission:main',
          'guard = bunker_rgbd.guard:main']})
