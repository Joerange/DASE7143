"""Configuration and launch routing tests, independent of a running ROS graph."""
import ast
import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]/'src/robot_bringup'


class Action:
    def __init__(self, *args, **kwargs): self.args, self.kwargs = args, kwargs


class Configuration:
    def __init__(self,name): self.name = name
    def perform(self,context): return context[self.name]


def load_launch(name):
    modules = {}
    def module(name, **attributes):
        result = types.ModuleType(name)
        result.__dict__.update(attributes)
        modules[name] = result
    module('ament_index_python')
    module('ament_index_python.packages',get_package_share_directory=lambda name:str(ROOT))
    module('launch',LaunchDescription=Action)
    module('launch.actions',DeclareLaunchArgument=Action,IncludeLaunchDescription=Action,OpaqueFunction=Action,LogInfo=Action)
    module('launch.substitutions',LaunchConfiguration=Configuration)
    module('launch.launch_description_sources',PythonLaunchDescriptionSource=Action)
    module('launch_ros')
    module('launch_ros.actions',Node=Action)
    module('nav2_common')
    module('nav2_common.launch',RewrittenYaml=Action)
    spec = importlib.util.spec_from_file_location(name, ROOT/'launch'/name)
    obj = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,modules): spec.loader.exec_module(obj)
    return obj


class BringupTests(unittest.TestCase):
    def setUp(self):
        self.nav = yaml.safe_load((ROOT/'config/nav2.yaml').read_text())

    def test_same_physical_footprint_padding_applied_once(self):
        reference = yaml.safe_load((ROOT/'config/nav2_footprint.yaml').read_text())
        for which in ['local_costmap','global_costmap']:
            cost = self.nav[which][which]['ros__parameters']
            expected = reference[which][which]['ros__parameters']
            self.assertEqual(json.loads(cost['footprint']),json.loads(expected['footprint']))
            self.assertEqual(cost['footprint_padding'],.05)
            self.assertIn('obstacle_layer',cost['plugins'])
            self.assertTrue(cost['track_unknown_space'])
            self.assertFalse(cost['obstacle_layer']['scan']['inf_is_valid'])

    def test_motion_limits_and_full_footprint_planner(self):
        c = self.nav['controller_server']['ros__parameters']
        self.assertIn('ObstacleFootprint',c['FollowPath']['critics'])
        self.assertEqual(c['FollowPath']['min_vel_x'],0.)
        self.assertLessEqual(c['FollowPath']['max_vel_x'],.10)
        self.assertEqual(c['FollowPath']['max_vel_y'],0.)
        self.assertLessEqual(c['general_goal_checker']['xy_goal_tolerance'],.03)
        self.assertFalse(self.nav['planner_server']['ros__parameters']['GridBased']['allow_unknown'])
        s = self.nav['velocity_smoother']['ros__parameters']
        self.assertEqual(s['smoothing_frequency'],50.)
        guard=yaml.safe_load((ROOT/'config/mission.yaml').read_text())['command_guard_node']['ros__parameters']
        self.assertLessEqual(s['velocity_timeout'],guard['command_timeout'])
        self.assertLessEqual(s['velocity_timeout'],.3)
        self.assertEqual(s['min_velocity'][0],0.)

    def test_recovery_cannot_reverse_spin_or_delete_obstacles(self):
        self.assertEqual(self.nav['behavior_server']['ros__parameters']['behavior_plugins'],['wait'])
        for file in (ROOT/'behavior_trees').glob('*.xml'):
            tree = ET.parse(file)
            tags = {n.tag for n in tree.iter()}
            self.assertIn('Wait',tags)
            self.assertFalse(tags & {'Spin','BackUp','DriveOnHeading','ClearEntireCostmap'})
            for element in tree.iter('RecoveryNode'):
                self.assertLessEqual(int(element.attrib['number_of_retries']),2)

    def task_context(self):
        return {'use_sim_time':'false','motion_enabled':'false','start_chassis':'false','allow_runtime_control':'false',
                'start_sensors':'false','enable_navigation':'false','enable_perception':'false',
                'autostart':'false','target_label':'trash can','rviz':'false','can_interface':'can0',
                'nav_autostart':'true','nav_params_file':str(ROOT/'config/nav2.yaml')}

    def test_default_task_is_disarmed_without_chassis_or_nav(self):
        obj = load_launch('task.launch.py')
        actions = obj.setup(self.task_context())
        nodes = [a.kwargs for a in actions if 'package' in a.kwargs]
        self.assertEqual({n['package'] for n in nodes},{'robot_safety','robot_mission'})
        for n in nodes: self.assertFalse(n['parameters'][-1]['motion_enabled'])
        mission = next(n for n in nodes if n['package']=='robot_mission')
        self.assertFalse(mission['parameters'][-1]['autostart'])
        ctx = self.task_context();ctx['start_chassis']='true'
        with self.assertRaises(ValueError): obj.setup(ctx)
        ctx = self.task_context();ctx['motion_enabled']='true'
        with self.assertRaises(ValueError): obj.setup(ctx)

    def test_nav_velocities_never_bypass_guard(self):
        obj = load_launch('nav2.launch.py')
        nodes = obj.setup({'use_sim_time':'false','nav_autostart':'true','nav_params_file':str(ROOT/'config/nav2.yaml')})
        commands={n.kwargs['executable']:n.kwargs.get('remappings',[]) for n in nodes}
        self.assertIn(('cmd_vel','/cmd_vel_nav'),commands['controller_server'])
        self.assertIn(('cmd_vel','/cmd_vel_nav'),commands['behavior_server'])
        self.assertIn(('cmd_vel_smoothed','/cmd_vel_nav_smoothed'),commands['velocity_smoother'])
        self.assertNotIn('/cmd_vel',[target for pairs in commands.values() for _,target in pairs])
        self.assertFalse({'amcl','map_server'} & set(commands))
        self.assertEqual(sum(n.kwargs.get('executable')=='async_slam_toolbox_node' for n in nodes),1)

    def test_launch_cannot_grant_motion_or_start_unsupervised_driver(self):
        obj = load_launch('task.launch.py')
        for retired in ('motion_enabled','start_chassis'):
            ctx = self.task_context()
            ctx.update(enable_navigation='true',allow_runtime_control='true')
            ctx[retired] = 'true'
            with self.assertRaisesRegex(ValueError, 'retired'):
                obj.setup(ctx)
        ctx = self.task_context()
        ctx.update(enable_navigation='true',allow_runtime_control='true')
        actions = obj.setup(ctx)
        nodes = [a.kwargs for a in actions if 'package' in a.kwargs]
        self.assertFalse(any(n['package']=='bunker_base' for n in nodes))
        for node in nodes:
            self.assertFalse(node['parameters'][-1]['motion_enabled'])
            self.assertTrue(node['parameters'][-1]['allow_runtime_control'])

    def test_online_navigation_needs_no_map_argument_or_saved_graph(self):
        obj = load_launch('nav2.launch.py')
        # An empty mapping context has no map path; setup succeeds with only flags.
        nodes = obj.setup({'use_sim_time':'false','nav_autostart':'true','nav_params_file':str(ROOT/'config/nav2.yaml')})
        declared = {a.args[0] for a in obj.generate_launch_description().args[0] if a.args}
        self.assertNotIn('map', declared)
        self.assertNotIn('map_server', self.nav)
        self.assertNotIn('amcl', self.nav)
        slam = next(n.kwargs for n in nodes if n.kwargs.get('package') == 'slam_toolbox')
        self.assertIn(('pose', '/slam_toolbox/pose'), slam['remappings'])
        cfg = yaml.safe_load((ROOT/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
        self.assertEqual(cfg['mode'], 'mapping')
        self.assertEqual(cfg['map_file_name'], '')
        self.assertEqual((cfg['map_frame'], cfg['odom_frame'], cfg['base_frame']), ('map','odom','base_link'))
        self.assertEqual(cfg['scan_topic'], '/scan')
        self.assertFalse(cfg['use_map_saver'])

    def test_task_enables_online_slam_without_fixed_map_route(self):
        obj = load_launch('task.launch.py')
        ctx = self.task_context();ctx['enable_navigation'] = 'true'
        actions = obj.setup(ctx)
        includes = [a for a in actions if 'launch_arguments' in a.kwargs]
        nav = next(a for a in includes if a.args[0].args[0].endswith('nav2.launch.py'))
        self.assertNotIn('map', dict(nav.kwargs['launch_arguments']))
        guard = next(a.kwargs for a in actions if a.kwargs.get('package') == 'robot_safety')
        self.assertEqual(guard['parameters'][-1]['localization_mode'], 'slam')
        self.assertFalse(guard['parameters'][-1]['motion_enabled'])
        self.assertFalse(any(a.kwargs.get('package') in ('nav2_amcl', 'nav2_map_server', 'bunker_base') for a in actions))

    def test_continuously_updated_map_and_pure_rotation_slam(self):
        static = self.nav['global_costmap']['global_costmap']['ros__parameters']['static_layer']
        self.assertEqual(static['map_topic'], '/map')
        self.assertTrue(static['map_subscribe_transient_local'])
        self.assertTrue(static['subscribe_to_updates'])
        self.assertFalse(static['footprint_clearing_enabled'])
        cfg = yaml.safe_load((ROOT/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
        self.assertFalse(cfg['restamp_tf'])
        self.assertEqual(cfg['minimum_travel_distance'], 0.)
        self.assertGreaterEqual(cfg['minimum_time_interval'], .2)
        self.assertLessEqual(cfg['map_update_interval'], 1.)

    def test_navigation_lifecycle_excludes_localizers_and_slam_normal_node(self):
        obj = load_launch('nav2.launch.py')
        nodes = obj.setup({'use_sim_time':'false', 'nav_autostart':'false'})
        managers = [a.kwargs for a in nodes if a.kwargs.get('package') == 'nav2_lifecycle_manager']
        self.assertEqual(len(managers), 1)
        params = managers[0]['parameters'][0]
        self.assertFalse(params['autostart'])
        self.assertEqual(set(params['node_names']), {'planner_server','controller_server','behavior_server',
                                                    'bt_navigator','velocity_smoother'})

    def test_scan_keeps_close_side_obstacles_and_unknown_returns(self):
        scan=yaml.safe_load((ROOT/'config/scan.yaml').read_text())['pointcloud_to_laserscan']['ros__parameters']
        self.assertLessEqual(scan['range_min'],.10)
        self.assertTrue(scan['use_inf'])
        self.assertEqual(scan['target_frame'],'base_link')
        obj=load_launch('sensors.launch.py')
        nodes=obj.setup({'use_sim_time':'false','start_sensors':'false'})
        converter=next(a.kwargs for a in nodes if a.kwargs.get('package')=='pointcloud_to_laserscan')
        self.assertIn(('cloud_in','/perception/lidar_filtered'),converter['remappings'])
        self.assertTrue(any(a.kwargs.get('executable')=='lidar_filter_node' for a in nodes))

    def test_mapping_owns_map_tf_without_amcl_or_motion(self):
        obj=load_launch('mapping.launch.py')
        nodes=obj.setup({'use_sim_time':'false','start_sensors':'false'})
        packages={a.kwargs.get('package') for a in nodes}
        self.assertIn('slam_toolbox',packages)
        self.assertFalse(packages & {'bunker_base','nav2_amcl','robot_mission'})

    def test_sources_compile_and_xml_valid(self):
        for p in (ROOT/'launch').glob('*.py'): ast.parse(p.read_text(),filename=str(p))
        ast.parse((ROOT/'scripts/lidar_filter_node').read_text())
        ET.parse(ROOT/'package.xml')


if __name__ == '__main__': unittest.main()
