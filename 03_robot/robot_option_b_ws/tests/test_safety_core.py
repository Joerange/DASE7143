"""Fault injection against the pure guard; does not publish ROS commands."""
import math
import unittest

from robot_safety.core import Config, Guard, Sample, scan_points, swept_collision


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.guard = Guard(Config(motion_enabled=True))
        self.now = 10.0
        self.health()
        self.mission()
        self.guard.tick(self.now, self.now)  # consume zero-speed rearm barrier

    def health(self):
        g, t = self.guard, self.now
        g.scan = Sample(t, t, scan_points([3.0] * 720, -math.pi, math.pi / 360,
                                        0.10, 10.0, g.config))
        g.depth = Sample(t, t, [])
        g.odom = Sample(t, t, (0.0, 0.0))
        g.localization = Sample(t, t, (0.01, 0.02))
        g.tf = Sample(t, t, True)

    def mission(self, source='search', state='SEARCH_CONTINUOUS', epoch=1, generation='one'):
        value = dict(schema=1, stamp=self.now, task_generation=generation, state=state,
                     source=source, control_epoch=epoch, motion_enabled=True, target_track_id='bucket')
        self.guard.update_mission(value, self.now)
        return value

    def nav(self, state='NAVIGATE'):
        self.mission(source='nav', state=state, epoch=2)
        self.guard.tick(self.now, self.now)
        self.guard.command('nav', .1, 0., self.now)
        self.guard.command('nav_smoothed', .1, 0., self.now)
        self.guard.speed_limit(dict(schema=1, stamp=self.now, task_generation='one',
                                    control_epoch=2, valid=True, max_linear=.1), self.now)

    def test_search_does_not_need_target(self):
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now + .02, self.now + .02)
        self.assertGreater(velocity[1], 0.)
        self.assertEqual(status['reason'], 'authorized')

    def test_disarmed_never_moves(self):
        self.guard.config = Config(motion_enabled=False)
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(velocity, (0., 0.))
        self.assertTrue(status['ready'])
        self.assertEqual(status['reason'], 'disarmed')

    def test_command_watchdog(self):
        self.guard.command('search', 0., .15, self.now)
        self.now += .21
        self.health()
        self.mission()
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'search_command_stale')

    def test_smoothing_cannot_refresh_dead_upstream(self):
        self.nav()
        self.now += .21
        self.health()
        self.mission(source='nav', state='NAVIGATE', epoch=2)
        self.guard.command('nav_smoothed', .1, 0., self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'nav_command_stale')

    def test_late_old_epoch_or_generation_rejected(self):
        self.mission(epoch=5)
        self.assertFalse(self.guard.update_mission(dict(schema=1, stamp=self.now,
            task_generation='one', state='NAVIGATE', source='nav', control_epoch=4, motion_enabled=True), self.now))
        self.assertEqual(self.guard.identity, ('one', 5, 'search'))
        self.mission(epoch=0, generation='two')
        self.assertFalse(self.guard.update_mission(dict(schema=1, stamp=self.now,
            task_generation='one', state='SEARCH_CONTINUOUS', source='search',
            control_epoch=6, motion_enabled=True), self.now))

    def test_source_change_clears_command_and_requires_zero(self):
        self.guard.command('search', 0., .15, self.now)
        self.guard.odom.value = (0., .10)
        self.mission(source='nav', state='NAVIGATE', epoch=2)
        self.guard.command('nav', .1, 0., self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(status['reason'], 'waiting_for_actual_stop')
        self.assertFalse(self.guard.commands)
        self.assertEqual(velocity, (0., 0.))

    def test_acquisition_stamp_cannot_be_refreshed_by_receipt(self):
        self.guard.scan.stamp = self.now - 1.
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(status['reason'], 'scan_stale')
        self.assertFalse(status['ready'])

    def test_depth_missing_or_invalid_stops(self):
        self.guard.depth.value = None
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(status['reason'], 'depth_invalid')

    def test_scan_empty_not_free(self):
        for ranges in ([], [float('inf')] * 720, [float('nan')] * 720):
            value = scan_points(ranges, -math.pi, math.pi/360, .1, 10., self.guard.config)
            self.assertFalse(value['valid'])

    def test_missing_rear_sector_prevents_turn(self):
        self.guard.scan.value['sectors'][0] = 0.
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(status['reason'], 'scan_sector_unknown')

    def test_rotating_rectangular_corner_is_protected(self):
        # Outside current side envelope, inside the corner's imminent sweep.
        points = [(0.40, 0.38)]
        self.assertTrue(swept_collision(points, (0., .2), (0., .2), Config()))
        self.assertFalse(swept_collision([(0.8, 0.8)], (0., .2), (0., .2), Config()))

    def test_braking_distance_not_just_center_threshold(self):
        self.assertTrue(swept_collision([(0.47, 0.)], (.1, 0.), (.1, 0.), Config()))
        self.assertFalse(swept_collision([(0.9, 0.)], (.1, 0.), (.1, 0.), Config()))

    def test_low_depth_obstacle_stops_immediately(self):
        self.nav()
        self.guard.output = (.1, 0.)
        self.guard.depth.value = [(0.40, 0.)]
        velocity, status = self.guard.tick(self.now + .02, self.now + .02)
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'obstacle_in_braking_sweep')
        self.assertFalse(self.guard.commands)

    def test_approach_rejects_stale_target(self):
        self.nav('APPROACH')
        velocity, status = self.guard.tick(self.now + .02, self.now + .02)
        self.assertEqual(status['reason'], 'target_stale')

    def test_approach_uses_physical_gap_and_current_identity(self):
        self.nav('APPROACH')
        self.guard.target_update(dict(schema=1, stamp=self.now, task_generation='one',
            targets=[dict(track_id='bucket', stamp=self.now, gap=.54, bearing=0., valid=True)]), self.now)
        velocity, status = self.guard.tick(self.now + .02, self.now + .02)
        self.assertEqual(status['reason'], 'target_minimum_gap')

    def test_reverse_forbidden(self):
        self.nav()
        self.guard.command('nav_smoothed', -.1, 0., self.now)
        velocity, status = self.guard.tick(self.now + .02, self.now + .02)
        self.assertEqual(status['reason'], 'reverse_disabled')

    def test_lower_speed_cap_overrides_smoothing(self):
        self.nav()
        self.guard.output = (.1, 0.)
        self.guard.speed_limit(dict(schema=1, stamp=self.now, task_generation='one',
                                    control_epoch=2, valid=True, max_linear=.02), self.now)
        velocity, status = self.guard.tick(self.now + .02, self.now + .02)
        self.assertLessEqual(velocity[0], .02)

    def test_localization_covariance_rejects_bad_pose(self):
        self.guard.localization.value = (.5, .01)
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(status['reason'], 'localization_uncertain')

    def test_clock_paused_still_expires_receipt(self):
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now + 1., self.now)
        self.assertEqual(velocity, (0., 0.))
        self.assertFalse(status['ready'])

    def test_measured_overspeed_fails_closed(self):
        self.guard.odom.value = (.5, 0.)
        self.guard.command('search', 0., .15, self.now)
        velocity, status = self.guard.tick(self.now, self.now)
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(status['reason'], 'measured_overspeed')


if __name__ == '__main__':
    unittest.main()
