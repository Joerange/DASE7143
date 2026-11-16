"""Constructor smoke tests against the installed ROS API; no sensor or API work."""
import importlib.util
import unittest
from unittest.mock import patch


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 Python is unavailable')
class PerceptionROSSmokeTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init(args=[])

    def tearDown(self):
        import rclpy
        if self.owns_context:
            rclpy.try_shutdown()

    def test_perception_constructs_without_overwriting_node_properties(self):
        from robot_perception.perception_node import PerceptionNode
        # Exercise all ROS publisher/subscriber/TF/synchronizer setup without
        # loading weights, starting background threads or touching sensors.
        with patch('threading.Thread.start'), patch('threading.Thread.join'):
            node = PerceptionNode()
            try:
                self.assertEqual(set(node.data_publishers), {'candidates', 'estimates', 'event'})
                self.assertFalse(node.model_ready)
            finally:
                node.destroy_node()

    def test_vlm_constructs_without_request(self):
        from robot_perception.vlm_confirm_node import VLMConfirmNode
        node = VLMConfirmNode()
        try:
            self.assertIsNone(node.active)
            self.assertEqual(node.seen, set())
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
