"""Stream framing and map protocol checks; no Bluetooth connection."""
import unittest
from communication.android_protocol import AndroidState, FrameBuffer, to_frame


class ProtocolTests(unittest.TestCase):
    def test_fragmented_frames(self):
        frames = FrameBuffer()
        self.assertEqual(frames.feed('//OBSTA'), [])
        self.assertEqual(frames.feed('CLE,1,5,7,E////BEGIN,1//'),
                         ['OBSTACLE,1,5,7,E', 'BEGIN,1'])

    def test_map_then_begin_passes_robot_pose_and_obstacles(self):
        state = AndroidState()
        state.handle('MAP/ROBOT,10,2,N/OBSTACLE,1,10,15,S/OBSTACLE,2,18,10,W')
        messages = state.handle('BEGIN,1')
        self.assertEqual([m['cat'] for m in messages], ['obstacles', 'control'])
        self.assertEqual(messages[0]['value']['robot'], {'x': 10, 'y': 2, 'd': 0})
        self.assertEqual(messages[0]['value']['obstacles'][0], {'id': 1, 'x': 10, 'y': 15, 'd': 4})

    def test_reset_map_clears_obstacles(self):
        state = AndroidState()
        state.handle('OBSTACLE,1,5,7,E')
        state.handle('MAP')
        self.assertEqual(state.obstacles, {})

    def test_target_uses_numeric_image_id(self):
        self.assertEqual(to_frame('image-rec', {'obstacle_id': '1', 'image_id': '38'}),
                         '//TARGET,1,38//')
        self.assertIsNone(to_frame('image-rec', {'obstacle_id': '1', 'image_id': 'NA'}))

    def test_compass_location(self):
        self.assertEqual(to_frame('location', {'x': 10, 'y': 13, 'd': 0}), '//ROBOT,10,13,N//')
