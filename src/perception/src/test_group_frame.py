import unittest
from math import pi
from group_frame import GroupFramePublisher


class TestGroupFrame(unittest.TestCase):
    def test_compute_frame_2_robots(self):
		config = [(0, 1), (0, -1)]
		group_frame_publisher = GroupFramePublisher(config)
		translations = [(4, 0), (0, 0)]

		origin, angle = group_frame_publisher.compute_frame(translations)

		self.assertEqual(origin, (2, 0))
		self.assertAlmostEqual(angle, pi/2)

    def test_compute_frame__square_robots(self):
		config = [(0, 1), (-1, 0), (0, -1), (1, 0)]
		group_frame_publisher = GroupFramePublisher(config)
		translations = [(1, 1), (-1, 1), (-1, -1), (1, -1)]

		origin, angle = group_frame_publisher.compute_frame(translations)

		self.assertEqual(origin, (0, 0))
		self.assertAlmostEqual(angle, pi/4)


if __name__ == '__main__':
    unittest.main()