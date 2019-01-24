#!/usr/bin/env python
import sys
import numpy as np

import unittest

from path_finder import PathFinder
from path_validator import PathValidator
from path_visualizer import PathVisualizer
from waypoint import Waypoint


class Challenge1TestCase(unittest.TestCase):

    VIZ = False

    def setUp(self):
        self.path_finder = PathFinder()

    def tearDown(self):
        self.path_finder = None

    def _run_test(self, grid, queries):
        for query in queries:
            path = self.path_finder.get_path(grid, query[0], query[1])
            if Challenge1TestCase.VIZ:
                PathVisualizer.viz_path(grid, query, path)
            self.assertTrue(PathValidator.is_valid_path(grid, query, path),
                            "Invalid path %s for query %s." % (path, query))

    def test_no_obstacles_straight_line(self):
        grid = np.zeros((20, 20)).astype(np.bool)
        queries = [
            [Waypoint(5, 5, 0), Waypoint(5, 8, 0)],
            [Waypoint(16, 5, 1), Waypoint(8, 5, 1)],
            [Waypoint(5, 15, 3), Waypoint(16, 15, 3)],
        ]
        self._run_test(grid, queries)

    def test_no_obstacles_with_turns(self):
        grid = np.zeros((20, 20)).astype(np.bool)
        queries = [
            [Waypoint(5, 7, 0), Waypoint(15, 8, 3)],
            [Waypoint(16, 5, 2), Waypoint(8, 5, 1)],
            [Waypoint(15, 15, 1), Waypoint(16, 15, 3)],
        ]
        self._run_test(grid, queries)

    def test_with_one_obstacle(self):
        grid = np.zeros((20, 20)).astype(np.bool)
        grid[10:14, 10:14] = True
        queries = [
            [Waypoint(5, 7, 0), Waypoint(15, 11, 3)],
            [Waypoint(16, 5, 2), Waypoint(8, 5, 1)],
            [Waypoint(15, 15, 1), Waypoint(16, 15, 3)],
        ]
        self._run_test(grid, queries)

    def test_with_multiple_obstacles(self):
        grid = np.zeros((20, 20)).astype(np.bool)

        grid[3:4, 0:15] = True
        grid[13:14, 5:20] = True

        queries = [
            [Waypoint(0, 0, 0), Waypoint(19, 19, 3)]
        ]
        self._run_test(grid, queries)


if __name__ == '__main__':
    if '--viz' in sys.argv:
        Challenge1TestCase.VIZ = True
        sys.argv.pop()
    unittest.main()
