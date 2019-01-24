
class PathValidator(object):
    """Helper class to test if the path is valid for the query and grid."""

    @staticmethod
    def is_valid_path(grid, query, path):
        if path[0] != query[0] or path[-1] != query[1]:
            return False

        for i in xrange(1, len(path)):
            if (not PathValidator.is_valid_transition(path[i - 1], path[i]) or
                    not PathValidator.is_valid_waypoint(path[i], grid)):
                return False

        return True

    @staticmethod
    def is_valid_waypoint(wp, grid):
        return 0 <= wp.x < grid.shape[0] and 0 <= wp.y < grid.shape[1] and not grid[wp.x, wp.y]

    @staticmethod
    def is_valid_transition(wp_a, wp_b):
        change_or = (wp_b.orientation - wp_a.orientation + 4) % 4
        change_xy = (wp_b.x - wp_a.x, wp_b.y - wp_a.y)

        vertical = {
            0: [(0, 1), (0, -1)],
            1: [(-1, -1), (1, 1)],
            2: [],
            3: [(-1, 1), (1, -1)]
        }

        horizontal = {
            0: [(1, 0), (-1, 0)],
            1: [(-1, 1), (1, -1)],
            2: [],
            3: [(-1, -1), (1, 1)]
        }

        return change_xy in (vertical[change_or] if wp_a.orientation % 2 == 0 else horizontal[change_or])
