class Waypoint(object):
    """Class to represent the position and orientation of Mary Anne.

    Note about orientation. Orientation is an integer between 0 and 3 inclusive.
        0 maps to the unit vector (0, 1)
        1 maps to the unit vector (1, 0)
        2 maps to the unit vector (0, -1)
        3 maps to the unit vector (-1, 0)
    An easy way to think about the orientation is that 0 is North, 1 is East, 2 is South, and 3 is West.
    """
    def __init__(self, x, y, orientation):
        self._x = x
        self._y = y
        self._orientation = orientation

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def orientation(self):
        return self._orientation

    @property
    def tuple(self):
        return tuple((self._x, self._y, self._orientation))

    def __eq__(self, other):
        return self._x == other.x and self._y == other.y and self._orientation == other.orientation

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return 'Waypoint(%s, %s, %s)' % (self._x, self._y, self._orientation)

    def __repr__(self):
        return str(self)

    def __hash__(self):
        return hash((self._x, self._y, self._orientation))
