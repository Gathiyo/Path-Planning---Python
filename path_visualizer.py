import numpy as np
import matplotlib.pyplot as plt


class PathVisualizer(object):
    """Helper class to visualize the path, query and grid."""

    @staticmethod
    def viz_path(grid, query, path):
        plt.axis([0, grid.shape[0], 0, grid.shape[1]])
        plt.scatter(np.where(grid)[0] + 0.5, np.where(grid)[1] + 0.5, s=65, marker='s')

        for wp in path:
            switch = {
                0: (0, 0.25),
                1: (0.25, 0),
                2: (0, -0.25),
                3: (-0.25, 0)
            }
            plt.arrow(wp.x, wp.y, *switch[wp.orientation], width=0.02, capstyle='projecting')
            plt.text(wp.x + 0.3, wp.y + 0.3, str(wp.tuple))
            if wp == query[0]:
                plt.text(wp.x - 0.3, wp.y - 0.3, 'START')
            if wp == query[1]:
                plt.text(wp.x + 0.3, wp.y - 0.3, 'END')

        plt.ion()
        plt.show()
        raw_input("Press Enter to close plot...")
        plt.close()
