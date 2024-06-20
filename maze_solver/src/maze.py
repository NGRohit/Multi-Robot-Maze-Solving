import numpy as np

class Maze:

    def __init__(self, arr, robotloc=None):

        self.robotloc = robotloc
        print(self.robotloc[0])
        self.map = np.array(arr)

        self.width = self.map.shape[0]
        self.height = self.map.shape[1]

        # Keep safe
        newly_added = []
        for i in range(self.width):
            for j in range(self.height):
                if self.map[i][j] == 1 and (i, j) not in newly_added:
                    for a in range(i - 3, i + 3):
                        for b in range(j - 3, j + 3):
                            if 0 <= a < self.width and 0 <= b < self.height and self.map[a][b] == 0:
                                self.map[a][b] = 1
                                newly_added.append((a, b))

    def index(self, x, y):
        return (self.height - y - 1) * self.width + x


    # returns True if the location is a floor
    def is_floor(self, x, y):
        if x < 0 or x >= self.width:
            return False
        if y < 0 or y >= self.height:
            return False

        return self.map[y][x] == 0


    def has_robot(self, x, y):
        if x < 0 or x >= self.width:
            return False
        if y < 0 or y >= self.height:
            return False

        for i in range(0, len(self.robotloc), 2):
            rx = self.robotloc[i]
            ry = self.robotloc[i + 1]
            if rx == x and ry == y:
                return True

        return False
