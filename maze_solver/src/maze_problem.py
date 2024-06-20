from time import sleep
from maze import Maze

class MazeProblem:

    def __init__(self, maze, goal_locations):
        self.maze = maze
        self.goal_locations = goal_locations
        states = list(maze.robotloc)
        states.insert(0, 0)
        self.start_state = tuple(states)


    def __str__(self):
        string =  "Mazeworld problem: "
        return string

    def animate_path(self, path):
        # reset the robot locations in the maze
        self.maze.robotloc = tuple(self.start_state[1:])

        for state in path:
            print(str(self))
            self.maze.robotloc = tuple(state[1:])
            sleep(1)

            print(str(self.maze))


    def get_successors(self, state):
        successors = []
        for i in range(1, len(state), 2):
            options = [(-1, 0), (0, 1), (1, 0), (0, -1)]
            for option in options:
                new_state = (state[i] + option[0], state[i + 1] + option[1])
                if self.legality_check(new_state):
                    successors.append(state[0:i] + new_state + state[i+2:])

        return successors


    def legality_check(self, state):
        return self.maze.is_floor(state[0], state[1]) and not self.maze.has_robot(state[0], state[1])


    def goal_test(self, state):
        if state[1:] == self.goal_locations:
            return True

        return False

    def manhattan_heuristic(self, state):
        distance = 0
        for i in range(len(self.goal_locations)):
            distance += abs(self.start_state[i + 1] - self.goal_locations[i])

        return distance