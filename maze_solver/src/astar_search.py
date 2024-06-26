from search_solution import SearchSolution
from heapq import heappush, heappop

class AstarNode:
    # each search node except the root has a parent node
    # and all search nodes wrap a state object

    def __init__(self, state, heuristic, parent=None, transition_cost=0):
        self.state = state
        self.heuristic = heuristic
        self.parent = parent
        self.transition_cost = transition_cost

    def priority(self):
        return (self.transition_cost + self.heuristic, self.state[0])
        

    # comparison operator,
    # needed for heappush and heappop to work with AstarNodes:
    def __lt__(self, other):
        return self.priority() < other.priority()


# take the current node, and follow its parents back
#  as far as possible. Grab the states from the nodes,
#  and reverse the resulting list of states.
def backchain(node):
    result = []
    current = node
    while current:
        result.append(current.state)
        current = current.parent

    result.reverse()
    return result


def astar_search(search_problem, heuristic_fn):
    start_node = AstarNode(search_problem.start_state, heuristic_fn(search_problem.start_state))
    pqueue = []
    heappush(pqueue, start_node)

    solution = SearchSolution(search_problem, "Astar with heuristic " + heuristic_fn.__name__)

    visited_cost = {}
    visited_cost[tuple(start_node.state)] = 0

    while pqueue:
        current_node = heappop(pqueue)

        if search_problem.goal_test(current_node.state):
            solution.path = backchain(current_node)
            solution.cost = visited_cost[tuple(current_node.state)]
            return solution

        search_problem.maze.robotloc = current_node.state[1:]

        for child in search_problem.get_successors(current_node.state):
            if tuple(child) not in visited_cost:
                visited_cost[tuple(child)] = current_node.transition_cost + 1
                solution.nodes_visited += 1
                heappush(pqueue, AstarNode(child, heuristic_fn(child), current_node, current_node.transition_cost + 1))

            elif visited_cost[tuple(child)] > current_node.transition_cost + 1:
                visited_cost[tuple(child)] = current_node.transition_cost + 1

    return solution