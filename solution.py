#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems


# SOKOBAN HEURISTICS
def heur_alternate(state):
    """A better heuristic"""
    # INPUT: a Sokoban state
    # OUTPUT: a numeric value serving as an estimate of the distance of the state to the goal.

    # Return a high heuristic value if the state is unsolvable
    if not is_solvable(state):
        return 5000

    sum_dist = 0
    available_storage = list(state.storage)

    # Loop through each box to calculate distances and check for deadlocks.
    for box in state.boxes:
        # Skip boxes already in storage.
        if box in state.storage:
            continue

        # Check for various types of deadlocks.
        if corner_deadlock(state, box) or adjacent_deadlock(state, box) or edge_deadlock(state, box):
            return 5000

        # Calculate the minimum distance from robots to this box.
        min_dist_rtb = min(abs(robot[0] - box[0]) + abs(robot[1] - box[1]) for robot in state.robots)

        # Calculate the minimum distance from this box to its closest storage.
        min_dist_bts, closest_storage = min(
            (abs(box[0] - storage[0]) + abs(box[1] - storage[1]), storage) for storage in state.storage
        )

        # Remove the closest storage from available storage if applicable.
        if closest_storage in available_storage:
            available_storage.remove(closest_storage)

        # Calculate the number of obstacles around the box.
        objects_around = [
            (box[0] + 1, box[1]), (box[0] + 1, box[1] + 1), (box[0] - 1, box[1] + 1),
            (box[0] + 1, box[1] - 1), (box[0] - 1, box[1] - 1), (box[0] - 1, box[1]),
            (box[0], box[1] - 1), (box[0], box[1] + 1)
        ]
        sum_dist += min_dist_bts + min_dist_rtb + len(set(state.obstacles) & set(objects_around))

    return sum_dist



def is_solvable(state):
    """
    Check three conditions at the very beginning if the Sokoban state is solvable.
    """
    # Checks whether there is a robot or not.
    if len(state.robots) == 0:
        return False

    # Checks whether the number of storage is large than or equal to the number of boxes.
    if len(state.boxes) > len(state.storage):
        return False

    # Checks whether the storage can be accessed or not.
    for storage in state.storage:
        if (len(state.boxes) < len(state.storage)) and storage_not_accessible(storage, state):
            return False

    return True


def storage_not_accessible(storage, state):
    """
    Checks if the storage location is accessible.
    A storage is considered to be inaccessible if it is surrounded by walls or obstacles
    such that a box cannot be moved into it.
    """
    # Check adjacent sides
    up_block = (storage[1] == 0) or ((storage[0], storage[1] + 1) in state.obstacles)
    down_block = (storage[1] == state.height - 1) or ((storage[0], storage[1] - 1) in state.obstacles)
    left_block = (storage[0] == 0) or ((storage[0] - 1, storage[1]) in state.obstacles)
    right_block = (storage[0] == state.width - 1) or ((storage[0] + 1, storage[1]) in state.obstacles)

    # A storage is accessible if at least one adjacent side is not blocked
    return up_block and down_block and left_block and right_block


def corner_deadlock(state, box):
    """Checks if box is corner_deadlock, but not at the goal state"""
    # Check if the box is blocked from above.
    up_block = (box[1] == 0) or ((box[0], box[1] + 1) in state.obstacles)

    # Check if the box is blocked from below.
    down_block = (box[1] == state.height - 1) or ((box[0], box[1] - 1) in state.obstacles)

    # Check if the box is blocked from left.
    left_block = (box[0] == 0) or ((box[0] - 1, box[1]) in state.obstacles)

    # Check if the box is blocked from right.
    right_block = (box[0] == state.width - 1) or ((box[0] + 1, box[1]) in state.obstacles)

    # The box is a deadlock if it's in a corner, that is, blocked on two adjacent sides,
    # and not in its storage location.
    return (up_block or down_block) and (left_block or right_block) and box not in state.storage


def edge_deadlock(state, box):
    """Checks if there is a deadlock due to map walls on a box and all possible storage points"""

    # Check if the box is on the vertical or horizontal edge of the map.
    on_vertical_edge = (box[0] == 0) or (box[0] == state.width - 1)
    on_horizontal_edge = (box[1] == 0) or (box[1] == state.height - 1)

    # If the box is on an edge, check if all storage locations are inaccessible from that edge.
    if on_vertical_edge or on_horizontal_edge:
        for storage in state.storage:
            if on_vertical_edge and (storage[0] == box[0]):
                return False
            if on_horizontal_edge and (storage[1] == box[1]):
                return False
        return True
    return False


def adjacent_deadlock(state, box):
    """Check if a box is adjacent to another box and against a wall or obstacle. """
    x, y = box
    left = (x - 1, y)
    right = (x + 1, y)
    up = (x, y - 1)
    down = (x, y + 1)

    left_wall = x == 0
    right_wall = x == state.width - 1
    up_wall = y == 0
    down_wall = y == state.height - 1

    left_blocked = left in state.boxes
    right_blocked = right in state.boxes
    up_blocked = up in state.boxes
    down_blocked = down in state.boxes

    if (left_wall or right_wall) and (up_blocked or down_blocked):
        return True
    if (up_wall or down_wall) and (left_blocked or right_blocked):
        return True
    return False


def heur_zero(state):
    """Zero Heuristic can be used to make A* search perform uniform cost search"""
    return 0


def heur_manhattan_distance(state):
    """admissible sokoban puzzle heuristic: manhattan distance"""
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    total_dist = 0
    for box in state.boxes:
        min_dist = float('inf')
        for storage in state.storage:
            dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if dist < min_dist:
                min_dist = dist
        total_dist += min_dist
    return total_dist


def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    # f(n) = g(n) + weight * h(n)
    g_val = sN.gval
    h_val = sN.hval
    f_val = g_val + weight * h_val

    return f_val


# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    """Provides an implementation of weighted a-star, as described in the HW1 handout"""
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    se = SearchEngine(strategy='custom', cc_level='full')

    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn,
                   fval_function=wrapped_fval_function)
    return se.search(timebound)


def iterative_astar(initial_state, heur_fn, weight=1,
                    timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    """Provides an implementation of realtime a-star, as described in the HW1 handout"""
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    current_weight = weight
    best_state = None
    best_cost = float('inf')
    end_time = os.times()[0] + timebound
    search_stats = None

    while current_weight >= 1 and os.times()[0] < end_time:
        # Update time remaining for the search
        remaining_time = end_time - os.times()[0]
        if remaining_time <= 0:
            break

        # Run weighted A* with the current weight
        state, search_stats = weighted_astar(initial_state, heur_fn, current_weight, remaining_time)
        if state and state.gval < best_cost:
            best_state, best_cost = state, state.gval

        # Decrease the weight for the next iteration
        current_weight -= 0.1  # adjust this decrement as needed

    return best_state, search_stats


def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    """Provides an implementation of anytime greedy best-first search, as described in the HW1 handout"""
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    start_time = os.times()[0]

    best_solution = None
    best_cost = float('inf')
    stats = None

    se = SearchEngine('best_first', 'full')

    while os.times()[0] - start_time < timebound:
        remaining_time = timebound - (os.times()[0] - start_time)

        se.init_search(initial_state, sokoban_goal_state, heur_fn)

        solution, current_stats = se.search(remaining_time)
        if solution and solution.gval < best_cost:
            best_solution, best_cost = solution, solution.gval
            stats = current_stats

        if best_solution:
            initial_state = best_solution

    return best_solution, stats
