import numpy as np
import queue
import math
import sys
import matplotlib.pyplot as plt

"""
Diameter, in meters.
Yes, we're approximating the bot as a circle.
"""
youbot_diameter = 0.6

"""
Hardcoded representation of the map.
"""
lines_map = [((0, 0), (4, 0)),
             ((4, 0), (4, 3)),
             ((0, 0), (0, 3)),
             ((0, 3), (4, 3)),
             ((3, 0), (3, 1)),
             ((2, 1), (2, 3))]

EMPTY = 0
BLOCKED = 1
MAYBE = 2 # Unused
# rows (y)
B_SIZE = 30
# cols (x)
B_SIZE2 = 40

ROOT_2 = math.sqrt(2)

"""
Determine if there is a line of sight between two given points.
This method is quite ugly and uses a conservative version of
bresenham's line drawing algorithm to trace the line more
efficiently, at the cost of some darned ugly code.
"""
def line_of_sight(a, b):
    if (a, b) in line_of_sight_cache:
        return line_of_sight_cache[(a, b)] == 1
    elif (b, a) in line_of_sight_cache:
        return line_of_sight_cache[(b, a)] == 1
    pair = (a, b)
    dx = b[0] - a[0]
    if dx < 0:
        tmp = a
        a = b
        b = tmp
        dx = -dx
    # a is always on the left side.
    dy = b[1] - a[1]
    if dx == 0:
        if dy > 0:
            for i in range(a[1], b[1] + 1):
                if board[a[0]][i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
            line_of_sight_cache[pair] = 1
            return True
        else:
            for i in range(b[1], a[1] + 1):
                if board[a[0]][i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
            line_of_sight_cache[pair] = 1
            return True
    if dy > 0:
        # going up.
        if dy == dx:
            if board[a[0]][a[1]] != EMPTY:
                line_of_sight_cache[pair] = -1
                return False
            for i in range(1, dx + 1):
                if board[a[0] + i][a[1] + i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                if board[a[0] + i - 1][a[1] + i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                if board[a[0] + i][a[1] + i - 1] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
            line_of_sight_cache[pair] = 1
            return True
        elif dy < dx:
            # slope shallow.
            y = a[1]
            error = 0;
            for i in range(a[0], b[0] + 1):
                if board[i][y] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                error += dy/dx
                if error >= 0.5:
                    error -= 1
                    y += 1
                    if y < B_SIZE:
                        if board[i][y] != EMPTY:
                            line_of_sight_cache[pair] = -1
                            return False
#                        if board[i - 1][y] != EMPTY:
#                            return False
            line_of_sight_cache[pair] = 1
            return True
        else:
            # slope steep.
            x = a[0]
            error = 0
            for i in range(a[1], b[1] + 1):
                if board[x][i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                error += dx/dy
                if error >= 0.5:
                    error -= 1
                    x += 1
                    if x < B_SIZE2:
                        if board[x][i] != EMPTY:
                            line_of_sight_cache[pair] = -1
                            return False
#                        if board[x][i - 1] != EMPTY:
#                            return False
            line_of_sight_cache[pair] = 1
            return True
    else:
        # going down.
        if -dy == dx:
            if board[a[0]][a[1]] != EMPTY:
                line_of_sight_cache[pair] = -1
                return False
            for i in range(1, dx + 1):
                if board[a[0] + i][a[1] - i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                if board[a[0] + i - 1][a[1] - i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                if board[a[0] + i][a[1] - i + 1] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
            line_of_sight_cache[pair] = 1
            return True
        dy = -dy
        if dy < dx:
            # slope shallow.
            y = a[1]
            error = 0;
            for i in range(a[0], b[0] + 1):
                if board[i][y] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                error += dy/dx
                if error >= 0.5:
                    error -= 1
                    y -= 1
                    if y >= 0:
                        if board[i][y] != EMPTY:
                            line_of_sight_cache[pair] = -1
                            return False
#                        if board[i - 1][y] != EMPTY:
#                            return False
            line_of_sight_cache[pair] = 1
            return True
        else:
            # slope steep.
            x = a[0]
            error = 0
            for i in range(a[1], b[1] - 1, -1):
                if board[x][i] != EMPTY:
                    line_of_sight_cache[pair] = -1
                    return False
                error += dx/dy
                if error >= 0.5:
                    error -= 1
                    x += 1
                    if x < B_SIZE2:
                        if board[x][i] != EMPTY:
                            line_of_sight_cache[pair] = -1
                            return False
            line_of_sight_cache[pair] = 1
            return True

"""
Run dijkstra starting from a given node, and reconstruct paths from each node in all_nodes.
"""
def djikstra(start, all_nodes):
    distances[(start, start)] = 0
    first_step[(start, start)] = start
    frontier = queue.PriorityQueue()
    prev = dict()

    # queue.put((priority, item))
    frontier.put((0, start, start))
    while not frontier.empty():
        dist, curr, from_node = frontier.get()
        neighbor_data = get_neighbors(curr, all_nodes)
        # print("visiting {}, from: {}".format(curr, from_node))
        for n in neighbor_data:
            neighbor = n[0]
            # print("Neighbor: {}".format(neighbor))
            neighbor_from = curr

            # Attempt to do line-of-sight shortcutting to create diagonal paths between pairs of points that have direct line of sight.
            if line_of_sight(from_node, neighbor):
                # print("shortcut {} to {}".format(from_node, neighbor))
                direct_dist = math.sqrt((from_node[0] - neighbor[0])**2 + (from_node[1] - neighbor[1])**2)
                neighbor_from = from_node
                new_dist = distances[(start, from_node)] + direct_dist
            else:
                new_dist = dist + n[1]

            old_dist = distances[(start, neighbor)]
            # print("New: {}, old: {}".format(new_dist, old_dist))
            if new_dist < old_dist:
                distances[(start, neighbor)] = new_dist
                prev[neighbor] = neighbor_from
                frontier.put((new_dist, neighbor, neighbor_from))

    # Reconstruct paths for every destination node.
    for node in all_nodes:
        if node == start:
            continue;
        head = node
        while prev[head] != start:
            head = prev[head]
        first_step[(start, node)] = head

"""
Get immediate neighbors of a node. Uses an 8-connected grid.
"""
def get_neighbors(node, valids):
    neighbors = []
    flags = 0
    if node[0] > 0 and (node[0] - 1, node[1]) in valids:
        flags |= 1
        neighbors.append(((node[0] - 1, node[1]), 1))
    if node[1] > 0 and (node[0], node[1] - 1) in valids:
        flags |= 2
        neighbors.append(((node[0], node[1] - 1), 1))
    if node[0] < B_SIZE2 - 1 and (node[0] + 1, node[1]) in valids:
        flags |= 4
        neighbors.append(((node[0] + 1, node[1]), 1))
    if node[1] < B_SIZE - 1 and (node[0], node[1] + 1) in valids:
        flags |= 8
        neighbors.append(((node[0], node[1] + 1), 1))
    if flags & 3 == 3 and (node[0] - 1, node[1] - 1) in valids:
        neighbors.append(((node[0] - 1, node[1] - 1), ROOT_2))
    if flags & 9 == 9 and (node[0] - 1, node[1] + 1) in valids:
        neighbors.append(((node[0] - 1, node[1] + 1), ROOT_2))
    if flags & 6 == 6 and (node[0] + 1, node[1] - 1) in valids:
        neighbors.append(((node[0] + 1, node[1] - 1), ROOT_2))
    if flags & 12 == 12 and (node[0] + 1, node[1] + 1) in valids:
        neighbors.append(((node[0] + 1, node[1] + 1), ROOT_2))
    return neighbors

# Initialize the data structures.
# This should really be moved elsewhere so we don't have globals and high startup costs, but eh.
distances = {((x, y), (a, b)): 999 for x in range(B_SIZE2) for y in range(B_SIZE) for a in range(B_SIZE2) for b in range(B_SIZE)}
first_step = {((x, y), (a, b)): None for x in range(B_SIZE2) for y in range(B_SIZE) for a in range(B_SIZE2) for b in range(B_SIZE)}
line_of_sight_cache = dict()
board = None

"""
Run everything needed to populate the pathing lookup data structures.
Loads them from dijkstra_save.pickl instead of that exists.

If it computed stuff, saves it to dijkstra_save.pickl.
"""
def setup_dijkstras(should_plot=True):
    
    global board
    board = np.ones((B_SIZE2, B_SIZE))

    def feasible(point):
        for a, b in lines_map:
            dist = line_point_distance_simple(a, b, point)
            # print(a, b, dist)
            if dist < youbot_diameter/2:
                return False
        return True

    points = []
    for x in range(B_SIZE2):
        for y in range(B_SIZE):
            point = (x/10, y/10)
            if feasible(point):
                board[x][y] = 0
                points.append((x, y))
    if should_plot:
        plt.scatter(*zip(*points))
        plt.show()
    if try_load():
        return
    
    graph_nodes = set()

    for x in range(B_SIZE2):
        for y in range(B_SIZE):
            tile = board[x][y]
            if tile == EMPTY:
                graph_nodes.add((x, y))

    progress_counter = 0
    for node in graph_nodes:
        djikstra(node, graph_nodes)
        progress_counter += 1
        if progress_counter % B_SIZE == 0:
            print("Finished {} tiles".format(progress_counter), file=sys.stderr)
    save()

def save():
    global first_step
    import pickle
    pickle.dump(first_step, open("dijkstra_save.pickl", "wb"))

def try_load():
    global first_step
    try:
        import pickle
        first_step = pickle.load(open("dijkstra_save.pickl", "rb"))
        return True
    except:
        return False

"""
Use the lookup table to get a heading that we should
point towards to get from "me" to "target".
"""
prev_heading = -1
def get_local_heading(me, target):
    global prev_heading
    if me == target:
        return prev_heading
    step1 = first_step[(me, target)]
    if step1 == None:
        return prev_heading
    prev_heading = angle(me, step1)
    return prev_heading

if __name__ == "__main__":
    from utils import *
    setup_dijkstras()
else:
    from scripts.utils import *