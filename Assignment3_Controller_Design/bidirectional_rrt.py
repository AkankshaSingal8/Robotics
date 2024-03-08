import numpy as np
import matplotlib.pyplot as plt
import csv

class node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent

def growtree(nodes, point, delta, obstacles):
    nearest = None
    min_distance = 10000000

    for n in nodes:
        distance = np.sqrt(np.sum((n.point - point) ** 2))
        if distance < min_distance:
            min_distance = distance
            nearest = n

    dir = np.array(point) - np.array(nearest.point)
    dist = np.sqrt(np.sum((point - nearest.point) ** 2))
    new_point = nearest.point + (dir / dist) * min(delta, dist)

    flag = True
    for i in range(1000):
        rdm_point = np.array(nearest.point) + (np.array(new_point) - np.array(nearest.point)) * (i / 1000)
        for center, radius in obstacles.items():
            d = np.sqrt(np.sum((rdm_point - center) ** 2))
            if d < (radius + 0.1):
                flag = False

    if flag:
        new_node = node(new_point, nearest)
        nodes.append(new_node)
        return new_node
    return None

def connect(s, g, obstacles, delta):
    while True:
        flag = True
        for i in range(1000):
            rdm_point = np.array(s.point) + (np.array(g.point) - np.array(s.point)) * (i / 1000)
            for center, radius in obstacles.items():
                d = np.sqrt(np.sum((rdm_point - center) ** 2))
                if d < radius:
                    flag = False

        if flag:
            return node(g.point, s), s
        s = growtree([s], g.point, delta, obstacles)
        if s is None:
            return None, None


start_node = node(np.array([1, 1]))
goal_node = node(np.array([20, 20]))

delta = 2.5
obstacles = {(4.5, 3): 2, (3, 12): 2, (15, 15): 3}

start_pts = [start_node]
goal_pts = [goal_node]

pt_of_connection = None
parent_of_connection = None

for i in range(10000):
    random_point = np.random.uniform(0, 30, 2)
    if i % 2 == 0:
        new_node = growtree(start_pts, random_point, delta, obstacles)
        if new_node is not None:
            connection, parent = connect(new_node, goal_pts[-1], obstacles, delta)
            if connection is not None:
                pt_of_connection = connection
                parent_of_connection = parent
                start_pts.append(connection)
                break
    else:
        new_node = growtree(goal_pts, random_point, delta, obstacles)
        if new_node is not None:
            connection, parent = connect(new_node, start_pts[-1], obstacles, delta)
            if connection is not None:
                pt_of_connection = connection
                parent_of_connection = parent
                goal_pts.append(connection)
                break

def extract_path(points_arr, path):
    
    current = points_arr[-1]
    while current is not None:
        tmp = current
        path.append(tmp.point)
        current = tmp.parent
        
    return path


# for node in start_pts[1:]:
#     print(node.point, node.parent.point)

# print('\nGoal points:')
# for node in goal_pts[1:]:
#     print(node.point, node.parent.point)

goal_to_intersection = []
goal_to_intersection = extract_path(goal_pts, goal_to_intersection)
# print(goal_to_intersection)

start_to_intersection = []
start_to_intersection = extract_path(start_pts, start_to_intersection)
# print(start_to_intersection)
final_path = start_to_intersection[::-1] + goal_to_intersection[1:]
# print(final_path)

with open('path2.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['x', 'y'])
    for point in final_path:
        writer.writerow(point)


fig, ax = plt.subplots()
plt.plot(start_node.point[0], start_node.point[1], 'go')
plt.plot(goal_node.point[0], goal_node.point[1], 'ro')

for node in start_pts:
    if node.parent is not None:
        ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'g-')
for node in goal_pts:
    if node.parent is not None:
        ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'b-')

for center, radius in obstacles.items():
    circle = plt.Circle(center, radius, color='b', fill=False)
    ax.add_artist(circle)

for i in range(len(final_path) - 1):
    ax.plot([final_path[i][0], final_path[i+1][0]], [final_path[i][1], final_path[i+1][1]], 'r-')

plt.xlim(0, 30)
plt.ylim(0, 30)
plt.title("Bidirectional RRT")
plt.show()
