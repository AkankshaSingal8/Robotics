import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean

start = np.array([1, 1])
goal = np.array([20, 20])

obstacles = {(4.5, 3) : 2, (3, 12) : 2, (15, 15) : 3}

class node:
    def __init__(self, point, parent = None):
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


def connect(s, g, obstacles):
    while True:
        flag = True
        for i in range(1000):
            rdm_point = np.array(s.point) + (np.array(g.point) - np.array(s.point)) * (i / 1000)
            for center, radius in obstacles.items():
                d = np.sqrt(np.sum((rdm_point - center) ** 2))
                if d < radius:
                    flag = False


        if flag:
            return node(g.point, s)
        s = growtree([s], g.point, delta, obstacles)
        if s is None:
            return None


start_node = node((1, 1))
goal_node = node((20, 20))


delta = 2.5

start_pts = [start_node]
goal_pts = [goal_node]


for i in range(10000):
    random_point = np.random.uniform(0, 30, 2)
    if i % 2 == 0:
       
        new_node = growtree(start_pts, random_point, delta, obstacles)
        if new_node != None:
            connection = connect(new_node, goal_pts[-1], obstacles)
            if connection != None:
                start_pts.append(connection)
                break
    else:
       
        new_node = growtree(goal_pts, random_point, delta, obstacles)
        if new_node != None:
            connection = connect(new_node, start_pts[-1], obstacles)
            if connection != None:
                start_pts.append(connection)
                break


fig, ax = plt.subplots()
plt.plot(start[0], start[1], 'go')
plt.plot(goal[0], goal[1], 'ro')

for node in start_pts:
    if node.parent != None:
        ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'g-')
for node in goal_pts:
    if node.parent != None:
        ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'b-')


for center, radius in obstacles.items():
    circle = plt.Circle(center, radius, color='b', fill = False)
    ax.add_artist(circle)

plt.xlim(0, 30)
plt.ylim(0, 30)
plt.title("Bidirectional RRT")

plt.show()
