#paraboloidic
import numpy as np
import matplotlib.pyplot as plt

start = np.array([1, 1])
goal = np.array([20, 20])

obstacles = {(4.5, 3) : 2, (3, 12) : 2, (15, 15) : 3}

ka = 10  
kr = 10000
q_star = 10000000000

def gradient_potential(x, y, goal, obstacles):
    grad_x = ka * (x - goal[0]) 
    grad_y = ka * (y - goal[1])  

    
    for centre, r in obstacles.items():
        dist_to_obs = np.sqrt((x - centre[0]) ** 2 + (y - centre[1]) ** 2)
        if dist_to_obs <= (r + q_star):
            grad_x += kr * (1/q_star - 1/dist_to_obs) * (x - centre[0]) / dist_to_obs**3
            grad_y += kr * (1/q_star - 1/dist_to_obs) * (y - centre[1]) / dist_to_obs**3

    return -grad_x, -grad_y


path = []
current_position = np.array([1, 1])
learning_rate = 0.1
for i in range(1000):
    path.append(current_position.copy())
    grad_x, grad_y = gradient_potential(current_position[0], current_position[1], goal, obstacles)
    
    current_position[0] += learning_rate * grad_x 
    current_position[1] += learning_rate * grad_y 

    if current_position [0] == goal[0] and current_position [1] == goal[1]:
        path.append(current_position.copy())
        break


path = np.array(path)
print(path)

plt.figure()
plt.xlim(0, 30)
plt.ylim(0, 30)

plt.plot(start[0], start[1], 'go')
plt.plot(goal[0], goal[1], 'ro')

for centre, radius in obstacles.items():
    circle = plt.Circle(centre, radius, color='b', alpha=0.3)
    plt.gca().add_patch(circle)

plt.plot(path[:, 0], path[:, 1])

plt.show()