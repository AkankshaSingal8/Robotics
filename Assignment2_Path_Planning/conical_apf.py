import numpy as np
import matplotlib.pyplot as plt
start = [1, 1]
goal = [20, 20]
obstacles = {(4.5, 3) : 2, (3, 12) : 2, (15, 15) : 3}

ka = 5
kr = 2
q_star = 10


def attractive_potential(x, y, goal):
    U_attr = ka * np.sqrt((x - goal[0])**2 + (y - goal[1])**2)
    return U_attr

def repulsive_potential(x, y, obstacles, q_star, kr):
    U_rep = np.zeros_like(x)
    for (x_obs, y_obs), r_obs in obstacles.items():
        d = np.sqrt((x - x_obs)**2 + (y - y_obs)**2)
        mask = d <= (r_obs + q_star)
        U_rep[mask] += 0.5 * kr * ((1 / d[mask] - 1 / q_star)**2)
    return U_rep

x_range = np.linspace(0, 30, 100)
y_range = np.linspace(0, 30, 100)

X, Y = np.meshgrid(x_range, y_range)

u_tot = attractive_potential(X, Y, goal) + repulsive_potential(X, Y, obstacles, q_star, kr)

fig = plt.figure()
axis = fig.add_subplot(111, projection='3d')
axis.plot_surface(X, Y, u_tot)
axis.set_title('Conical Surface Plot')

plt.show()
