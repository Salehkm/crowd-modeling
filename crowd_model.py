import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



# Function to update the position of each person
def update(frame, scat, data):
    data[:, 0:2] += data[:, 2:4]  # Update positions
    scat.set_offsets(data[:, 0:2])  # Set new positions for the scatter plot
    return scat,

# Create initial plot
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
scat = ax.scatter([], [])  # Scatter plot for people


# Initial positions and velecoties 
# P1(X=-4,Y=0,Vx=0.09,Vy=0) P2(X=4,Y=0,Vx=-0.09,Vy=0)
crowd= np.column_stack([[-4,4], [0,0], [0.09,-0.09],[0, 0]])
# 
# Update function for animation
ani = FuncAnimation(fig, update, frames=200, fargs=(scat, crowd), blit=True)

plt.show()


#will be deleted

# # Function to generate initial position and speed of each person
# def gen_data():
#     x = np.random.uniform(-5, 5, N)  # Random initial x positions
#     y = np.random.uniform(-5, 5, N)  # Random initial y positions
#     vx = np.random.uniform(-0.1, 0.1, N)  # Random initial x velocities
#     vy = np.random.uniform(-0.1, 0.1, N)  # Random initial y velocities
#     return np.column_stack([x, y, vx, vy])
# Generate initial data
# data = gen_data()
