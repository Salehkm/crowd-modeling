import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from numpy import linalg as LA


class Pedestrians:
 
    # init method or constructor
    def __init__(self,v_desired,x_desired,v_current,x_current,v_next,x_next,r_radius,graph,f_drive,f_ab):
        self.v_desired  =   v_desired
        self.x_desired  =   x_desired
        self.r_radius   =   r_radius
        self.v_current  =   v_current
        self.x_current  =   x_current
        self.v_next     =   v_next
        self.x_next     =   x_next
        self.f_drive    =   f_drive
        self.f_ab       =   f_ab
        self.graph      =   graph

# Function to caluclate driving force f_drive of α
def calculate_f_drive(p):
    t_ralxation_time=1
    e_desired=(p.x_desired-p.x_current)/(LA.norm(p.x_desired-p.x_current))
    f_drive=(p.v_desired*e_desired-p.v_current)/t_ralxation_time
    return f_drive
    

# Function to caluclate repulsive interactions with other pedestrians β
def calculate_f_ab(p1,p2):
    A_a1=4
    B_a1=0.5
    r=0.01
    lamda_a=2
    nab_normlized_vector=(p1.x_current-p2.x_current)/LA.norm(p1.x_current-p2.x_current)
    e_desired=(p1.x_desired-p1.x_current)/(LA.norm(p1.x_desired-p1.x_current))
    cos_phi=-nab_normlized_vector*e_desired
    d=LA.norm(p2.x_current-p1.x_current)
    re=math.exp((r-d)/B_a1)
    f_ab=A_a1*re*nab_normlized_vector*(lamda_a+(((1-lamda_a)*(1+cos_phi))/2))
    return f_ab


def calculate_position(p):
    x_next=p.x_current+p.v_current
    return x_next

def calculate_velocity(p):
    v_next=p.v_current+p.f_ab+p.f_drive
    return v_next

#Function to caluclate new state of Pedestrians
def pedestrian_update(p1,p2):
    p1.f_drive=calculate_f_drive(p1)
    p2.f_drive=calculate_f_drive(p2)
    p1.f_ab=calculate_f_ab(p1,p2)
    p2.f_ab=calculate_f_ab(p2,p1)
    p1.v_next=calculate_velocity(p1)
    p2.v_next=calculate_velocity(p2)
    p1.x_next=calculate_position(p1)
    p2.x_next=calculate_position(p2)
    return p1,p2

path1=np.array([5,0.01])
line_array=np.array([0,0])

def update(t):
    # obtain point coordinates 
    global p1,p2,first,path1
    if(first):
        first=False
        return point2,point1, p1_line, p2_line

    p1,p2=pedestrian_update(p1,p2)
    # set point's coordinates

    point1.set_data([p1.x_next[0]],[p1.x_next[1]])
    point2.set_data([p2.x_next[0]],[p2.x_next[1]])
    p1_line.set_data(np.append(p1_line.get_xdata(), [p1.x_next[0]]), np.append(p1_line.get_ydata(), [p1.x_next[1]]))
    p2_line.set_data(np.append(p2_line.get_xdata(), [p2.x_next[0]]), np.append(p2_line.get_ydata(), [p2.x_next[1]]))

    p1.v_current=p1.v_next
    p2.v_current=p2.v_next
    p1.x_current=p1.x_next
    p2.x_current=p2.x_next

    return point2,point1,p1_line, p2_line


# create a figure with an axes
fig, ax = plt.subplots()
# set the axes limits
ax.axis([-6,6,-6,6])
# set equal aspect such that the circle is not shown as ellipse
ax.set_aspect("equal")
# create a point in the axes
point1, = ax.plot(-5,0, marker="o")
point2, = ax.plot(5,0, marker="o")
p1_line, = ax.plot([], [], linestyle="-", color="b")
p2_line, = ax.plot([], [], linestyle="-", color="b")


p1=Pedestrians(
               v_desired=np.array([0.1,0]),
               x_desired=np.array([5,0]),
               v_current=np.array([0,0]),
               x_current=np.array([-5,-0.01]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point1,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

p2=Pedestrians(

               v_desired=np.array([0.1,0]),
               x_desired=np.array([-5,0]),
               v_current=np.array([0,0]),
               x_current=np.array([5,0.01]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point2,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

# Updating function, to be repeatedly called by the animation

first=True
# create animation with 10ms interval, which is repeated,
ani = FuncAnimation(fig, update, blit=True, repeat=True,
                    frames=200)

plt.show()




















# # Function to update the position of each person
# def update(frame, scat, data):
#     data[:, 0:2] += data[:, 2:4]  # Update positions
#     crowd= np.array([[3], 
#         [4]] )
#     scat.set_offsets(crowd)  # Set new positions for the scatter plot
    
#     return scat,





# # Create initial plot
# fig, ax = plt.subplots()
# ax.set_xlim(-5, 5)
# ax.set_ylim(-5, 5)
# scat = ax.scatter([], [])  # Scatter plot for people


# # Initial positions and velecoties 
# # P1(X=-4,Y=0,Vx=0.09,Vy=0) P2(X=4,Y=0,Vx=-0.09,Vy=0)
# crowd= np.column_stack([[-4,4], [0,0], [0.09,-0.09],[0.09, 0]])
# print(crowd[:, 2:4])
# # 
# # Update function for animation
# ani = FuncAnimation(fig, update, frames=200, fargs=(scat, crowd), blit=True)


# plt.show()


# #will be deleted

# # # Function to generate initial position and speed of each person
# # def gen_data():
# #     x = np.random.uniform(-5, 5, N)  # Random initial x positions
# #     y = np.random.uniform(-5, 5, N)  # Random initial y positions
# #     vx = np.random.uniform(-0.1, 0.1, N)  # Random initial x velocities
# #     vy = np.random.uniform(-0.1, 0.1, N)  # Random initial y velocities
# #     return np.column_stack([x, y, vx, vy])
# # Generate initial data
# # data = gen_data()
