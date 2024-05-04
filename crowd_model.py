import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
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
    x_error=0.1

    if(((LA.norm(p.x_desired-p.x_current))/LA.norm(p.x_desired))<=x_error):
        p.v_desired=np.array([0,0])
    t_ralxation_time=1
    e_desired=(p.x_desired-p.x_current)/(LA.norm(p.x_desired-p.x_current))
    f_drive=(p.v_desired*e_desired-p.v_current)/t_ralxation_time
    return f_drive
    

# Function to caluclate repulsive interactions with other pedestrians β
def calculate_f_ab(p1,p2):
    A_a1=0.5
    B_a1=0.4
    r=0.01
    lamda_a=2
    nab_normlized_vector=(p1.x_current-p2.x_current)/LA.norm(p1.x_current-p2.x_current)
    if(LA.norm(p1.v_current)==0):
        return 0
    #e_desired=(p1.x_desired-p1.x_current)/(LA.norm(p1.x_desired-p1.x_current))
    #cos_phi=-nab_normlized_vector*e_desired
    e_desired=(p1.x_current-p2.x_current)/(LA.norm(p1.x_current-p2.x_current))
    print(LA.norm(p1.v_current))
    cos_phi=np.dot(p1.v_current,e_desired)/(LA.norm(p1.v_current)*LA.norm(e_desired))

    d=LA.norm(p2.x_current-p1.x_current)
    re=math.exp((r-d)/B_a1)
    f_ab=A_a1*re*nab_normlized_vector*(lamda_a+(((1-lamda_a)*(1+cos_phi))/2))
    return f_ab
def calculate_f_ab_att(p1,p2):
    A_a1=0.01
    B_a1=0.3
    r=0.01
    lamda_a=2
    nab_normlized_vector=(p1.x_current-p2.x_current)/LA.norm(p1.x_current-p2.x_current)
    if(LA.norm(p1.v_current)==0):
        return 0
    #e_desired=(p1.x_desired-p1.x_current)/(LA.norm(p1.x_desired-p1.x_current))
    #cos_phi=-nab_normlized_vector*e_desired
    e_desired=(p1.x_current-p2.x_current)/(LA.norm(p1.x_current-p2.x_current))
    print(LA.norm(p1.v_current))
    cos_phi=np.dot(p1.v_current,e_desired)/(LA.norm(p1.v_current)*LA.norm(e_desired))

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
def pedestrian_update(p1,p2,p3,p4,p5,p6):
    p1.f_drive=calculate_f_drive(p1)
    p2.f_drive=calculate_f_drive(p2)
    p3.f_drive=calculate_f_drive(p3)
    p4.f_drive=calculate_f_drive(p4)
    p5.f_drive=calculate_f_drive(p5)
    p6.f_drive=calculate_f_drive(p6)

    p1.f_ab=calculate_f_ab(p1,p2)
    p1.f_ab=p1.f_ab+calculate_f_ab(p1,p4)
    p1.f_ab=p1.f_ab+calculate_f_ab(p1,p6)
    p1.f_ab=p1.f_ab-calculate_f_ab_att(p1,p3)
    p1.f_ab=p1.f_ab-calculate_f_ab_att(p1,p5)

    p2.f_ab=calculate_f_ab(p2,p3)
    p2.f_ab=p2.f_ab+calculate_f_ab(p2,p1)
    p2.f_ab=p2.f_ab+calculate_f_ab(p2,p5)
    p2.f_ab=p2.f_ab-calculate_f_ab_att(p2,p4)
    p2.f_ab=p2.f_ab-calculate_f_ab_att(p2,p6)

    p3.f_ab=calculate_f_ab(p3,p2)
    p3.f_ab=p3.f_ab+calculate_f_ab(p3,p4)
    p3.f_ab=p3.f_ab+calculate_f_ab(p3,p6)
    p3.f_ab=p3.f_ab-calculate_f_ab_att(p3,p1)
    p3.f_ab=p3.f_ab-calculate_f_ab_att(p3,p5)

    p4.f_ab=calculate_f_ab(p4,p3)
    p4.f_ab=p4.f_ab+calculate_f_ab(p4,p1)
    p4.f_ab=p4.f_ab+calculate_f_ab(p4,p5)
    p4.f_ab=p4.f_ab-calculate_f_ab_att(p4,p2)
    p4.f_ab=p4.f_ab-calculate_f_ab_att(p4,p6)

    p5.f_ab=calculate_f_ab(p5,p4)
    p5.f_ab=p5.f_ab+calculate_f_ab(p5,p2)
    p5.f_ab=p5.f_ab+calculate_f_ab(p5,p6)
    p5.f_ab=p5.f_ab-calculate_f_ab_att(p5,p2)
    p5.f_ab=p5.f_ab-calculate_f_ab_att(p5,p2)

    p6.f_ab=calculate_f_ab(p6,p3)
    p6.f_ab=p6.f_ab+calculate_f_ab(p6,p1)
    p6.f_ab=p6.f_ab+calculate_f_ab(p6,p5)
    p6.f_ab=p6.f_ab-calculate_f_ab_att(p6,p2)
    p6.f_ab=p6.f_ab-calculate_f_ab_att(p6,p2)

    p1.v_next=calculate_velocity(p1)
    p2.v_next=calculate_velocity(p2)
    p3.v_next=calculate_velocity(p3)
    p4.v_next=calculate_velocity(p4)
    p5.v_next=calculate_velocity(p5)
    p6.v_next=calculate_velocity(p6)

    p1.x_next=calculate_position(p1)
    p2.x_next=calculate_position(p2)
    p3.x_next=calculate_position(p3)
    p4.x_next=calculate_position(p4)
    p5.x_next=calculate_position(p5)
    p6.x_next=calculate_position(p6)

    return p1,p2,p3,p4,p5,p6

path1=np.array([5,0.01])
line_array=np.array([0,0])

def update(t):
    # obtain point coordinates 
    global p1,p2,p3,p4,p5,p6,first,path1
    if(first):
        first=False
        return point2,point1, p1_line, p2_line

    p1,p2,p3,p4,p5,p6=pedestrian_update(p1,p2,p3,p4,p5,p6)
    # set point's coordinates

    point1.set_data([p1.x_next[0]],[p1.x_next[1]])
    point2.set_data([p2.x_next[0]],[p2.x_next[1]])
    point3.set_data([p3.x_next[0]],[p3.x_next[1]])
    point4.set_data([p4.x_next[0]],[p4.x_next[1]])
    point5.set_data([p5.x_next[0]],[p5.x_next[1]])
    point6.set_data([p6.x_next[0]],[p6.x_next[1]])

    p1_line.set_data(np.append(p1_line.get_xdata(), [p1.x_next[0]]), np.append(p1_line.get_ydata(), [p1.x_next[1]]))
    p2_line.set_data(np.append(p2_line.get_xdata(), [p2.x_next[0]]), np.append(p2_line.get_ydata(), [p2.x_next[1]])) 
    p3_line.set_data(np.append(p3_line.get_xdata(), [p3.x_next[0]]), np.append(p3_line.get_ydata(), [p3.x_next[1]]))
    p4_line.set_data(np.append(p4_line.get_xdata(), [p4.x_next[0]]), np.append(p4_line.get_ydata(), [p4.x_next[1]]))
    p5_line.set_data(np.append(p5_line.get_xdata(), [p5.x_next[0]]), np.append(p5_line.get_ydata(), [p5.x_next[1]]))
    p6_line.set_data(np.append(p6_line.get_xdata(), [p6.x_next[0]]), np.append(p6_line.get_ydata(), [p6.x_next[1]]))

    p1.v_current=p1.v_next
    p2.v_current=p2.v_next
    p1.x_current=p1.x_next
    p2.x_current=p2.x_next
    p3.v_current=p3.v_next
    p3.x_current=p3.x_next
    p4.v_current=p4.v_next
    p4.x_current=p4.x_next
    p5.v_current=p5.v_next
    p5.x_current=p5.x_next
    p6.v_current=p6.v_next
    p6.x_current=p6.x_next

    return point2,point1,point3,point4,point5,point6,p1_line, p2_line,p3_line,p4_line,p5_line,p6_line


# create a figure with an axes
fig, ax = plt.subplots()
# set the axes limits
ax.axis([-6,6,-6,6])
# set equal aspect such that the circle is not shown as ellipse
ax.set_aspect("equal")
# create a point in the axes
point1, = ax.plot(-3,-4, marker="o",color='b')
point2, = ax.plot(5,0, marker="o",color='r')
point3, = ax.plot(-4,-3, marker="o",color='b')
point4, = ax.plot(5,1, marker="o",color='r')
point5, = ax.plot(-2,-5, marker="o",color='b')
point6, = ax.plot(5,-1, marker="o",color='r')

p1_line, = ax.plot([], [], linestyle="-", color="b")
p2_line, = ax.plot([], [], linestyle="-", color="r")
p3_line, = ax.plot([], [], linestyle="-", color="b")
p4_line, = ax.plot([], [], linestyle="-", color="r")
p5_line, = ax.plot([], [], linestyle="-", color="b")
p6_line, = ax.plot([], [], linestyle="-", color="r")


p1=Pedestrians(
               v_desired=np.array([0.1,0.1]),
               x_desired=np.array([3.5,4]),
               v_current=np.array([0,0]),
               x_current=np.array([-3,-4]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point1,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

p2=Pedestrians(

               v_desired=np.array([0.1,0.1]),
               x_desired=np.array([-4,0]),
               v_current=np.array([0,0]),
               x_current=np.array([5,0]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point2,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

p3=Pedestrians(

               v_desired=np.array([0.1,0.1]),
               x_desired=np.array([1,4]),
               v_current=np.array([0,0]),
               x_current=np.array([-4,-3]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point3,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

p4=Pedestrians(

               v_desired=np.array([0.1,0.1]),
               x_desired=np.array([-4,1]),
               v_current=np.array([0,0]),
               x_current=np.array([5,1]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point4,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

p5=Pedestrians(

               v_desired=np.array([0.1,0.1]),
               x_desired=np.array([2,4]),
               v_current=np.array([0,0]),
               x_current=np.array([-2,-5]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point4,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

p6=Pedestrians(

               v_desired=np.array([0.1,0.1]),
               x_desired=np.array([-4,-1]),
               v_current=np.array([0,0]),
               x_current=np.array([5,-1]),
               v_next=np.array([0,0]),
               x_next=np.array([0,0]),
               r_radius=3,
               graph=point4,
               f_drive=np.array([0,0]),
               f_ab=np.array([0,0]))

# Updating function, to be repeatedly called by the animation

first=True
# create animation with 10ms interval, which is repeated,
ani = FuncAnimation(fig, update, blit=True, repeat=True,
                    frames=300)
writer = PillowWriter(fps=100,
                                metadata=dict(artist='Me'),
                                bitrate=1800)
ani.save('C:/Users/alkha/Downloads/scatter.gif', writer=writer)

plt.show()
