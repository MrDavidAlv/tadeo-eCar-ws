
'''
Script para revisar el comportamiento de las velocidades y posiciones de los motores 
con respecto a las velocidades angulares y lineales deseadas.

@olmerg
'''


from numpy import copysign,sqrt,power
from math import pi
import numpy as np
import matplotlib.pyplot as plt



#velocidad lineal maxima, asumimos un maximo de 30km/h
max_vel=15
#velocidad angular de giro en grados/s
max_ang=100.0
lin_vel_=np.linspace(0,max_vel,50)
ang_vel_=np.linspace(-max_ang,max_ang,50)

lin_vel_des, ang_vel_des = np.meshgrid(lin_vel_/3.6, ang_vel_*pi/180, sparse=False)
print(lin_vel_des.shape)
print(ang_vel_des.shape)
radius_=0.1  # wheel radius (m) - matches model.sdf and fourws_kinematics_node
base_= 1.058 # wheelbase front-to-rear (m)
track_=0.55  # track width left-to-right (m)
# Distance between a wheel joint (from the midpoint of the wheel width) and the associated steering joint:
# We consider that the distance is the same for every wheel
wheel_steering_y_offset_=0

vel_steering_offset = (ang_vel_des*wheel_steering_y_offset_)/radius_
sign=copysign(1.0, lin_vel_des)
vel_left_front =  sign* sqrt((power(lin_vel_des - ang_vel_des * track_ / 2, 2) + power(base_ * ang_vel_des / 2.0, 2))) / radius_ - vel_steering_offset
vel_right_front = sign * sqrt((power(lin_vel_des + ang_vel_des * track_ / 2, 2) + power(base_ * ang_vel_des / 2.0, 2))) / radius_ + vel_steering_offset
vel_left_rear = sign * sqrt((power(lin_vel_des - ang_vel_des * track_ / 2, 2) + power(base_ * ang_vel_des / 2.0, 2))) / radius_ - vel_steering_offset
vel_right_rear = sign * sqrt((power(lin_vel_des + ang_vel_des * track_ / 2, 2) + power(base_ * ang_vel_des / 2.0, 2))) / radius_ + vel_steering_offset


resta=np.abs(2.0*lin_vel_des)-np.abs(ang_vel_des*track_)
front_left_steering = np.where(resta>0, np.arctan2(ang_vel_des*base_ , (2.0*lin_vel_des - ang_vel_des*track_)),copysign(pi/2, ang_vel_des))
front_right_steering = np.where(resta>0, np.arctan2(ang_vel_des*base_ ,(2.0*lin_vel_des + ang_vel_des*track_)),copysign(pi/2, ang_vel_des))


rear_left_steering = -front_left_steering
rear_right_steering = -front_right_steering

fig = plt.figure(figsize=(15,15))

titles=['rear-left','front-left','rear-right','front-right']
graphs=[vel_left_rear,vel_left_front,vel_right_rear,vel_right_front]
axes = fig.subplots(2,2)
for i in range(len(titles)):
    origin = 'lower'
    CS = axes[i//2,i%2].contour(lin_vel_,ang_vel_,graphs[i]*60.0/(2*pi),50, origin=origin)
    #CS2 = axes[i//2,i%2].contour(CS, levels=CS.levels[::2], origin=origin)
    axes[i//2,i%2].set_title(titles[i])
    axes[i//2,i%2].set_xlabel('linear velocity(km/h)')
    axes[i//2,i%2].set_ylabel('angular velocity(degree/s)')
    cbar = fig.colorbar(CS,ax=axes[i//2,i%2])
    cbar.ax.set_ylabel('angular speed wheel(rpm)')
fig = plt.figure(figsize=(15,15))
graphs=[rear_left_steering,front_left_steering,rear_right_steering,front_right_steering]
axes = fig.subplots(2,2)
for i in range(len(titles)):
    origin = 'lower'
    CS = axes[i//2,i%2].contourf(lin_vel_,ang_vel_,graphs[i]*180.0/(pi),50, origin=origin)
    #CS2 = axes[i//2,i%2].contour(CS, levels=CS.levels[::2], origin=origin)
    axes[i//2,i%2].set_title(titles[i])
    axes[i//2,i%2].set_xlabel('linear velocity(km/h)')
    axes[i//2,i%2].set_ylabel('angular velocity(degree/s)')
    cbar = fig.colorbar(CS,ax=axes[i//2,i%2])
    cbar.ax.set_ylabel('angular angle(degree)')

fig = plt.figure()
ax = plt.axes(projection='3d')
#ax.scatter3D(
ax.contour3D(lin_vel_,ang_vel_,vel_right_front*60.0/(2*pi),50)
ax.contour3D(lin_vel_,ang_vel_,vel_left_front*60.0/(2*pi),50)
ax.contour3D(lin_vel_,ang_vel_,vel_right_rear*60.0/(2*pi),50)
ax.contour3D(lin_vel_,ang_vel_,vel_left_rear*60.0/(2*pi),50)
plt.show()