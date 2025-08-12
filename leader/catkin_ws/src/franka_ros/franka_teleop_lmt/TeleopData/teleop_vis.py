#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


data_leader = np.genfromtxt('/home/rmil/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/leader_1.txt', delimiter='	', names=True)
data_follower = np.genfromtxt('/home/rmil/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/follower_1.txt', delimiter='	', names=True)

first_packet_follower = np.where(data_follower['first_packet']==1)[0][0]
first_packet_leader = np.where(data_leader['first_packet']==1)[0][0]

# Time alignment

fig1 = plt.figure(figsize=(20, 8)) 
plt.subplot(331)
plt.gca().set_title('Velocity X')
plt.plot(data_leader['time'] , data_leader['Vl_x'] , label='Vl_x')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , data_follower['Vf_x'][first_packet_follower:-1] , label='Vf_x')
plt.legend(prop={'size':13})


plt.subplot(332)
plt.gca().set_title('Velocity Y')
plt.plot(data_leader['time'] , data_leader['Vl_y'] , label='Vl_y')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , data_follower['Vf_y'][first_packet_follower:-1] , label='Vf_y')
plt.legend(prop={'size':13})

plt.subplot(333)
plt.gca().set_title('Velocity Z')
plt.plot(data_leader['time'] , data_leader['Vl_z'] , label='Vl_z')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , data_follower['Vf_z'][first_packet_follower:-1] , label='Vf_z')
plt.legend(prop={'size':13})


data_leader['Xl_x'] = data_leader['Xl_x'] - data_leader['Xl_x'][1] + data_follower['Xf_x'][1];
plt.subplot(334)
plt.gca().set_title('Position X')
plt.plot(data_leader['time'] , data_leader['Xl_x'] , label='Xl_x')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , data_follower['Xf_x'][first_packet_follower:-1] , label='Xf_x')
plt.legend(prop={'size':13})

data_leader['Xl_y'] = data_leader['Xl_y'] - data_leader['Xl_y'][1] + data_follower['Xf_y'][1];
plt.subplot(335)
plt.gca().set_title('Position Y')
plt.plot(data_leader['time'] , data_leader['Xl_y'] , label='Xl_y')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , data_follower['Xf_y'][first_packet_follower:-1] , label='Xf_y')
plt.legend(prop={'size':13})

data_leader['Xl_z'] = data_leader['Xl_z'] - data_leader['Xl_z'][1] + data_follower['Xf_z'][1];
plt.subplot(336)
plt.gca().set_title('Position Z')
plt.plot(data_leader['time'] , data_leader['Xl_z'] , label='Xl_z')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , data_follower['Xf_z'][first_packet_follower:-1] , label='Xf_z')
plt.legend(prop={'size':13})


plt.subplot(337)
plt.gca().set_title('Force X')
plt.plot(data_leader['time'] , -data_leader['Fl_x'] , label='Fl_x')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , -data_follower['Fc_x'][first_packet_follower:-1] , label='Fc_x')
plt.ylim([-15, 20])
plt.legend(prop={'size':13})
#plt.legend()

plt.subplot(338)
plt.gca().set_title('Force Y')
plt.plot(data_leader['time'] , -data_leader['Fl_y'] , label='Fl_y')
plt.plot(data_follower['time'][0:-first_packet_follower-1] , -data_follower['Fc_y'][first_packet_follower:-1] , label='Fc_y')
plt.ylim([-15, 20])
plt.legend(prop={'size':13})

plt.subplot(339)
plt.gca().set_title('Force Z')
plt.plot(data_leader['time'] , -data_leader['Fl_z'] , label='Fl_z')
plt.plot(data_follower['time'] [0:-first_packet_follower-1], -data_follower['Fc_z'][first_packet_follower:-1] , label='Fc_z')
plt.ylim([-15, 20])
plt.legend(prop={'size':13})


plt.show()
