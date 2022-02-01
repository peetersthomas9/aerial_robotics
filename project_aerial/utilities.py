import numpy as np
from cflib.crazyflie.log import LogConfig
import matplotlib.pyplot as plt
from matplotlib import colors
import time
import math

class map_utile():
    def __init__(self, x_start, y_start):
        """
        initialise the map, the map is divided in squares of 1x1 cm
        0 : empty tile in the neutral zone
        1 : obstacle
        2 : start and landing zones
        3 : center of quadcopter
        4 : landing and starting pads
        """
        self.map = np.zeros((100,60)) # One square every 5 cm
        self.map[0:30,:] = 2.
        self.map[69:100,:] = 2.
        self.map[round(x_start/5),round(y_start/5)] = 3.

    def update_map_obstacle(self, pos, obstacle = False):
        """
        used to update the map and define the obstacle
        """

        # condition to be sure that the drone is put correctly on the map
        if pos[0] < 10:
            pos[0] = 10
        elif pos[0]>490:
            pos[0] = 490
        pos[0] =  np.ceil((pos[0])/5).astype(int) -1  # divide by 5 to fit on the map size (100,60)

        if pos[1] < 10:
            pos[1] = 10
        elif pos[1] >290:
            pos[1] = 290

        pos[1] =  np.ceil((pos[1])/5).astype(int) -1
        x1,y1 = pos[0], pos[1]
        
        self.map[x1,y1] = 1 # update the map 
        
    def update_pad_pos(self, pos, start = True):
        """
        Update pos of the pad on the map
        """
        if start == True :
            # condition to be sure that it is inside the map
            if pos[0]>485:
                pos[0]=485
            if pos[1]>285:
                pos[1]=pos[1]=285
            if pos[1]<15:
                pos[1]=15
                
            # create the pad as a 6 by 6 square on the map
            x1, x2 = np.ceil((pos[0]- 15)/5).astype(int)-1, np.ceil((pos[0]+ 15)/5).astype(int)-1
            y1, y2 = np.ceil((pos[1]- 15)/5).astype(int)-1, np.ceil((pos[1]+ 15)/5).astype(int)-1

            self.map[x1:x2,y1:y2] = 4

    def plot_map(self):
        # create discrete colormap
        cmap = colors.ListedColormap(['red', 'blue', 'green', 'white', 'black'])
        bounds = [0,1,2,3,4,5]
        norm = colors.BoundaryNorm(bounds, cmap.N)

        fig, ax = plt.subplots(figsize = (30,30))
        ax.imshow(self.map, cmap=cmap, norm=norm)

        # draw gridlines
        ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=0.1)
        ax.set_xticks(np.arange(0.5, 60.5, 1))
        ax.set_yticks(np.arange(0.5, 100.5, 1))

        plt.show()


class find_pad():

    def __init__(self):
        self.pad_bord1=None
        self.pad_bord2=[]

        self.pad_bord3=[]
        self.pad_bord4=[]    

        self.states_pad = {
        1 : "border1", 
        2 : "border2",
        3 : "border3",
        }
        return

    def find_pad_out(self,var_z_history,var_x_history,var_y_history):
        """
        Funtion to check if we are at the border of the pad when we the drone comes from outsie the pad
        """

        # take the max and min from the last 20 values in z
        min_z=min(var_z_history[-20:-1])
        max_z=max(var_z_history[-20:-1])
        find=False

        # check if the difference is above a treshold : if it is the case the border is found and we extract the coordinate of the 
        # point of the border by finding the index of the z with the min value (oscilation of z due to the change of height)
        if (max_z-min_z)>0.035 : 

            ind_z=var_z_history[-10:-1].index(min(var_z_history[-10:-1]))
            
            self.pad_bord1=[var_x_history[-10+ind_z],var_y_history[-10+ind_z]]
            
            find=True
        return find

    def find_pad_in(self,var_z_history,var_x_history,var_y_history,var=0):
        """
        Funtion to check if we are at the border of the pad when we the drone comes from outsie the pad
        """
         # compute the min and max value of the last 20 value obtain for z 
        min_z=min(var_z_history[-20:-1])
        max_z=max(var_z_history[-20:-1])
        find=False
        # check if the difference is above a treshold : if it is the case the border is found and we extract the coordinate of the 
        # point of the border by finding the index of the z with the max value (oscilation of z due to the change of height)
        if (max_z-min_z)>0.035 :
            ind_z=var_z_history[-10:-1].index(max(var_z_history[-10:-1]))
            # check with the variable which pad we need to upload
            if var==0:
                self.pad_bord2=[var_x_history[-20+ind_z],var_y_history[-20+ind_z]]
            if var==1:
                self.pad_bord3=[var_x_history[-20+ind_z],var_y_history[-20+ind_z]]
            if var==2:
                self.pad_bord4=[var_x_history[-20+ind_z],var_y_history[-20+ind_z]]
            find=True
        return find

class call_backs():
    def __init__(self, cf):
        self._cf = cf
        self._time_start=time.time()
        self._time_history =[]
        self.var_z_history = [] # value of our z estimate [m]
        self.var_x_history = [] # value  of our x estimate [m]
        self.var_y_history = [] # value  of our y estimate [m]
        self.var_yaw_history = []  # value  of our yaw estimate [deg]

        self.front = [] # value measured in our front sensor
        self.back = []  # value measured in our back sensor
        self.right = [] # value measured in our right sensor
        self.left = []  # value measured in our left sensor

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        print('Call backs configured')

        self.ratio=0.85

        return
    
    def _connected(self, link_id):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # The definition of the logconfig can be made before connecting
        
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=20) 
        self._lg_stab.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        self._lg_stab.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        self._lg_stab.add_variable('stateEstimate.z', 'float')  # estimated Z coordinate
        self._lg_stab.add_variable('stabilizer.yaw') #estimated Yaw 

        


        self._lg_range = LogConfig(name='ranger', period_in_ms=20) # We also chhanged the update_period in motion commander
        # add range value in the 4 directions
        self._lg_range.add_variable('range.front', 'float')
        self._lg_range.add_variable('range.back', 'float')
        self._lg_range.add_variable('range.left', 'float')
        self._lg_range.add_variable('range.right', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_range)
            # # This callback will receive the data
  
            self._lg_stab.data_received_cb.add_callback(self._get_pos) # add callback to get the position estimation of our drone
            self._lg_range.data_received_cb.add_callback(self._get_range) # add callback to get the ranger values

            # Start the logging
            self._lg_stab.start()
            self._lg_range.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        return
        
    def _disconnected(self, link_id):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_id)
        return

    def _connection_failed(self, link_id, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_id, msg))
        return

    def _connection_lost(sef, link_id, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_id, msg))

    def _get_pos(self, timestamp, data, logconf):
        """This callback is called when position data are received"""
        self._time_history.append(time.time()-self._time_start)
        self.var_z_history.append(data['stateEstimate.z'])
        self.var_x_history.append(data['stateEstimate.x']*self.ratio)
        self.var_y_history.append(data['stateEstimate.y'])
        self.var_yaw_history.append(data['stabilizer.yaw'])

    def _get_range(self, timestamp, data, logconf):
        """This callback is called when range datas are received"""

        self.front.append(data['range.front'])
        self.back.append(data['range.back'])
        self.right.append(data['range.right'])
        self.left.append(data['range.left'])
    
    def plot_log(self):
        """ function to plot the values of z"""
        plt.plot(self.var_z_history)
        plt.show()
