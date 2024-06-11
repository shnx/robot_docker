#!/usr/bin/env python3
#mohammad.shannak@students.uni-freiburg.de
import rospy
from nav_msgs.msg import OccupancyGrid,MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Point,PointStamped
import random
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from Robot import Robot
from utils import prob_to_log_odds
from utils import log_odds_to_prob
from utils import linear_mapping_of_values
from scipy.sparse import lil_matrix
import scipy.sparse
import scipy.sparse.linalg
from scipy.sparse import csr_matrix,linalg
class OccupancyGridMap:
    def __init__(self, name='ogm',resolution=0.2, width=8, height=8, center_x=0, center_y=0, use_subscriber=False,use_sub_topic=None,ekf=False):
        #rospy.init_node('occupancy_grid_map_node')

        self.name=name
        self.resolution = resolution
        self.width = width
        self.height = height
        self.center_x = center_x
        self.center_y = center_y
        self.use_subscriber = use_subscriber
        self.img=None
        self.desired_width=int(width/resolution)
        self.original_shape=self.desired_width
        
        PRIOR_PROB = 0.5
        OCC_PROB   = 0.7
        FREE_PROB  = 0.3
        self.inv_map= prob_to_log_odds(PRIOR_PROB) * np.ones( (self.desired_width, self.desired_width))
        if(ekf):
            
            #self.mu = (cellnum,1))
            self.mu=np.zeros( (self.desired_width, self.desired_width))
            self.empty_numpy_shaped=self.mu
            self.mu_flattened = self.mu.reshape(-1, 1)
            self.H_squared=np.zeros( (self.desired_width, self.desired_width))
            self.derivative=np.zeros( (self.desired_width, self.desired_width))
            self.H=self.H_squared.reshape(-1, 1)
            self.Z=self.mu_flattened
            self.K=self.mu_flattened
            self.Sigma=np.identity(self.desired_width * self.desired_width)
            self.Sigma=5*self.Sigma
            #5*np.identity(self.desired_width*self.desired_width)
            self.free_perceived_prob=np.zeros((self.desired_width, self.desired_width))
            self.p_false_alarm=self.empty_numpy_shaped
            self.p_true_detection=self.empty_numpy_shaped
            self.mu_prob=self.mu_flattened
            


        
        if not use_subscriber:
            # Case 1: Create its own message
            
            self.pub = rospy.Publisher(name, OccupancyGrid, queue_size=1)
            self.sub = None
            self.setup_occupancy_grid_msg()
            self.img = np.full((self.occupancy_grid.info.height, self.occupancy_grid.info.width), 0.5)
        else:
            # Case 2: Set up a subscriber
            self.pub = rospy.Publisher(name, OccupancyGrid, queue_size=1)
            self.sub = rospy.Subscriber(use_sub_topic, OccupancyGrid, self.subscriber_callback)
            self.occupancy_grid = None
            self.img=None

    def subscriber_callback(self, msg):
        # Callback function for subscriber
        self.occupancy_grid = msg
        self.occupancy_grid.info.resolution=round(self.occupancy_grid.info.resolution,3)
        self.resolution=round(self.occupancy_grid.info.resolution,3)
        self.center_x_index=int((self.center_x - self.occupancy_grid.info.origin.position.x) / self.resolution)
        self.center_y_index=int((self.center_y - self.occupancy_grid.info.origin.position.y) / self.resolution)
        #self.img = np.full((self.occupancy_grid.info.height, self.occupancy_grid.info.width), 0.5)
        
        #self.prepare_img_from_state()
        
    def setup_occupancy_grid_msg(self):
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header = Header()
        self.occupancy_grid.header.frame_id = "map"
        self.occupancy_grid.info = MapMetaData()
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.width = int(self.width / self.resolution)
        self.occupancy_grid.info.height = int(self.height / self.resolution)
        self.occupancy_grid.info.origin.position.x = self.center_x - 0.5 * self.width
        self.occupancy_grid.info.origin.position.y = self.center_y - 0.5 * self.height
        self.occupancy_grid.data = [-1.0] * (self.occupancy_grid.info.width * self.occupancy_grid.info.height)
        self.center_x_index=int((self.center_x - self.occupancy_grid.info.origin.position.x) / self.resolution)
        self.center_y_index=int((self.center_y - self.occupancy_grid.info.origin.position.y) / self.resolution)

    def publish_occupancy_grid(self):
        # Update the timestamp before publishing
        self.occupancy_grid.header.stamp = rospy.Time.now()
        reshaped_img = self.img.reshape(-1, 1)

        self.occupancy_grid.data = reshaped_img.flatten().astype(int).tolist()
    
        # Publish the occupancy grid map
        self.pub.publish(self.occupancy_grid)
    
    def prepare_img(self):
    # Calculate limits for the region of interest
        size=int(np.sqrt(len(self.occupancy_grid.data)))
        self.img = np.full((size, size), 0.5)
        print(" len of data",len(self.occupancy_grid.data))
        
        print("Shape of image from subscribed gt occupancy grid ",self.name," shape:",self.img.shape)

        #start_time = time.time()
        
        self.img=np.array(self.occupancy_grid.data)
        self.img=self.img.reshape((self.occupancy_grid.info.height, self.occupancy_grid.info.width))
        self.img = np.where(self.img == -1.0, 0.5, self.img)#0.5 instead of none
        self.img = np.where(self.img == 100, 1, self.img)
        self.img=np.clip(self.img,0,1)
        #end_time = time.time()

        
        if(self.occupancy_grid.info.width>self.desired_width):
           self.small_img(self.desired_width)

    def small_img(self,desired_width=None):
        
        self.limit_down=int((self.occupancy_grid.info.width/2)-self.desired_width/2) 
        self.limit_up=int(self.limit_down + self.desired_width)
        
        self.img_small = np.full((int(desired_width), int(desired_width)), 0.5)
        
 
       
        self.img_small = self.img[self.limit_down:self.limit_up, self.limit_down:self.limit_up]
        print("Small image was prepared for",self.name,"use it as ",self.name,".img_small", "shape is",self.img_small.shape)
    
    def state_to_prob(self,state):
    # calculate occupancy probability using the cell state
        return 1/(1+np.exp(-state))

    def KalmanGain(self, H, z):
        #returns the KalmanGain given the measurement likelihood  as well as the Jacobian
        #self.Sigma.dot(np.transpose(H)).dot(np.linalg.inv(H.dot(self.Sigma).dot(np.transpose(H))+((z*(1-z)))))
        return self.Sigma.dot(np.transpose(H))#.dot(np.linalg.inv(H.dot(self.Sigma).dot(np.transpose(H))+((z*(1-z)))))

    
    def mderive(self, x):
        #the occupancy probability derived after x
        return math.exp(x)/((math.exp(x)+1)*(math.exp(x)+1))
    def polar_to_cartesian(self,x1, y1, r, theta):
        x2 = x1 + r * np.cos(theta)
        y2 = y1 + r * np.sin(theta)
        return x2, y2        
            

    def odom_coords_to_2d_array(self, x, y):
        """
        Transform a coord in the odom frame to coords in a 2D array.
        
        E.g.
        resolution  = 0.1
        map_size_ x = 101
        map_size_ y = 101
        (0.8, 1.1) -> 0.8 / 0.1 + 50, 1.1*100 + 50
                   -> (130, 160)
        
        """
        
        ix = int(x / self.occupancy_grid.info.resolution) + self.occupancy_grid.info.width  // 2
        iy = int(y / self.occupancy_grid.info.resolution) + self.occupancy_grid.info.height // 2
        
        return (ix, iy)
    def I_J(self, cone_x_y, d,angle):
        #PointStamped(point=Point())
        """
        Get the grid cells that belong to the line between the origin and target.
        The returned points' coordinates are int-indexes of the map 2D array.
        
        Based on the Bresenham's line algorithm, pag 13:
        http://members.chello.at/~easyfilter/Bresenham.pdf 
        """
        
        ix_iy_I = []
        ix_iy_J = []


        point_x,point_y=self.polar_to_cartesian(cone_x_y.x,cone_x_y.y,d,angle)#self.angle+np.deg2rad(-8+i)+robot.get_yaw_angle()
        x0 = int(cone_x_y.x/self.occupancy_grid.info.resolution)+self.occupancy_grid.info.width  // 2
        y0 = int(cone_x_y.y/self.occupancy_grid.info.resolution)+self.occupancy_grid.info.width  // 2
        x1 = int(point_x/self.occupancy_grid.info.resolution)+self.occupancy_grid.info.width  // 2
        y1 = int(point_y/self.occupancy_grid.info.resolution)+self.occupancy_grid.info.width  // 2
        
        
        dx = np.abs(x1 - x0)
        sx = 1 if (x0 < x1) else -1
        
        dy = -np.abs(y1 - y0)
        sy = 1 if (y0 < y1) else -1

        err = dx + dy

        #perceptual_range = []
        
        while True:
            if(abs(int(x0))>self.desired_width or abs(int(y0))>self.desired_width):
                print("over")
                break
            #perceptual_range.append((x0, y0))
            e2 = 2 * err

            if (e2 >= dy):
                if (x0 == x1): 
                    #print("reached x")
                    

                    break
                err += dy
                x0 += sx
            
            if (e2 <= dx):
                if (y0 == y1): 
                    #print("reached y")
                    

                    break
                err += dx
                y0 += sy
            ix_iy_I.append((x0,y0))
            
        ix_iy_J.append((x0,y0)) 

        return ix_iy_I,ix_iy_J
            
                
                
     
            
        
            

            
        """l_prev = self.get_map_val(ix, iy)
            if l_prev is None: continue
            l = l_prev + prob_to_log_odds(p) - prob_to_log_odds(PRIOR_PROB)
            self.mark_map(ix, iy, value=l)"""

    def plot_occupancy_grid(self):
        """
        Plot the preprocessed occupancy grid map using matplotlib.pyplot.
        """
        extent = [
            self.occupancy_grid.info.origin.position.x,
            self.occupancy_grid.info.origin.position.x + self.width,
            self.occupancy_grid.info.origin.position.y,
            self.occupancy_grid.info.origin.position.y + self.height
        ]
        plt.grid(True, linestyle='--', color='black', alpha=0.5, which='both', linewidth=0.5)
        plt.imshow(self.img, cmap='gray_r', origin='lower', extent=extent, vmin=0, vmax=1)  # 'gray_r' for reversed grayscale
        plt.title('Occupancy Grid Map')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.colorbar(label='Occupancy Probability')
        x_ticks = np.arange(self.occupancy_grid.info.origin.position.x, self.occupancy_grid.info.origin.position.x + self.width, 1.0)
        y_ticks = np.arange(self.occupancy_grid.info.origin.position.y, self.occupancy_grid.info.origin.position.y + self.height, 1.0)
        plt.xticks(x_ticks)
        plt.yticks(y_ticks)

        plt.show()
    def get_col_row_index_from_xy(self, x, y):
        """
        Retrieve the 1D index from x, y coordinates.

        :param x: X-coordinate in meters.
        :param y: Y-coordinate in meters.
        :return: 1D index.
        """
        col = int((x - self.occupancy_grid.info.origin.position.x) / self.resolution)
        row = int((y - self.occupancy_grid.info.origin.position.y) / self.resolution)
        index = row * self.occupancy_grid.info.width + col

        return col,row,index-1
    def get_index_from_xy(self, x, y):
        """
        Retrieve the 1D index from x, y coordinates.

        :param x: X-coordinate in meters.
        :param y: Y-coordinate in meters.
        :return: 1D index.
        """
        col = int((x - self.occupancy_grid.info.origin.position.x) / self.resolution)
        row = int((y - self.occupancy_grid.info.origin.position.y) / self.resolution)
        index = int(row * self.occupancy_grid.info.width + col-1)

        return index
    def get_xy_from_index(self, index):
        """
        Retrieve the x, y coordinates from a 1D index.

        :param index: 1D index.
        :return: Tuple of x, y coordinates.
        """
        col = index % self.occupancy_grid.info.width
        row = index // self.occupancy_grid.info.width
        x = self.occupancy_grid.info.origin.position.x + col * self.resolution
        y = self.occupancy_grid.info.origin.position.y + row * self.resolution
        return x, y
    def get_xy_from_col_row(self, col,row):
        """
        Retrieve the x, y coordinates from a col row.

        :param index: 1D index.
        :return: Tuple of x, y coordinates.
        """
        
        x = self.occupancy_grid.info.origin.position.x + col * self.resolution
        y = self.occupancy_grid.info.origin.position.y + row * self.resolution
        return x, y
