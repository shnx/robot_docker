#!/usr/bin/env python3

import numpy as np

import rospy
import time
from nav_msgs.msg import OccupancyGrid,MapMetaData
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped,Point
from utils import prob_to_log_odds

from utils import log_odds_to_prob
from utils import linear_mapping_of_values
from scipy.sparse import csr_matrix, eye
from matplotlib.lines import Line2D
import random
import tkinter as tk
from tkinter import Canvas, Button, Checkbutton
from tkinter import Label
import scipy.sparse
from Mapping import OccupancyGridMap
from Robot import Robot
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from Sensor_array import SonarArray
from sonar_subscriber import SonarSubscriber
from PIL import Image
from scipy.sparse import csr_matrix,linalg

from scipy.linalg import pinv

import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import ttk, Button
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.stats import norm
class TabbedApp:
    def __init__(self, root,sonar_subscriber,robot_pose_subscriber):
        self.sonar_subscriber = sonar_subscriber
        self.robot=robot_pose_subscriber
        self.opening_angle = self.sonar_subscriber.opening_angle
        self.num_J_cells_in_ray=1
        self.polar_chart_angles=[]
        self.polar_chart_values=[]

        distances = np.arange(0.01, self.sonar_subscriber.max_range, 0.01)
        rounded_distances = np.round(distances, 2)
        probabilities = np.linspace(0.03, 0.005, len(distances))
        self.prob_fa = dict(zip(rounded_distances, probabilities))


        angles = np.arange(0, self.opening_angle+1, 1)


        mean = self.opening_angle/2
        std_dev = 10
        max_probability = 0.95
        probabilities = norm.pdf(angles, mean, std_dev)


        probabilities /= np.max(probabilities)
        probabilities *= max_probability


        rounded_angles = np.round(angles, 1)


        self.p_true_detection_gaussian = dict(zip(rounded_angles, probabilities))


        self.root = root
        self.root.title("Inter-Cell Correlation")

        self.notebook = ttk.Notebook(root)
        self.tab1 = ttk.Frame(self.notebook)
        self.tab2 = ttk.Frame(self.notebook)
        self.tab3 = ttk.Frame(self.notebook)
        self.tab4 = ttk.Frame(self.notebook)
        self.tab5 = ttk.Frame(self.notebook)
        self.notebook.add(self.tab1, text="Tab 1 -GT")
        self.notebook.add(self.tab2, text="Tab 2 - Inv")
        self.notebook.add(self.tab3, text="Tab 3 - Sonar Values")
        self.notebook.add(self.tab4, text="Tab 4 - EKF")
        self.notebook.add(self.tab5, text="Tab 5 - Covariance")

        self.notebook.pack(expand=1, fill="both")

        
        next_step = Button(self.tab1, text="Next Step", command=self.next_step)
        next_step.grid(row=1, column=3, pady=5)
        next_step = Button(self.tab2, text="Next Step", command=self.next_step)
        next_step.grid(row=1, column=3, pady=5)
        next_step = Button(self.tab3, text="Next Step", command=self.next_step)
        next_step.grid(row=1, column=3, pady=5)
        next_step = Button(self.tab4, text="Next Step", command=self.next_step)
        next_step.grid(row=1, column=3, pady=5)
        next_step = Button(self.tab5, text="Next Step", command=self.next_step)
        next_step.grid(row=1, column=3, pady=5)
        

        self.PRIOR_PROB = 0.5
        self.OCC_PROB   = 0.8
        self.FREE_PROB  = 0.2
        
        #self.gt_subscriber=rospy.Subscriber("map", OccupancyGrid, self.next_step)#update plots each time gmapping creats new map


    def create_plots(self,tab1_img=np.random.rand(10, 10),tab2_img=np.random.rand(10, 10),tab4_img=np.random.rand(10, 10),tab5_img=np.random.rand(10, 10)):
        self.figure1, self.ax1 = plt.subplots() 
        
        self.plot_image(self.tab1, self.figure1, self.ax1, tab1_img, 'gray_r', 'GT MAP')
        label_text = 'Robot Position'
        self.ax1.text(self.x_robot_chart, self.y_robot_chart, label_text, color='red', verticalalignment='bottom', horizontalalignment='left')
        self.ax1.scatter(self.x_robot_chart, self.y_robot_chart, color='red', marker='o')
        canvas1 = FigureCanvasTkAgg(self.figure1, master=self.tab1)
        canvas1.draw()
        canvas1.get_tk_widget().grid(row=0, column=0, pady=5)
        plt.close(self.figure1)     


        self.figure2, self.ax2 = plt.subplots()
        self.plot_image(self.tab2, self.figure2, self.ax2, tab2_img, 'gray_r', "Log-odds INV Map")
        
        canvas2 = FigureCanvasTkAgg(self.figure2, master=self.tab2)
        canvas2.draw()
        canvas2.get_tk_widget().grid(row=0, column=0, pady=5)
        plt.close(self.figure2) 



        self.figure3, self.ax3 = plt.subplots(subplot_kw={'projection': 'polar'})
        
        self.ax3.set_ylim(0, np.max(self.polar_chart_values))
        self.ax3.scatter(self.polar_chart_angles, self.polar_chart_values, marker='o', linestyle='-', color='g')
        #self.ax3.set_theta_offset(np.pi / 2)
        self.ax3.set_theta_direction(1)
        self.ax3.set_rlabel_position(0)
        canvas3 = FigureCanvasTkAgg(self.figure3, master=self.tab3)
        canvas3.draw()
        canvas3.get_tk_widget().grid(row=0, column=0, pady=5)
        plt.close(self.figure3) 

        self.figure4, self.ax4 = plt.subplots()
        self.plot_image(self.tab4, self.figure4, self.ax4, tab4_img, 'gray_r', "EKF  Map")
        canvas4 = FigureCanvasTkAgg(self.figure4, master=self.tab4)
        canvas4.draw()
        canvas4.get_tk_widget().grid(row=0, column=0, pady=5)
        plt.close(self.figure4) 



        self.figure5, self.ax5 = plt.subplots()
        # self.ax5.imshow(ekf_map.Sigma, cmap='viridis', interpolation='nearest')
        # self.ax5.title('Covariance Matrix')
        # self.figure5.colorbar(label='Covariance')
        # canvas5 = FigureCanvasTkAgg(self.figure5, master=self.tab5)
        # canvas5.draw()
        # canvas5.get_tk_widget().grid(row=0, column=0, pady=5)
        # plt.close(self.figure5)    

        self.plot_image(self.tab5, self.figure5, self.ax5, tab5_img, 'coolwarm', "Covariance",vmin=int(np.min(tab5_img)),vmax=int(np.max(tab5_img)))
        canvas5 = FigureCanvasTkAgg(self.figure5, master=self.tab5)
        canvas5.draw()
        canvas5.get_tk_widget().grid(row=0, column=0, pady=5)

    def plot_image(self, tab, figure, axis, image, cmap, title,vmin=0, vmax=1):
        axis.clear()
        axis.grid(True, linestyle='--', color='black', alpha=0.5, which='both', linewidth=0.5)
        axis.set_title(title)

        im = axis.imshow(image, cmap=cmap, origin='lower', vmin=vmin, vmax=vmax)
        cbar = figure.colorbar(im, ax=axis, label='Value')

        return im, cbar
    
    def polar_to_cartesian(self,x1, y1, r, theta):
        x2 = x1 + r * np.cos(theta)
        y2 = y1 + r * np.sin(theta)
        return x2, y2

    def next_step(self,data=None):
        #robot.update()
        gt_map.prepare_img()#prepare image from occupancy grid message
        print("shape of gt map",gt_map.img.shape)
        print("shape of gt map small",gt_map.img_small.shape)
        print("shape of ekf map",ekf_map.img.shape)
        inv_map.publish_occupancy_grid()
        

      
        self.sonar_update()
        inv_map.img = np.clip(inv_map.img, 0, 1)
        unique_values = np.unique(inv_map.img)
        #print("Unique values in the before INV map:", unique_values)
        inv_map.img=np.round(inv_map.img, decimals=2)
        unique_values = np.unique(np.round(inv_map.img, decimals=2))
        #print("Unique values in the after INV map:", unique_values)
        
        
        self.create_plots(tab1_img=gt_map.img_small,tab2_img=inv_map.img,tab4_img=ekf_map.img,tab5_img=ekf_map.Sigma)
        """ 
        num_zeros_gt= np.count_nonzero(np.equal(gt_map.img_small, 0))
        num_zeros_inv= np.count_nonzero(np.equal(inv_map.img, 0))
        num_zeros_ekf= np.count_nonzero(np.equal(ekf_map.img, 0))
        iou_inv=float(num_zeros_inv/num_zeros_gt)
        iou_ekf=float(num_zeros_ekf/num_zeros_gt)
        f1_label = Label(self.tab5, text=f"num_zeros_gt : {num_zeros_gt}")
        f1_label.grid(row=30, column=0, pady=5)
        f2_label = Label(self.tab5, text=f"num_zeros_inv : {num_zeros_inv}")
        f2_label.grid(row=34, column=0, pady=5)
        f3_label = Label(self.tab5, text=f"num_zeros_ekf : {num_zeros_ekf}")
        f3_label.grid(row=36, column=0, pady=5)
        f4_label = Label(self.tab5, text=f"IOU INV : {iou_inv}")
        f4_label.grid(row=38, column=0, pady=5)
        f5_label = Label(self.tab5, text=f"IOU EKF : {iou_ekf}")
        f5_label.grid(row=40, column=0, pady=5)
        f_score_ekf=self.calculate_f1_score(ekf_map.img,gt_map.img_small,0.45,0.55)
        f_score_inv=self.calculate_f1_score(inv_map.img,gt_map.img_small,0.45,0.55)
        f5_label = Label(self.tab5, text=f"F-score INV : {f_score_inv}")
        f5_label.grid(row=42, column=0, pady=5)
        f5_label = Label(self.tab5, text=f"F-score EKF : {f_score_ekf}")
        f5_label.grid(row=44, column=0, pady=5)"""
        
    def sonar_update(self):

        self.robot_position=self.robot.get_position() 

        self.y_robot_chart=int(self.robot_position.y/inv_map.occupancy_grid.info.resolution)+inv_map.occupancy_grid.info.width  // 2
        self.x_robot_chart=int(self.robot_position.x/inv_map.occupancy_grid.info.resolution)+inv_map.occupancy_grid.info.width  // 2
        
        sonar_data = self.sonar_subscriber.generate_measurement()
        self.polar_chart_angles=[]
        self.polar_chart_values=[]
       
        #ekf_map.p_true_detection=ekf_map.empty_numpy_shaped # reset false alarm 
        #ekf_map.p_false_alarm=ekf_map.empty_numpy_shaped# reset 
        #ekf_map.free_perceived_prob=ekf_map.empty_numpy_shaped
        #ekf_map.H_squared=ekf_map.empty_numpy_shaped

        for topic in self.sonar_subscriber.sonar_topics:
            #topic="Sonar_front_30_left"
            J_all_set = set()#here unique cells (iy,ix) will be saved
            I_all_set = set()
            J_all_set.clear()
            I_all_set.clear()
            print(topic)
            J_free_perceived_prob_products=1.0
            I_free_perceived_prob_products=1.0
            
            d=sonar_data[topic]

            if (d is None): continue
            #print(self.robot.get_yaw_angle())
            #print(self.sonar_subscriber.extract_angle_from_topic(topic=topic))
            angle=self.sonar_subscriber.extract_angle_from_topic(topic=topic)+self.robot.get_yaw_angle()-np.deg2rad(self.opening_angle/2)#angle cone start

            print("d  of ",topic , d, "angle ", angle)
            #ekf_map.H_squared=np.zeros_like(ekf_map.H_squared)
            #ekf_map.derivative=np.zeros_like(ekf_map.derivative)
            print("Number of non-zero values H_squared:", np.count_nonzero(ekf_map.H_squared))
            print("Number of non-zero values derivative:", np.count_nonzero(ekf_map.derivative))
            for i in range(int(self.opening_angle)):
                
                
                I,J=inv_map.I_J(self.robot_position,d,angle+np.deg2rad(i))
                
                self.polar_chart_angles.append(angle+np.deg2rad(i))
                self.polar_chart_values.append(d)
                
                
                for index, (ix,iy) in enumerate(I): 
                    if 0 <= iy < ekf_map.img.shape[0] and 0 <= ix < ekf_map.img.shape[0] :


                        if(index>=len(I)-self.num_J_cells_in_ray):

                            p = self.OCC_PROB
                            J_all_set.add((ix, iy))

                            
                            #print("I_free_perceived_prob_products unique",np.unique(I_free_perceived_prob_products))
                            #J_free_perceived_prob_products=np.round(J_free_perceived_prob_products,5)
                        else :
                            p = self.FREE_PROB


                            I_all_set.add((ix, iy))
                            #if(np.round(ekf_map.free_perceived_prob[iy,ix],3)!=0):
                            
                            #J_free_perceived_prob_products=J_free_perceived_prob_products*ekf_map.free_perceived_prob[iy,ix]
                            #I_free_perceived_prob_products=np.round(I_free_perceived_prob_products,5)

                        
                        self.update_inv_map_with_logodds(inv_map,ix,iy,p)
                #print(ekf_map.free_perceived_prob)
                #print("unique",np.unique(ekf_map.free_perceived_prob))
                #print("I_free_perceived_prob_products unique",np.unique(I_free_perceived_prob_products))
            #print("as set",J_all_set)
            J_all = list(J_all_set)
            I_all = list(I_all_set)
            
            #ekf_map.mu_empty
            
            for index, (ix,iy) in enumerate(J_all):
                    #calculating H
                #ekf_map.H_squared[iy,ix]=0
                J_free_perceived_prob_products=J_free_perceived_prob_products*ekf_map.free_perceived_prob[iy,ix]
                ekf_map.p_false_alarm[iy,ix]=self.prob_fa.get(round(d,2), 0.005)
                ekf_map.p_true_detection[iy,ix]=self.p_true_detection_gaussian.get(i,0.7)
                part_1=ekf_map.state_to_prob(ekf_map.mu[iy,ix])*(1-ekf_map.p_true_detection[iy,ix])
                part_2=(1-ekf_map.state_to_prob(ekf_map.mu[iy,ix]))*(1-ekf_map.p_false_alarm[iy,ix])
                ekf_map.free_perceived_prob[iy,ix]=part_1+part_2 #p_miss
            for index, (ix,iy) in enumerate(J_all):                            
                ekf_map.derivative[iy,ix]=ekf_map.mderive(ekf_map.state_to_prob(ekf_map.mu[iy,ix]))
                #denominator = 1#ekf_map.free_perceived_prob[iy, ix]
                ekf_map.H_squared[iy,ix]=J_free_perceived_prob_products*(ekf_map.derivative[iy,ix]*((1-ekf_map.p_true_detection[iy,ix])-(1-ekf_map.p_false_alarm[iy,ix])*ekf_map.derivative[iy,ix]))



            start = time.time()

            print("time started")           
            #ekf_map.H=np.zeros_like(ekf_map.H)
            ekf_map.H=ekf_map.H_squared.copy()

            ekf_map.H=ekf_map.H.reshape(-1,1)

            H_sparse=ekf_map.H.T.copy()

            #Sigma_sparse = csr_matrix(ekf_map.Sigma)
            #Sigma_sparse = ekf_map.Sigma.copy()

            intermediate_term_sparse = H_sparse.dot(ekf_map.Sigma.dot(H_sparse.T))

            intermediate_term_sparse_reg = intermediate_term_sparse +(1-J_free_perceived_prob_products)*(1-(1-J_free_perceived_prob_products)) #alpha * np.eye(intermediate_term_sparse.shape[0])
            intermediate_inverse_sparse = pinv(intermediate_term_sparse_reg)

            K_sparse = ekf_map.Sigma.dot(H_sparse.T.dot(intermediate_inverse_sparse))

            ekf_map.K = np.array(K_sparse)

            
            ekf_map.Sigma =ekf_map.Sigma - ekf_map.Sigma.dot(ekf_map.K.dot(H_sparse))#I_minus_KH_sparse.dot(Sigma_sparse)

            temp=ekf_map.mu.reshape(-1, 1)

            

            #ekf_map.K = np.random.rand(*ekf_map.mu_flattened.shape)
            ekf_map.mu_flattened = (temp +ekf_map.K*(1-(1-J_free_perceived_prob_products)))

            ekf_map.mu_prob=ekf_map.state_to_prob(ekf_map.mu_flattened)

                
            ekf_map.img=ekf_map.mu_prob.reshape(ekf_map.original_shape,ekf_map.original_shape)
            ekf_map.mu=ekf_map.mu_flattened.reshape(ekf_map.original_shape,ekf_map.original_shape)
            #ekf_map.Sigma=scipy.sparse.identity(ekf_map.desired_width * ekf_map.desired_width)
            #ekf_map.Sigma=5*ekf_map.Sigma
            end = time.time()
            elapsed = end - start
            print("Time EKF J",elapsed)
            for index, (ix,iy) in enumerate(I_all):
                ekf_map.p_false_alarm[iy,ix]=self.prob_fa.get(round(d,2), 0.005)
                ekf_map.p_true_detection[iy,ix]=self.p_true_detection_gaussian.get(i,0.7)
                part_1=ekf_map.state_to_prob(ekf_map.mu[iy,ix])*(1-ekf_map.p_true_detection[iy,ix])
                part_2=(1-ekf_map.state_to_prob(ekf_map.mu[iy,ix]))*(1-ekf_map.p_false_alarm[iy,ix])
                ekf_map.free_perceived_prob[iy,ix]=part_1+part_2 #p_miss
                I_free_perceived_prob_products=I_free_perceived_prob_products*ekf_map.free_perceived_prob[iy,ix]
                    #calculating H
                #ekf_map.H_squared[iy,ix]=0
            for index, (ix,iy) in enumerate(I_all):
                ekf_map.derivative[iy,ix]=ekf_map.mderive(ekf_map.state_to_prob(ekf_map.mu[iy,ix]))
                #denominator = 1#ekf_map.free_perceived_prob[iy, ix]
                ekf_map.H_squared[iy,ix]=-I_free_perceived_prob_products*(ekf_map.derivative[iy,ix]*((1-ekf_map.p_true_detection[iy,ix])-(1-ekf_map.p_false_alarm[iy,ix])*ekf_map.derivative[iy,ix]))
            #print(len(I_all_set))
            start = time.time()

            print("time started")           
            #ekf_map.H=np.zeros_like(ekf_map.H)
            ekf_map.H=ekf_map.H_squared.copy()

            ekf_map.H=ekf_map.H.reshape(-1,1)

            H_sparse=ekf_map.H.T.copy()

            #Sigma_sparse = ekf_map.Sigma.copy()

            intermediate_term_sparse = H_sparse.dot(ekf_map.Sigma.dot(H_sparse.T))

            intermediate_term_sparse_reg = intermediate_term_sparse +(I_free_perceived_prob_products)*(1-(I_free_perceived_prob_products)) #alpha * np.eye(intermediate_term_sparse.shape[0])
            intermediate_inverse_sparse = pinv(intermediate_term_sparse_reg)

            K_sparse = ekf_map.Sigma.dot(H_sparse.T.dot(intermediate_inverse_sparse))

            ekf_map.K = np.array(K_sparse)

            
            ekf_map.Sigma =ekf_map.Sigma - ekf_map.Sigma.dot(ekf_map.K.dot(H_sparse))#I_minus_KH_sparse.dot(Sigma_sparse)

            temp=ekf_map.mu.reshape(-1, 1)

            

            #ekf_map.K = np.random.rand(*ekf_map.mu_flattened.shape)
            ekf_map.mu_flattened = (temp +ekf_map.K*(1-(I_free_perceived_prob_products)))

            ekf_map.mu_prob=ekf_map.state_to_prob(ekf_map.mu_flattened)

                
            ekf_map.img=ekf_map.mu_prob.reshape(ekf_map.original_shape,ekf_map.original_shape)
            ekf_map.mu=ekf_map.mu_flattened.reshape(ekf_map.original_shape,ekf_map.original_shape)
            #ekf_map.Sigma=scipy.sparse.identity(ekf_map.desired_width * ekf_map.desired_width)
            #ekf_map.Sigma=5*ekf_map.Sigma
            end = time.time()
            elapsed = end - start
            print("Time EKF I",elapsed)
            
            print(np.max(ekf_map.Sigma))
            print(np.min(ekf_map.Sigma))
            print(np.min(ekf_map.Sigma))

        
            


    def update_inv_map_with_logodds(self,inv_map,ix,iy,p):
        if 0 <= iy < inv_map.inv_map.shape[0] and 0 <= ix < inv_map.inv_map.shape[1]:


                        
                    
            l_prev = inv_map.inv_map[iy, ix]
                        #l_prev = inv_map.inv_map[iy,ix]
                    
            l = l_prev + prob_to_log_odds(p) - prob_to_log_odds(self.PRIOR_PROB)
                        
                        #l=np.clip(0,prob_to_log_odds)
            inv_map.inv_map[iy,ix]=l
                        
            temp=log_odds_to_prob(l)
                        #temp=np.clip(temp,0.0,1.0)
            inv_map.img[iy, ix] =temp 

    def threshold_map(self,map_array, lower_limit, upper_limit):
        """
        Thresholds the occupancy grid map array.

        Parameters:
        - map_array: NumPy array representing the occupancy grid map.
        - lower_limit: Lower limit for thresholding.
        - upper_limit: Upper limit for thresholding.

        Returns:
        - Binary thresholded map array.
        """
        thresholded_map = np.copy(map_array)
        thresholded_map[thresholded_map < lower_limit] = 0
        thresholded_map[thresholded_map > upper_limit] = 1
        return thresholded_map
 
    def calculate_f1_score(self,predicted_map, ground_truth_map, lower_limit, upper_limit):
        """
        Calculates the F1 score after thresholding the occupancy grid maps.

        Parameters:
        - predicted_map: NumPy array representing the predicted occupancy grid map.
        - ground_truth_map: NumPy array representing the ground truth occupancy grid map.
        - lower_limit: Lower limit for thresholding.
        - upper_limit: Upper limit for thresholding.

        Returns:
        - F1 score.
        """
        # Threshold the maps
        valid_indices = np.where(ground_truth_map != 0.5)
        filtered_ground_truth_map = ground_truth_map[valid_indices]
        filtered_predicted_map = predicted_map[valid_indices]
        predicted_binary_map = self.threshold_map(filtered_predicted_map, lower_limit, upper_limit)
        ground_truth_binary_map = self.threshold_map(filtered_ground_truth_map, lower_limit, upper_limit)

        # Calculate True Positives, False Positives, False Negatives
        TP = np.sum(np.logical_and(predicted_binary_map == 0, ground_truth_binary_map == 0))
        FP = np.sum(np.logical_and(predicted_binary_map == 0, ground_truth_binary_map == 1))
        FN = np.sum(np.logical_and(predicted_binary_map == 1, ground_truth_binary_map == 0))

        # Calculate Precision and Recall
        precision = TP / (TP + FP) if (TP + FP) != 0 else 0
        recall = TP / (TP + FN) if (TP + FN) != 0 else 0

        # Calculate F1 score
        f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) != 0 else 0

        return f1_score

    def calculate_iou(self,predicted_map, ground_truth_map, lower_limit, upper_limit):
        """
        Calculates the Intersection over Union (IoU) after thresholding the occupancy grid maps.

        Parameters:
        - predicted_map: NumPy array representing the predicted occupancy grid map.
        - ground_truth_map: NumPy array representing the ground truth occupancy grid map.
        - lower_limit: Lower limit for thresholding.
        - upper_limit: Upper limit for thresholding.

        Returns:
        - IoU (Intersection over Union).
        """
        # Threshold the maps
        predicted_binary_map = self.threshold_map(predicted_map, lower_limit, upper_limit)
        ground_truth_binary_map = self.threshold_map(ground_truth_map, lower_limit, upper_limit)

        # Calculate True Positives, False Positives, False Negatives
        TP = np.sum(np.logical_and(predicted_binary_map == 0, ground_truth_binary_map == 0))
        FP = np.sum(np.logical_and(predicted_binary_map == 0, ground_truth_binary_map == 1))
        FN = np.sum(np.logical_and(predicted_binary_map == 1, ground_truth_binary_map == 0))

        # Calculate IoU
        iou = TP / (TP + FP + FN) if (TP + FP + FN) != 0 else 0

        return iou




if __name__ == '__main__':
    try:
        rospy.init_node('gui_node')
        #rospy.Subscriber("/base_scan", LaserScan, self.callback_laser_scan)
        
        gt_map=OccupancyGridMap(name="GT_map",use_subscriber=True,use_sub_topic="/map")
        inv_map = OccupancyGridMap(name='INV_map',resolution=0.05, width=4, height=4, center_x=0, center_y=0, use_subscriber=False,use_sub_topic=None)
        ekf_map = OccupancyGridMap(name='EKF_map',resolution=0.05, width=4, height=4, center_x=0, center_y=0, use_subscriber=False,use_sub_topic=None,ekf=True)
        sonar=SonarSubscriber()# to get the sonar measurments from gazebo or our sonar model
        robot=Robot()# to get the robot gt pose from gazebo
        root = tk.Tk()
        app = TabbedApp(root,sonar,robot)
        root.mainloop()
        rospy.rate = rospy.Rate(0.5)
        rospy.rate.sleep()
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    