import numpy as np

import sys 
import os

from robot.kinematics import robot_ik, robot_fk, jacobians, translation_matrix, JointState, CartesianGoal, DEFAULT_PARAMS


# compute the expanded mass (D) matrix
def expanded_mass(js: JointState, COM:list[CartesianGoal],M: np.ndarray):
    
    # compute jacobians for each link 
    T = []
    for idx in range(1, 5):
        T.append(robot_fk(js, 0, idx))
        
    for i in range(len(COM)) :
        # for each link the jacobian is made by all the jacobians of the previous links, stacked
        
        # create matrices for all the next links filled with zeros 
        Tc = T
        for r in range(4):
        
            if i-r >=0:
                # translate from link frame to COM frame
                tc = (T[r]) @ translation_matrix(COM[r])
                Tc[r] = tc
                
        print(len(Tc))
        Jci = jacobians(Tc)
        
        for r in range(4):
            if i-r <0:
                Jci[0][:,r] = np.array([0,0,0,0,0,0])
        
        try:
            Jc = np.vstack((Jc, Jci[0]))  
        except NameError:
            Jc = Jci[0]  # if J does not exist create it
            
    
    
    return np.transpose(Jc) @ M @ Jc
    
    
        