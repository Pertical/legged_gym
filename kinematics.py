


import numpy as np
import torch 

from legged_gym.utils.math import *


#converts 2D cartesian points to polar anglers in radians: 0 - 2pi
def cartesian_to_polar(x, y):
    rad = torch.atan2(x, y)
    return torch.where(rad >= 0, rad, rad + 2*torch.pi)

#Rotation matrix in 3D 

def RotMatrix3D(rotation=torch.tensor([0,0,0]), is_radians=True, order='xyz'):
    roll, pitch, yaw = rotation

    # convert to radians if the input is in degrees
    if not is_radians: 
        roll = torch.radians(roll)
        pitch = torch.radians(pitch)
        yaw = torch.radians(yaw)
    
    # rotation matrix about each axis
    rotX = torch.tensor([[1, 0, 0], [0, torch.cos(roll), -torch.sin(roll)], [0, torch.sin(roll), torch.cos(roll)]])
    rotY = torch.tensor([[torch.cos(pitch), 0, torch.sin(pitch)], [0, 1, 0], [-torch.sin(pitch), 0, torch.cos(pitch)]])
    rotZ = torch.tensor([[torch.cos(yaw), -torch.sin(yaw), 0], [torch.sin(yaw), torch.cos(yaw), 0], [0, 0, 1]])

    # dictionary mapping rotation order to rotation matrices
    rotation_order = {
        'xyz': rotZ @ rotY @ rotX,
        'xzy': rotY @ rotZ @ rotX,
        'yxz': rotZ @ rotX @ rotY,
        'yzx': rotX @ rotZ @ rotY,
        'zxy': rotY @ rotX @ rotZ,
        'zyx': rotX @ rotY @ rotZ
    }

    return rotation_order.get(order, rotZ @ rotY @ rotX)  # return the corresponding rotation matrix
    



class kinematics():

    def __init__(self, robot_type):
        self.robot_type = robot_type

        if robot_type == 'go1':
            self.thigh_leg = 0.213
            self.calf_leg = 0.213
            self.hip_offset = 0.08

            self.body_length = 0.1881*2 #0.1881 is the distance between the hip joint and the body center point in x-y plane: x axis
            self.body_width = 0.04675*2 #0.04675 is the distance between the hip joint and the body center point in x-y plane: y axis


        #Leg Ids
        self.front_right = 0 
        self.front_left = 1
        self.rear_right =2 
        self.rear_left = 3

        self.right_legs = [self.front_right, self.rear_right]
        
        #Hip origins (front_right, front_left, rear_right, rear_left)
        #i.e., the coordinate of hip joints 
        self.hip_origins = torch.tensor([[self.body_length/2, -self.body_width/2, 0],
                                         [self.body_length/2, self.body_width/2, 0],
                                         [-self.body_length/2, -self.body_width/2, 0],
                                         [-self.body_length/2, self.body_width/2, 0]])
        

    #Adjust rotation and translation
    def leg_IK(self, xyz, rot = torch.tensor([0,0,0]), legID=0, is_radians=True, center_offset=torch.tensor([0,0,0])):
            
            # check is the leg is from the right side 
            is_right = (legID in self.right_legs)
            
            # add offset of each leg from the axis of rotation
            XYZ = torch.inverse(RotMatrix3D(rot,is_radians)) @ \
                ((torch.tensor(xyz) + self.leg_origins[legID,:] - torch.tensor(center_offset)).t()).t()
            
            # subtract the offset between the leg and the center of rotation 
            # so that the resultant coordiante is relative to the origin (j1) of the leg
            xyz_ = (XYZ - self.leg_origins[legID,:] + torch.tensor(center_offset)).flatten()

            # calculate the angles and coordinates of the leg relative to the origin of the leg
            return self.leg_IK_calc(xyz_, is_right)
    
    def leg_IK_calc(self, xyz, is_right=False):
         
         x, y, z = xyz

         