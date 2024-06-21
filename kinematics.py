


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
            self.hip_offset = 0.08 #The distance between hip joint and thigh joint

            self.body_length = 0.1881*2 #0.1881 is the distance between the hip joint and the body center point in x-y plane: x axis
            self.body_width = 0.04675*2 #0.04675 is the distance between the hip joint and the body center point in x-y plane: y axis


        #Leg Ids
        self.front_right = 0 
        self.front_left = 1
        self.rear_right =2 
        self.rear_left = 3

        self.phi = torch.deg2rad(90)

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
         
        x, y, z = xyz[0], xyz[1], xyz[2]

        # calculate the distance between the hip joint and the foot in y-z  plane
        len_A = torch.norm(torch.tensor([y, z])) #equiv. to sqrt(y**2 + z**2)

        #a_1: angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        a_1 = cartesian_to_polar(y, z)

        #a_2: angle between len_A and leg's projection line on y-z plane
        a_2 = torch.asin(torch.sin(self.phi)*self.hip_offset/len_A)

        torch.pi = torch.acos(torch.zeros(1)).item() * 2

        #a_3: angle between the hip_offset and length len_A
        a_3 = torch.pi - a_2 - self.phi

        #angle of hip_offset
        if is_right: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*torch.pi : theta_1 -= 2*torch.pi

        #Thigh joint coordinates
        j_2 = torch.tensor([0, self.hip_offset*torch.cos(theta_1), self.hip_offset*torch.sin(theta_1)]) 

        #Foot coordinates
        j_4 = torch.tensor(xyz)

        #Vector from j_2 to j_4
        J4_2_vec = j_4 - j_2

        if is_right: R = theta_1 - self.phi - torch.pi/2
        else: R = theta_1 + self.phi - torch.pi/2       

        #Create rotation matrix to work on a new 2D plane x z_
        rot_mtx = RotMatrix3D([-R, 0, 0], is_radians=True)
        j4_2_vec = rot_mtx @ J4_2_vec

        #xyz in the rotated coordinate system + offset due to hip_offset removed
        x_, y_, z_ = j4_2_vec[0], j4_2_vec[1], j4_2_vec[2]

        len_B = torch.norm(torch.tensor([x_, z_])) #equiv. to sqrt(x_**2 + z_**2)
        if len_B >= (self.thigh_leg + self.calf_leg):
            len_B = (self.thigh_leg + self.calf_leg) - 0.0001
            print("IK Error: Leg is overextended")
            return None
        
        b_1 = cartesian_to_polar(x_, z_)
        b_2 = torch.acos((self.thigh_leg**2 + len_B**2 - self.calf_leg**2)/(2*self.thigh_leg*len_B))
        b_3 = torch.acos((self.thigh_leg**2 + self.calf_leg**2 - len_B**2)/(2*self.thigh_leg*self.calf_leg))

        theta_2 = b_1 - b_2 
        theta_3 = torch.pi - b_3 

        j_1 = torch.tensor([0, 0, 0])

        j_3_ = torch.tensor ([self.thigh_leg*torch.cos(theta_2), 0, self.thigh_leg*torch.sin(theta_2)])
        j_3 = 







        



