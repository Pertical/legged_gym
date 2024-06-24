


# import numpy as np
# import torch 

# from legged_gym.utils.math import *


# #converts 2D cartesian points to polar anglers in radians: 0 - 2pi
# def cartesian_to_polar(x, y):
#     rad = torch.atan2(x, y)
#     return torch.where(rad >= 0, rad, rad + 2*torch.pi)

# #Rotation matrix in 3D 

# def RotMatrix3D(rotation=torch.tensor([0,0,0]), is_radians=True, order='xyz'):
#     roll, pitch, yaw = rotation

#     # convert to radians if the input is in degrees
#     if not is_radians: 
#         roll = torch.radians(roll)
#         pitch = torch.radians(pitch)
#         yaw = torch.radians(yaw)
    
#     # rotation matrix about each axis
#     rotX = torch.tensor([[1, 0, 0], [0, torch.cos(roll), -torch.sin(roll)], [0, torch.sin(roll), torch.cos(roll)]])
#     rotY = torch.tensor([[torch.cos(pitch), 0, torch.sin(pitch)], [0, 1, 0], [-torch.sin(pitch), 0, torch.cos(pitch)]])
#     rotZ = torch.tensor([[torch.cos(yaw), -torch.sin(yaw), 0], [torch.sin(yaw), torch.cos(yaw), 0], [0, 0, 1]])

#     # dictionary mapping rotation order to rotation matrices
#     rotation_order = {
#         'xyz': rotZ @ rotY @ rotX,
#         'xzy': rotY @ rotZ @ rotX,
#         'yxz': rotZ @ rotX @ rotY,
#         'yzx': rotX @ rotZ @ rotY,
#         'zxy': rotY @ rotX @ rotZ,
#         'zyx': rotX @ rotY @ rotZ
#     }

#     return rotation_order.get(order, rotZ @ rotY @ rotX)  # return the corresponding rotation matrix
    

# class kinematics():

#     def __init__(self, robot_type):
#         self.robot_type = robot_type

#         if robot_type == 'go1':
#             self.thigh_leg = 0.213
#             self.calf_leg = 0.213
#             self.hip_offset = 0.08 #The distance between hip joint and thigh joint

#             self.body_length = 0.1881*2 #0.1881 is the distance between the hip joint and the body center point in x-y plane: x axis
#             self.body_width = 0.04675*2 #0.04675 is the distance between the hip joint and the body center point in x-y plane: y axis


#         #Leg Ids
#         self.front_right = 0 
#         self.front_left = 1
#         self.rear_right =2 
#         self.rear_left = 3

#         self.phi = torch.deg2rad(90)

#         self.right_legs = [self.front_right, self.rear_right]
        
#         #Hip origins (front_right, front_left, rear_right, rear_left)
#         #i.e., the coordinate of hip joints 
#         self.hip_origins = torch.tensor([[self.body_length/2, -self.body_width/2, 0],
#                                          [self.body_length/2, self.body_width/2, 0],
#                                          [-self.body_length/2, -self.body_width/2, 0],
#                                          [-self.body_length/2, self.body_width/2, 0]])
        

#     #Adjust rotation and translation
#     def leg_IK(self, xyz, rot = torch.tensor([0,0,0]), legID=0, is_radians=True, center_offset=torch.tensor([0,0,0])):
            
#         # check is the leg is from the right side 
#         is_right = (legID in self.right_legs)
        
#         # add offset of each leg from the axis of rotation
#         XYZ = torch.inverse(RotMatrix3D(rot,is_radians)) @ \
#             ((torch.tensor(xyz) + self.leg_origins[legID,:] - torch.tensor(center_offset)).t()).t()
        
#         # subtract the offset between the leg and the center of rotation 
#         # so that the resultant coordiante is relative to the origin (j1) of the leg
#         xyz_ = (XYZ - self.leg_origins[legID,:] + torch.tensor(center_offset)).flatten()

#         # calculate the angles and coordinates of the leg relative to the origin of the leg
#         return self.leg_IK_calc(xyz_, is_right)

#     def leg_IK_calc(self, xyz, is_right=False):
         
#         x, y, z = xyz[0], xyz[1], xyz[2]

#         # calculate the distance between the hip joint and the foot in y-z  plane
#         len_A = torch.norm(torch.tensor([y, z])) #equiv. to sqrt(y**2 + z**2)

#         #a_1: angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
#         a_1 = cartesian_to_polar(y, z)

#         #a_2: angle between len_A and leg's projection line on y-z plane
#         a_2 = torch.asin(torch.sin(self.phi)*self.hip_offset/len_A)

#         torch.pi = torch.acos(torch.zeros(1)).item() * 2

#         #a_3: angle between the hip_offset and length len_A
#         a_3 = torch.pi - a_2 - self.phi

#         #angle of hip_offset
#         if is_right: theta_1 = a_1 - a_3
#         else: 
#             theta_1 = a_1 + a_3
#             if theta_1 >= 2*torch.pi : theta_1 -= 2*torch.pi

#         #Thigh joint coordinates
#         j_2 = torch.tensor([0, self.hip_offset*torch.cos(theta_1), self.hip_offset*torch.sin(theta_1)]) 

#         #Foot coordinates
#         j_4 = torch.tensor(xyz)

#         #Vector from j_2 to j_4
#         J4_2_vec = j_4 - j_2

#         if is_right: R = theta_1 - self.phi - torch.pi/2
#         else: R = theta_1 + self.phi - torch.pi/2       

#         #Create rotation matrix to work on a new 2D plane x z_
#         rot_mtx = RotMatrix3D([-R, 0, 0], is_radians=True)
#         j4_2_vec = rot_mtx @ J4_2_vec

#         #xyz in the rotated coordinate system + offset due to hip_offset removed
#         x_, y_, z_ = j4_2_vec[0], j4_2_vec[1], j4_2_vec[2]

#         len_B = torch.norm(torch.tensor([x_, z_])) #equiv. to sqrt(x_**2 + z_**2)
#         if len_B >= (self.thigh_leg + self.calf_leg):
#             len_B = (self.thigh_leg + self.calf_leg) - 0.0001
#             print("IK Error: Leg is overextended")
#             return None
        
#         b_1 = cartesian_to_polar(x_, z_)
#         b_2 = torch.acos((self.thigh_leg**2 + len_B**2 - self.calf_leg**2)/(2*self.thigh_leg*len_B))
#         b_3 = torch.acos((self.thigh_leg**2 + self.calf_leg**2 - len_B**2)/(2*self.thigh_leg*self.calf_leg))

#         theta_2 = b_1 - b_2 
#         theta_3 = torch.pi - b_3 

#         j_1 = torch.tensor([0, 0, 0])

#         j_3_ = torch.tensor ([self.thigh_leg*torch.cos(theta_2), 0, self.thigh_leg*torch.sin(theta_2)])
#         j_3 = 




from math import atan, pi, radians, cos, sin
import numpy as np
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt

def point_to_rad(p1, p2): # converts 2D cartesian points to polar angles in range 0 - 2pi
        
    if (p1 > 0 and p2 >= 0): return atan(p2/(p1))
    elif (p1 == 0 and p2 >= 0): return pi/2
    elif (p1 < 0 and p2 >= 0): return -abs(atan(p2/p1)) + pi
    elif (p1 < 0 and p2 < 0): return atan(p2/p1) + pi
    elif (p1 > 0 and p2 < 0): return -abs(atan(p2/p1)) + 2*pi
    elif (p1 == 0 and p2 < 0): return pi * 3/2
    elif (p1 == 0 and p2 == 0): return pi * 3/2 # edge case
    
def RotMatrix3D(rotation=[0,0,0],is_radians=True, order='xyz'):
    
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

    # convert to radians is the input is in degrees
    if not is_radians: 
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)
    
    # rotation matrix about each axis
    rotX = np.matrix([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    rotY = np.matrix([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    rotZ = np.matrix([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    
    # rotation matrix order (default: pitch -> roll -> yaw)
    if order == 'xyz': rotationMatrix = rotZ * rotY * rotX
    elif order == 'xzy': rotationMatrix = rotY * rotZ * rotX
    elif order == 'yxz': rotationMatrix = rotZ * rotX * rotY
    elif order == 'yzx': rotationMatrix = rotX * rotZ * rotY
    elif order == 'zxy': rotationMatrix = rotY * rotX * rotZ
    elif order == 'zyx': rotationMatrix = rotX * rotY * rotZ
    
    return rotationMatrix # roll pitch and yaw rotation 




class kinematics():
    
    def __init__(self):
        
        # note: leg IDs
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        self.link_1 = 0.08
        self.link_2 = 0.213
        self.link_3 = 0.213
        self.phi = radians(90)
        
        # body dimensions
        self.length = 0.3762
        self.width = 0.0935
        self.hight = 0.0
        
        # leg origins (left_f, left_b, right_b, right_f), i.e., the coordinate of j1
        self.leg_origins = np.matrix([[self.length/2, self.width/2, 0],
                          [-self.length/2, self.width/2, 0],
                          [-self.length/2, -self.width/2, 0],
                          [self.length/2, -self.width/2, 0],
                          [self.length/2, self.width/2, 0]])
        
    # this method adjust inputs to the IK calculator by adding rotation and 
    # offset of that rotation from the center of the robot
    def leg_IK(self, xyz, rot = [0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):

        #what is xyz? what does it represent? 

        
        # check is the leg is from the right side 
        is_right = (legID in self.right_legs)
        
        # add offset of each leg from the axis of rotation
        XYZ = asarray((inv(RotMatrix3D(rot,is_radians)) * \
            ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        # subtract the offset between the leg and the center of rotation 
        # so that the resultant coordiante is relative to the origin (j1) of the leg
        xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()

        # calculate the angles and coordinates of the leg relative to the origin of the leg
        return self.leg_IK_calc(xyz_, is_right)


    # IK calculator
    def leg_IK_calc(self, xyz, is_right=False): 

        x, y, z = xyz[0], xyz[1], xyz[2]    # unpack coordinates
        
        # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = norm([0,y,z])   
        
        # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        # a_2 : angle bewtween len_A and leg's projection line on YZ plane
        # a_3 : angle between link1 and length len_A
        a_1 = point_to_rad(y,z)                     
        a_2 = asin(sin(self.phi)*self.link_1/len_A) 
        a_3 = pi - a_2 - self.phi                   
        
        # angle of link1 about the x-axis 
        if is_right: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*pi: theta_1 -= 2*pi
        
        j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2 # vector from j2 to j4
        
        if is_right: R = theta_1 - self.phi - pi/2
        else: R = theta_1 + self.phi - pi/2
        
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        
        # xyz in the rotated coordinate system + offset due to link_1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        len_B = norm([x_, z_]) # norm(j4-j2)
        
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3
        
        # CALCULATE THE COORDINATES OF THE JOINTS FOR VISUALIZATION
        j1 = np.array([0,0,0])
        
        # calculate joint 3
        j3_ = np.reshape(np.array([self.link_2*cos(theta_2),0, self.link_2*sin(theta_2)]),[3,1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j3_, [1,3])).flatten()
        
        # calculate joint 4
        j4_ = j3_ + np.reshape(np.array([self.link_3*cos(theta_2+theta_3),0, self.link_3*sin(theta_2+theta_3)]), [3,1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j4_, [1,3])).flatten()
        
        # modify angles to match robot's configuration (i.e., adding offsets)
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        # print(degrees(angles[0]))
        return [angles[0], angles[1], angles[2], j1, j2, j3, j4]
    
    
    def base_pose(self, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        
        # offset due to non-centered axes of rotation
        offset = RotMatrix3D(rot, is_radians) * \
            (matrix(center_offset).transpose()) - matrix(center_offset).transpose()
        
        # rotate the base around the center of rotation (if there is no offset, then the center of 
        # rotation will be at the center of the robot)
        rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
        return rotated_base.transpose()
       
    # get coordinates of leg joints relative to j1
    def leg_pose(self, xyz, rot, legID, is_radians, center_offset=[0,0,0]):
        
        # get the coordinates of each joints relative to the leg's origin
        pose_relative = self.leg_IK(xyz, rot, legID, is_radians, center_offset)[3:]
        
        # adjust the coordinates according to the robot's orientation (roll, pitch, yaw)
        pose_true = RotMatrix3D(rot,is_radians) * (array(pose_relative).transpose())
        return pose_true.transpose()
    
    # plot rectangular base where each corner represents the origin of leg
    def plot_base(self, ax, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = (self.base_pose(rot, is_radians, center_offset)).transpose()     
        # plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'r')
        return
       
    # plot leg 
    def plot_leg(self, ax, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = ((self.leg_pose(xyz, rot, legID, is_radians, center_offset) \
                + self.base_pose(rot,is_radians,center_offset)[legID]).transpose())
        # plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'b')
        return

    def plot_robot(self, xyz, rot=[0,0,0], leg_N=4, is_radians=True, limit=0.250, center_offset=[0,0,0]):
    
        ax = self.ax_view(limit)  # set the view
        self.plot_base(ax,rot, is_radians, center_offset)  # plot base

        # plot legs
        for leg in range(leg_N):
            self.plot_leg(ax,xyz[leg],rot,leg, is_radians, center_offset) 
        
        # show figure
        plt.show()
        return
    

    # TO-DO : modify this function depending on your robot's configuration
    # adjusting angle for specific configurations of motors, incl. orientation
    # this will vary for each robot (possibly for each leg as well)
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi; # add offset # 90 degrees
        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 45*pi/180 # 45 degrees initial offset #
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi
            else: theta_1 = angles[0]
            
            theta_2 = -angles[1] - 45*pi/180
        
        theta_3 = -angles[2] + 45*pi/180
        return [theta_1, theta_2, theta_3]
        
    # set view  
    @staticmethod
    def ax_view(limit):
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax
    

if __name__ == "__main__":

    k = kinematics()








        



