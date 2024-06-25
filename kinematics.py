

from math import atan, pi, radians, cos, sin
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt




def point_to_rad(p1, p2):
    # Converts 2D cartesian points to polar angles in range -pi to pi
    angle = atan2(p2, p1)
    
    # Adjust angle to be in the range 0 to 2pi
    if angle < 0:
        angle += 2*pi
    
    return angle
    
def RotMatrix3D(rotation=[0,0,0]):
    
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

  
    # rotation matrix about each axis
    rotX = np.matrix([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    rotY = np.matrix([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    rotZ = np.matrix([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])

    return rotZ * rotY * rotX # roll pitch and yaw rotation 


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
        
        # leg origins (FR, FL, RR, RL), i.e., the coordinate of j1
        self.leg_origins = np.matrix([[self.length/2, self.width/2, 0],
                          [-self.length/2, self.width/2, 0],
                          [-self.length/2, -self.width/2, 0],
                          [self.length/2, -self.width/2, 0],
                          [self.length/2, self.width/2, 0]])
        
    # this method adjust inputs to the IK calculator by adding rotation and 
    # offset of that rotation from the center of the robot
    def leg_IK(self, xyz, rot = [0,0,0], legID=0, center_offset=[0,0,0]):

        #what is xyz? what does it represent? 

        
        # check is the leg is from the right side 
        is_right = (legID in self.right_legs)
        
        # add offset of each leg from the axis of rotation
        XYZ = asarray((inv(RotMatrix3D(rot)) * \
            ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        # subtract the offset between the leg and the center of rotation 
        # so that the resultant coordiante is relative to the origin (j1) of the leg
        xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()

        # calculate the angles and coordinates of the leg relative to the origin of the leg
        return self.leg_IK_calc(xyz_, is_right)


    # IK calculator
    def leg_IK_calc(self, xyz, is_right=False): 

        #Get the foot x, y, z coordinates
        x, y, z = xyz[0], xyz[1], xyz[2] 
        
        # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = np.sqrt(y**2 + z**2)

        #Calculate angle between len_A and the leg's projection line on YZ plane
        a_2 = np.arcsin(np.sin(self.phi)*self.link_1/len_A)

        #Calculate angle between link_1 and length len_A
        a_3 = np.pi - a_2 - self.phi

        #Calculate the angle from the positive y-axis to the end-effector
        a_1 = point_to_rad(y,z)

        #Length of the leg project in the YZ plane
        # len_H = np.sqrt(len_A**2 -self.link_1**2) 

        # angle_1 = np.arctan2(len_A, y)
        # theta_1 = a_3 - angle_1

        #Angle of link1 about the x-axis in positive y-axis
        if is_right: 
            theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3

            if theta_1 >= 2*np.pi:
                theta_1 -= 2*np.pi

        
        #Thight joint coordinates 
        #x, y, z = j2[0], j2[1], j2[2]
        j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)]) 

        #Foot joint coordinates
        j4 = array(xyz)
        #Vector from j2 to j4, i.e., the vector from the thight to the foot
        j4_2_vec = j4 - j2
    
    
        if is_right:
            R = theta_1 - self.phi - pi/2
        else: 
            R = theta_1 + self.phi - pi/2
        
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R,0,0])
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
        
        # modify angles to match robot's configuration (i.e., adding offsets)
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        #angles = [theta_1, theta_2, theta_3]
        # print(degrees(angles[0]))
        return [angles[0], angles[1], angles[2]]
    
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi; # add offset # 90 degrees
        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 0*pi/180 # 45 degrees initial offset #
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi
            else: theta_1 = angles[0]
            
            theta_2 = -angles[1] - 0*pi/180
        
        theta_3 = -angles[2] + 0*pi/180
        return [theta_1, theta_2, theta_3]
    

# Example usage:
if __name__ == "__main__":
    kinematics = kinematics()
    xyz_position = [0.0, 0.08, -0.2459]  # Example Cartesian position
    legID = 0  # Example leg ID
    angles = kinematics.leg_IK(xyz_position, legID=legID)
    print("Joint angles:", angles)







# from math import atan, pi, radians, cos, sin
# import numpy as np
# from numpy.linalg import inv, norm
# from numpy import array, asarray, matrix
# from math import *
# import matplotlib.pyplot as plt




# def point_to_rad(p1, p2):
#     # Converts 2D cartesian points to polar angles in range -pi to pi
#     angle = atan2(p2, p1)
    
#     # Adjust angle to be in the range 0 to 2pi
#     if angle < 0:
#         angle += 2*pi
    
#     return angle
    
# def RotMatrix3D(rotation=[0,0,0],is_radians=True, order='xyz'):
    
#     roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

#     # convert to radians is the input is in degrees
#     if not is_radians: 
#         roll = radians(roll)
#         pitch = radians(pitch)
#         yaw = radians(yaw)
    
#     # rotation matrix about each axis
#     rotX = np.matrix([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
#     rotY = np.matrix([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
#     rotZ = np.matrix([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    
#     # rotation matrix order (default: pitch -> roll -> yaw)
#     if order == 'xyz': rotationMatrix = rotZ * rotY * rotX
#     elif order == 'xzy': rotationMatrix = rotY * rotZ * rotX
#     elif order == 'yxz': rotationMatrix = rotZ * rotX * rotY
#     elif order == 'yzx': rotationMatrix = rotX * rotZ * rotY
#     elif order == 'zxy': rotationMatrix = rotY * rotX * rotZ
#     elif order == 'zyx': rotationMatrix = rotX * rotY * rotZ
    
#     return rotationMatrix # roll pitch and yaw rotation 


# class kinematics():
    
#     def __init__(self):
        
#         # note: leg IDs
#         left_front = 0
#         left_back  = 1
#         right_front= 2
#         right_back = 3
        
#         self.right_legs = [right_front, right_back]
        
#         self.link_1 = 0.08
#         self.link_2 = 0.213
#         self.link_3 = 0.213
#         self.phi = radians(90) 
        
#         # body dimensions
#         self.length = 0.3762
#         self.width = 0.0935
#         self.hight = 0.0
        
#         # leg origins (FR, FL, RR, RL), i.e., the coordinate of j1
#         self.leg_origins = np.matrix([[self.length/2, self.width/2, 0],
#                           [-self.length/2, self.width/2, 0],
#                           [-self.length/2, -self.width/2, 0],
#                           [self.length/2, -self.width/2, 0],
#                           [self.length/2, self.width/2, 0]])
        
#     # this method adjust inputs to the IK calculator by adding rotation and 
#     # offset of that rotation from the center of the robot
#     def leg_IK(self, xyz, rot = [0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):

#         #what is xyz? what does it represent? 

        
#         # check is the leg is from the right side 
#         is_right = (legID in self.right_legs)
        
#         # add offset of each leg from the axis of rotation
#         XYZ = asarray((inv(RotMatrix3D(rot,is_radians)) * \
#             ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
#         # subtract the offset between the leg and the center of rotation 
#         # so that the resultant coordiante is relative to the origin (j1) of the leg
#         xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()

#         # calculate the angles and coordinates of the leg relative to the origin of the leg
#         return self.leg_IK_calc(xyz_, is_right)


#     # IK calculator
#     def leg_IK_calc(self, xyz, is_right=False): 

#         #Get the foot x, y, z coordinates
#         x, y, z = xyz[0], xyz[1], xyz[2] 
        
#         # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
#         len_A = np.sqrt(y**2 + z**2)

#         #Calculate angle between len_A and the leg's projection line on YZ plane
#         a_2 = np.arcsin(np.sin(self.phi)*self.link_1/len_A)

#         #Calculate angle between link_1 and length len_A
#         a_3 = np.pi - a_2 - self.phi

#         #Calculate the angle from the positive y-axis to the end-effector
#         a_1 = point_to_rad(y,z)

#         #Length of the leg project in the YZ plane
#         # len_H = np.sqrt(len_A**2 -self.link_1**2) 

#         # angle_1 = np.arctan2(len_A, y)
#         # theta_1 = a_3 - angle_1

#         #Angle of link1 about the x-axis in positive y-axis
#         if is_right: 
#             theta_1 = a_1 - a_3
#         else: 
#             theta_1 = a_1 + a_3

#             if theta_1 >= 2*np.pi:
#                 theta_1 -= 2*np.pi

        
#         #Thight joint coordinates 
#         #x, y, z = j2[0], j2[1], j2[2]
#         j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)]) 

#         #Foot joint coordinates
#         j4 = array(xyz)
#         #Vector from j2 to j4, i.e., the vector from the thight to the foot
#         j4_2_vec = j4 - j2
    
    
#         if is_right:
#             R = theta_1 - self.phi - pi/2
#         else: 
#             R = theta_1 + self.phi - pi/2
        
#         # create rotation matrix to work on a new 2D plane (XZ_)
#         rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
#         j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        
#         # xyz in the rotated coordinate system + offset due to link_1 removed
#         x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
#         len_B = norm([x_, z_]) # norm(j4-j2)
        
#         # handling mathematically invalid input, i.e., point too far away to reach
#         if len_B >= (self.link_2 + self.link_3): 
#             len_B = (self.link_2 + self.link_3) * 0.99999
#             # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
#             print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
#         # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
#         # b_2 : angle between len_B and link_2
#         # b_3 : angle between link_2 and link_3
#         b_1 = point_to_rad(x_, z_)  
#         b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
#         b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
#         # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
#         theta_2 = b_1 - b_2    
#         theta_3 = pi - b_3
        
#         # modify angles to match robot's configuration (i.e., adding offsets)
#         angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
#         #angles = [theta_1, theta_2, theta_3]
#         # print(degrees(angles[0]))
#         return [angles[0], angles[1], angles[2]]
    
#     def angle_corrector(self, angles=[0,0,0], is_right=True):
#         angles[1] -= 1.5*pi; # add offset # 90 degrees
#         if is_right:
#             theta_1 = angles[0] - pi
#             theta_2 = angles[1] + 0*pi/180 # 45 degrees initial offset #
#         else: 
#             if angles[0] > pi:  
#                 theta_1 = angles[0] - 2*pi
#             else: theta_1 = angles[0]
            
#             theta_2 = -angles[1] - 0*pi/180
        
#         theta_3 = -angles[2] + 0*pi/180
#         return [theta_1, theta_2, theta_3]