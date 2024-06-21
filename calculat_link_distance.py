

import torch
import math 


#Joint limitations: for referencen 
        #Hip joint: -60 ~ 60 degrees, in radians: -1.0472 ~ 1.0472
        #Thigh joint: -38 ~ 170 degrees, in radians: -0.663225115 ~ 2.96705973
        #Calf joint: -156 ~ -48 degrees, in radians: -2.70526034 ~ -0.837758041


        # namespace UNITREE_LEGGED_SDK 
        # {
        #     constexpr double go1_Hip_max   = 1.047;    // unit:radian ( = 60   degree)
        #     constexpr double go1_Hip_min   = -1.047;   // unit:radian  ( = -60  degree)
        #     constexpr double go1_Thigh_max = 2.966;    // unit:radian ( = 170  degree)
        #     constexpr double go1_Thigh_min = -0.663;   // unit:radian ( = -38  degree)
        #     constexpr double go1_Calf_max  = -0.837;   // unit:radian ( = -48  degree)
        #     constexpr double go1_Calf_min  = -2.721;   // unit:radian ( = -156 degree)
        # }

        #Enum difinition for the four legs in Unitree Go1 
        # Leg0 FR = right front leg
        # Leg1 FL = left front leg
        # Leg2 RR = right rear leg
        # Leg3 RL = left rear leg


def distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2] - point1[2])**2)


def relativeCoorCalculation(coor1, coor2):

    relative_coor = [coor2[0] + coor1[0], coor2[1] + coor1[1], coor2[2] + coor1[2]]
    return relative_coor

def calLinkLength(base_coor, hip_joint_coor, thigh_joint_coor, calf_joint_coor, foot_coor):
    hipToBase_Distance = distance(base_coor, hip_joint_coor)

    thighJoint_RelatedToHipJointCoor = relativeCoorCalculation(hip_joint_coor, thigh_joint_coor)
    thighToHip_Distance = distance(hip_joint_coor, thighJoint_RelatedToHipJointCoor)

    calfJoint_RelatedToThighJointCoor = relativeCoorCalculation(thigh_joint_coor, calf_joint_coor)
    calfToThigh_Distance = distance(thigh_joint_coor, calfJoint_RelatedToThighJointCoor)

    footJoint_RelatedToCalfJointCoor = relativeCoorCalculation(calf_joint_coor, foot_coor)
    footToCalf_Distance = distance(calf_joint_coor, footJoint_RelatedToCalfJointCoor)

    return hipToBase_Distance, thighToHip_Distance, calfToThigh_Distance, footToCalf_Distance


def inverseKinematic(desire_bodyheight, thigh_length, calf_length):

    height = desire_bodyheight

    #Calculate Calf Joint angle based on the height
    calf_angle = math.acos((thigh_length**2 + calf_length**2 - height**2) / (2 * thigh_length * calf_length))

    #Calculate Thigh Joint angle based on the height
    thigh_angle = math.acos((height**2 + thigh_length**2 - calf_length**2) / (2 * height * thigh_length))

    return thigh_angle, calf_angle



def InverseKinematics(x, y, z):

    """
    x: x coordinate of the foot in meters 
    y: y coordinate of the foot in meters 
    z: z coordinate of the foot in meters

    For GO1, the rotation axis of the hip joint is the x-axis, the thigh joint is the y-axis, and the calf joint is the y-axis.
    The positive rotation direction conforms to the right-hand rule. 
    The positive direction of the x-axis is the forward direction of the robot.
    the positive direction of the y-axis is the upward direction of the robot, and the positive direction of the z-axis is the right direction of the robot.


    """

    #Constant lengths in meters 
    thigh_leg = 0.213 
    calf_leg = 0.213
    hip_offset = 0.08 

    thigh_leg = torch.tensor(thigh_leg)
    calf_leg = torch.tensor(calf_leg)
    hip_offset = torch.tensor(hip_offset)

    x = torch.tensor(x)
    y = torch.tensor(y)
    z = torch.tensor(z)

    #Calulating Hip joint angle, which rotates around x-axis

    d = torch.sqrt(z**2 + y**2) #Distance between the hip joint and the foot
    l = torch.sqrt(d**2 - hip_offset**2) #Distance between the hip joint and the foot in the x-y plane

    gamma_1 = - torch.atan2(y, z) #Angle between the z-axis to the line connecting the hip joint and the foot: d
    gamma_2 = torch.acos(hip_offset/d)  

    gamma = gamma_1 - gamma_2 #Hip Joint angle 

    #Calculating Thigh joint angle, which rotates around y-axis
    s = torch.sqrt(l**2 + x**2) #Distance between the hip joint and the foot in the x-z plane
    n = (s**2 - thigh_leg**2 - calf_leg**2)/(2 * thigh_leg)
    alpha = -torch.acos(n/calf_leg) #Calf Joint angle

    beta_1 = torch.acos((thigh_leg + n)/s) 
    beta_2 = -torch.atan2(x, l) 
    beta = beta_1 + beta_2 #Thigh Joint angle



    return gamma, alpha, beta

# Hip to Base center distance: 0.19382252836035338
# Thigh to Hip center distance: 0.08
# Thigh Length 0.213
# Calf Length 0.213
# Hip angle:  0.9228630819872481
# Calf angle:  -2.0809598055918954
# Thigh angle:  1.0676578979048683


# Hip angle:  tensor(-0.9229)
# Calf angle:  tensor(-1.0606)
# Thigh angle:  tensor(-0.0070)




if __name__ == "__main__":

    base_coor = [0., 0., 0.]
    hip_joint_coor = [0.1881, -0.04675, 0.]
    thigh_joint_coor = [0., -0.08, 0.]
    calf_joint_coor = [0., 0., -0.213]
    foot_coor = [0., 0., -0.213]

    FLfoot_desired_coor = [0.1881, -0.12675, -0.2] #FL
    FRfoot_desired_coor = [0.1881, 0.12675, -0.2] #FR
    RRfoot_desired_coor = [-0.1881, 0.12675, -0.2] #RR
    RLfoot_desired_coor = [-0.1881, -0.12675, -0.2] #RL 


    # footToCalfCoor = relativeCoorCalculation(calf_joint_coor, foot_coor)

    # print("footToCalfCoor: ", footToCalfCoor

    #hip jointcoor: 0.1881, -0.04675, 0.
    #thigh joint coor within the base coor: 0.1881, -0.12675, 0.
    #calf joint coor within the base joint coor: 0.1881, -0.12675, -0.213
    #foot joint coor within the based joint coor: 0.1881, -0.12675, -0.426 

    hipToBase_Distance, thighToHip_Distance, calfToThigh_Distance, footToCalf_Distance = calLinkLength(base_coor, hip_joint_coor, thigh_joint_coor, calf_joint_coor, foot_coor)

    print("Hip to Base center distance:", hipToBase_Distance)
    print("Thigh to Hip center distance:",thighToHip_Distance)
    print("Thigh Length",calfToThigh_Distance)
    print("Calf Length", footToCalf_Distance)


    # foot_desired_coor[2] = 0.30

    #Print all the angles for each foot desired position

    for foot_desired_coor in [FLfoot_desired_coor, FRfoot_desired_coor, RRfoot_desired_coor, RLfoot_desired_coor]:

        gamma, alpha, beta = InverseKinematics(foot_desired_coor[0], foot_desired_coor[1], foot_desired_coor[2])

        print("Hip angle: ", gamma)
        print("Calf angle: ", alpha)
        print("Thigh angle: ", beta)
        print("\n")

        print("Hip angle in degree: ", gamma * 180 / math.pi)
        print("Calf angle in degree: ", alpha * 180 / math.pi)
        print("Thigh angle in degree: ", beta * 180 / math.pi)

        

 