

import math 


def calLinkLength(base_coor, hip_joint_coor, thigh_joint_coor, calf_joint_coor, foot_coor):

    #coor format: x, y, z

    thighToHip_coor = relativeCoorCalculation(thigh_joint_coor, hip_joint_coor)

    calfToThigh_coor = relativeCoorCalculation(calf_joint_coor, thigh_joint_coor)

    footToCalf_coor = relativeCoorCalculation(foot_coor, calf_joint_coor)

    hipTo_base = math.sqrt(base_coor[0]**2 + base_coor[1]**2 + base_coor[2]**2) #distance between the hip joint and the base

    thighTo_hip = math.sqrt(thighToHip_coor[0]**2 + thighToHip_coor[1]**2 + thighToHip_coor[2]**2) #distance between the thigh joint and the hip joint

    #Thight length: between thight joint and carlf joint
    calfTo_thigh = math.sqrt(
   





    return hipTo_base, thighTo_hip, calfTo_thigh, footTo_calf

def inverseKinematic(base_coor, hip_joint_coor, thigh_joint_coor, calf_joint_coor):
    #Calculate the joint angles of the hip, thigh and calf joints base on the base coor (mainly the height)

    #coor format: x, y, z

    # Add your code here
    pass


def relativeCoorCalculation(coor1, coor2):

    relative_coor = [coor2[0] + coor1[0], coor2[1] + coor1[1], coor2[2] + coor1[2]]

    return relative_coor



if __name__ == "__main__":

    base_coor = [0., 0., 0.]
    hip_joint_coor = [0.1881, -0.04675, 0.]
    thigh_joint_coor = [0., -0.08, 0.]
    calf_joint_coor = [0., 0., -0.213]
    foot_coor = [0., 0., -0.213]

    # footToCalfCoor = relativeCoorCalculation(calf_joint_coor, foot_coor)

    # print("footToCalfCoor: ", footToCalfCoor)



    #hip jointcoor: 0.1881, -0.04675, 0.
    #thigh joint coor within the base coor: 0.1881, -0.12675, 0.
    #calf joint coor within the base joint coor: 0.1881, -0.12675, -0.213
    #foot joint coor within the based joint coor: 0.1881, -0.12675, -0.426 


    hipTo_base, thighTo_hip, calfTo_thigh, footTo_calf = calLinkLength(base_coor, hip_joint_coor, thigh_joint_coor, calf_joint_coor, foot_coor)

    print("hipTo_base: ", hipTo_base)
    print("thighTo_hip: ", thighTo_hip)
    print("calfTo_thigh: ", calfTo_thigh)
    print("footTo_calf: ", footTo_calf)

