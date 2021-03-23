from transformation import *

# Skeleton Model : Human`s Leg
# Method : Homogeneous Transformation matrix

def skeleton_HT(theta, Link):
    ## Center of mass
    # Base(J0) to J1
    # Rotation : none, translation : none
    R0_1 = Rotation.E()
    P0_1 = Translation(0, 0, 0)
    H0_1 = HT_matrix(R0_1, P0_1)

    ## Center of Hip
    # J1 to J2
    # Rotation : Z1_axis (-90'<theta1<90', ' is degree.), translation : -Z1_axis direction L1
    R1_2 = Rotation.z(theta[0])
    P1_2 = Translation(0, 0, -Link[0])
    H1_2 = HT_matrix(R1_2, P1_2)

    #####------------Right Leg Modeling Start!-------------------------
    ## Pelvis
    # J2 to J3
    # Rotation : Y2_axis (-90'<theta2<90', ' is degree.), translation : -Y2_axis direction L2
    R2_3 = Rotation.y(theta[1])
    P2_3 = Translation(0, -Link[1], 0)
    H2_3 = HT_matrix(R2_3, P2_3)

    # J3 to J4
    # Rotation : Z3_axis (-90'<theta3<90', ' is degree.), translation : none, L3 = 0
    R3_4 = Rotation.z(theta[2])
    P3_4 = Translation(0, 0, 0)
    H3_4 = HT_matrix(R3_4, P3_4)

    # J4 to J5
    # Rotation : X4_axis (-90'<theta4<90', ' is degree.), translation : none, L4 = 0
    R4_5 = Rotation.x(theta[3])
    P4_5 = Translation(0, 0, 0)
    H4_5 = HT_matrix(R4_5, P4_5)

    ## Knee
    # J5 to J6
    # Rotation : Y5_axis (0'<theta5<180', ' is degree.), translation : -Z5_axis direction L5
    R5_6 = Rotation.y(theta[4])
    P5_6 = Translation(0, 0, -Link[4])
    H5_6 = HT_matrix(R5_6, P5_6)

    ## Ankle
    # J6 to J7
    # Rotation : Y6_axis (-90'<theta6<90', ' is degree.), translation : -Z6_axis direction L6
    R6_7 = Rotation.y(theta[5])
    P6_7 = Translation(0, 0, -Link[5])
    H6_7 = HT_matrix(R6_7, P6_7)

    # J7 to J8
    # Rotation : X7_axis (-90'<theta7<90', ' is degree.), translation : none, L7 = 0
    R7_8 = Rotation.x(theta[6])
    P7_8 = Translation(0, 0, 0)
    H7_8 = HT_matrix(R7_8, P7_8)

    #####-----------Right Leg Modeling End----------------------

    #####------------Left Leg Modeling Start!--------------------

    ## Pelvis
    # J2 to J9
    # Rotation : Y2_axis (-90'<theta8<90', ' is degree.), translation : +Y2_axis direction L8
    R2_9 = Rotation.y(theta[7])
    P2_9 = Translation(0, Link[7], 0)
    H2_9 = HT_matrix(R2_9, P2_9)

    ###kinematic chain rule
    # J8 to J2
    # inverse transformation
    H2_8 = H2_3.dot(H3_4).dot(H4_5).dot(H5_6).dot(H6_7).dot(H7_8)
    P2_8 = np.array([H2_8[0][3],
                     H2_8[1][3],
                     H2_8[2][3]])
    R2_8 = np.array([[H2_8[0][0], H2_8[0][1], H2_8[0][2], 0],
                     [H2_8[1][0], H2_8[1][1], H2_8[1][2], 0],
                     [H2_8[2][0], H2_8[2][1], H2_8[2][2], 0],
                     [0, 0, 0, 0]])
    R8_2 = np.transpose(R2_8)
    P8_2 = np.dot(-R8_2[:3, :3], P2_8)
    P8_2 = [[0, 0, 0, P8_2[0]],
            [0, 0, 0, P8_2[1]],
            [0, 0, 0, P8_2[2]],
            [0, 0, 0, 1]]
    H8_2 = HT_matrix(R8_2, P8_2)

    # J8 to J9
    H8_9 = H8_2.dot(H2_9)

    # J9 to J10
    # Rotation : Z9_axis (-90'<theta9<90', ' is degree.), translation : none, L9 = 0
    R9_10 = Rotation.z(theta[8])
    P9_10 = Translation(0, 0, 0)
    H9_10 = HT_matrix(R9_10, P9_10)

    # J10 to J11
    # Rotation : X10_axis (-90'<theta10<90', ' is degree.), translation : none, L10 = 0
    R10_11 = Rotation.x(theta[9])
    P10_11 = Translation(0, 0, 0)
    H10_11 = HT_matrix(R10_11, P10_11)

    ## Knee
    # J11 to J12
    # Rotation : Y11_axis (0'<theta11<180', ' is degree.), translation : -Z11_axis direction L11
    R11_12 = Rotation.y(theta[10])
    P11_12 = Translation(0, 0, -Link[10])
    H11_12 = HT_matrix(R11_12, P11_12)

    ## Ankle
    # J12 to J13
    # Rotation : Y12_axis (-90'<theta12<90', ' is degree.), translation : -Z12_axis direction L12
    R12_13 = Rotation.y(theta[11])
    P12_13 = Translation(0, 0, -Link[11])
    H12_13 = HT_matrix(R12_13, P12_13)

    # J13 to J14
    # Rotation : X13_axis (-90'<theta13<90', ' is degree.), translation : none, L13 = 0
    R13_14 = Rotation.x(theta[12])
    P13_14 = Translation(0, 0, 0)
    H13_14 = HT_matrix(R13_14, P13_14)

    #####------------Left Leg Modeling End--------------------

    H_relative = [H0_1, H1_2, H2_3, H3_4, H4_5, H5_6, H6_7, H7_8,
                  H8_9, H9_10, H10_11, H11_12, H12_13, H13_14]

    return H_relative