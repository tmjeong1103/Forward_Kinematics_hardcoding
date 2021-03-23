import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from visualization import *

def update(val):
    theta = [s1Angle.val, s2Angle.val, s3Angle.val, s4Angle.val, s5Angle.val, s6Angle.val, s7Angle.val,
             s8Angle.val, s9Angle.val, s10Angle.val, s11Angle.val, s12Angle.val, s13Angle.val]
    Link = [1, 1, 0, 0, 2, 2, 0, 1, 0, 0, 2, 2, 0]

    update_model = skeleton_HT(theta, Link)
    Result = ForwardKinematics(update_model)
    ax.cla()
    ax.set_xlim(left=-5, right=5)
    ax.set_ylim(bottom=-5, top=5)
    ax.set_zlim(bottom=-5, top=5)
    drawVector(ax, Result, pointEnable=True, lineWidth=2)

##valuable initialize

# define Base point(J0)
# We must define Base point but now is (0,0,0)
# Base_point = np.array([10,10,10])

# joint[i+1] = theta[i]
# Center of Hip joint valuable (index 0)
# Right Leg joint valuables (index 1~6, 6 joints)
# Left Leg joint valuables (index 7~12, 6 joints)
theta = np.zeros(13)

# Link[i+1] = theta[i]
# Center Link (index 0, Link1)
# Right Leg Links (index 1~6, 6 Links/ L3,L4,L7 = 0)
# Left Leg Links (index 7~12, 6 Links/ L9,L10,L13 = 0)
# Right Leg Link
Link = [1, 1, 0, 0, 2, 2, 0, 1, 0, 0, 2, 2, 0]

model = skeleton_HT(theta, Link)
Result = ForwardKinematics(model)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.set_xlim(left=-5, right=5)
ax.set_ylim(bottom=-5, top=5)
ax.set_zlim(bottom=-5, top=5)
plt.subplots_adjust(bottom=0.4, top=1.0)

drawVector(ax, Result, pointEnable=True, lineWidth=2)

th1Angle = plt.axes([0.1, 0.36, 0.3, 0.015])
th2Angle = plt.axes([0.1, 0.33, 0.3, 0.015])
th3Angle = plt.axes([0.1, 0.30, 0.3, 0.015])
th4Angle = plt.axes([0.1, 0.27, 0.3, 0.015])
th5Angle = plt.axes([0.1, 0.24, 0.3, 0.015])
th6Angle = plt.axes([0.1, 0.21, 0.3, 0.015])
th7Angle = plt.axes([0.1, 0.18, 0.3, 0.015])
th8Angle = plt.axes([0.1, 0.15, 0.3, 0.015])
th9Angle = plt.axes([0.1, 0.12, 0.3, 0.015])
th10Angle = plt.axes([0.1, 0.09, 0.3, 0.015])
th11Angle = plt.axes([0.1, 0.06, 0.3, 0.015])
th12Angle = plt.axes([0.1, 0.03, 0.3, 0.015])
th13Angle = plt.axes([0.1, 0.00, 0.3, 0.015])

s1Angle = Slider(th1Angle, r'$ \theta_1 $', -90.0, 90.0, valinit=0)
s2Angle = Slider(th2Angle, r'$ \theta_2 $', -90.0, 90.0, valinit=0)
s3Angle = Slider(th3Angle, r'$ \theta_3 $', -90.0, 90.0, valinit=0)
s4Angle = Slider(th4Angle, r'$ \theta_4 $', -90.0, 90.0, valinit=0)
s5Angle = Slider(th5Angle, r'$ \theta_5 $', -0.0, 180.0, valinit=0)
s6Angle = Slider(th6Angle, r'$ \theta_6 $', -90.0, 90.0, valinit=0)
s7Angle = Slider(th7Angle, r'$ \theta_7 $', -90.0, 90.0, valinit=0)
s8Angle = Slider(th8Angle, r'$ \theta_8 $', -90.0, 90.0, valinit=0)
s9Angle = Slider(th9Angle, r'$ \theta_9 $', -90.0, 90.0, valinit=0)
s10Angle = Slider(th10Angle, r'$ \theta_10 $', -90.0, 90.0, valinit=0)
s11Angle = Slider(th11Angle, r'$ \theta_11 $', -0.0, 180.0, valinit=0)
s12Angle = Slider(th12Angle, r'$ \theta_12 $', -90.0, 90.0, valinit=0)
s13Angle = Slider(th13Angle, r'$ \theta_13 $', -90.0, 90.0, valinit=0)

s1Angle.on_changed(update)
s2Angle.on_changed(update)
s3Angle.on_changed(update)
s4Angle.on_changed(update)
s5Angle.on_changed(update)
s6Angle.on_changed(update)
s7Angle.on_changed(update)
s8Angle.on_changed(update)
s9Angle.on_changed(update)
s10Angle.on_changed(update)
s11Angle.on_changed(update)
s12Angle.on_changed(update)
s13Angle.on_changed(update)

# ax.view_init(azim=-150, elev=30)
plt.show()