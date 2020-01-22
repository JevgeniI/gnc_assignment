#!/usr/bin/python
import matplotlib.pyplot as pl
import matplotlib.mlab as mlab
import numpy as np

import quadrotor as quad
import formation_displacement as form
import quadlog
import animation as ani
from filterpy.discrete_bayes import discrete_bayes
#from scipy.stats import norm
import scipy.stats as scs

animation = False
print_bias = False

# Quadrotor
m = 0.65 # Kg
l = 0.23 # m
Jxx = 7.5e-3 # Kg/m^2
Jyy = Jxx
Jzz = 1.3e-2
Jxy = 0
Jxz = 0
Jyz = 0
J = np.array([[Jxx, Jxy, Jxz],
              [Jxy, Jyy, Jyz],
              [Jxz, Jyz, Jzz]])
CDl = 9e-3
CDr = 9e-4
kt = 3.13e-5  # Ns^2
km = 7.5e-7   # Ns^2
kw = 1/0.18   # rad/s

# Initial conditions
att_0 = np.array([0.0, 0.0, 0.0])
pqr_0 = np.array([0.0, 0.0, 0.0])
xyz1_0 = np.array([1.0, 1.2, 0.0])
xyz2_0 = np.array([1.2, 2.0, 0.0])
xyz3_0 = np.array([-1.1, 2.6, 0.0])
v_ned_0 = np.array([0.0, 0.0, 0.0])
w_0 = np.array([0.0, 0.0, 0.0, 0.0])

# Setting quads
q1 = quad.quadrotor(1, m, l, J, CDl, CDr, kt, km, kw,
        att_0, pqr_0, xyz1_0, v_ned_0, w_0)

q2 = quad.quadrotor(2, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz2_0, v_ned_0, w_0)

q3 = quad.quadrotor(3, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz3_0, v_ned_0, w_0)

P_star = np.array([-5.0, 0.0, 5.0, 0, 0, 8.66])

fc = form.formation_displacement(P_star)

# Simulation parameters
tf = 1000
dt = 50e-3
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 200

# Initialize Kalman filters on Quadrotors
q1.kf_init(dt)
q2.kf_init(dt)
q3.kf_init(dt)

# Data log
q1_log = quadlog.quadlog(time)
q2_log = quadlog.quadlog(time)
q3_log = quadlog.quadlog(time)

# Plots
quadcolor = ['r', 'g', 'b']
pl.close("all")
pl.ion()

if animation == True:
    fig = pl.figure(0)
    axis3d = fig.add_subplot(111, projection='3d')
    pl.figure(0)

# Desired position and heading, biases
U = np.array([2.0, 0.5, 2.0, 0.5, 2.0, 0.5])
b = np.array([0.1, 0.1, -0.2, 0.15, 0.2, -0.05])
b_est = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
U_bc = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

U_b = U + b

alt_d = 8

X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
V = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

for t in time:

    # Simulation

    if t < 1000:

        U_b = U + b

        q1.set_v_2D_alt_lya(U_b[0:2], -alt_d)
        q2.set_v_2D_alt_lya(U_b[2:4], -alt_d)
        q3.set_v_2D_alt_lya(U_b[4:6], -alt_d)

        q1.kf()
        q2.kf()
        q3.kf()

        b_est = np.array([q1.fx.x[1, 0], q1.fy.x[1, 0], q2.fx.x[1, 0], q2.fy.x[1, 0], q3.fx.x[1, 0], q3.fy.x[1, 0]]) - U

    else:

        if print_bias == False:
            print b
            print b_est
            print_bias = True

        X_old = X
        X = np.append(q1.xyz[0:2], np.append(q2.xyz[0:2], q3.xyz[0:2]))

        V = (X - X_old)/dt

        U = fc.u_vel(X)

        U_b = U + b
        U_bc = U_b - b_est

        c = 0.2
        a_max = 1.0

        #a = np.array(
        #    [c / abs(U_bc[0] - q1.fx.x[1, 0]), c / abs(U_bc[1] - q1.fy.x[1, 0]), c / abs(U_bc[2] - q2.fx.x[1, 0]),
        #     c / abs(U_bc[3] - q2.fy.x[1, 0]),
        #     c / abs(U_bc[4] - q3.fx.x[1, 0]), c / abs(U_bc[5] - q3.fy.x[1, 0])])

        a = np.array(
            [c / abs(U_bc[0] - V[0]), c / abs(U_bc[1] - V[1]), c / abs(U_bc[2] - V[2]),
             c / abs(U_bc[3] - V[3]),
             c / abs(U_bc[4] - V[4]), c / abs(U_bc[5] - V[5])])

        if a[0] > a_max:
            a[0] = a_max
        if a[1] > a_max:
            a[1] = a_max
        if a[2] > a_max:
            a[2] = a_max
        if a[3] > a_max:
            a[3] = a_max
        if a[4] > a_max:
            a[4] = a_max
        if a[5] > a_max:
            a[5] = a_max

        #U_bc = np.multiply(a, U_bc)

        q1.set_v_2D_alt_lya(U_bc[0:2], -alt_d)
        q2.set_v_2D_alt_lya(U_bc[2:4], -alt_d)
        q3.set_v_2D_alt_lya(U_bc[4:6], -alt_d)

    q1.step(dt)
    q2.step(dt)
    q3.step(dt)

    # Animation
    if it % frames == 0:

        if animation == True:

            pl.figure(0)
            axis3d.cla()
            ani.draw3d(axis3d, q1.xyz, q1.Rot_bn(), quadcolor[0])
            ani.draw3d(axis3d, q2.xyz, q2.Rot_bn(), quadcolor[1])
            ani.draw3d(axis3d, q3.xyz, q3.Rot_bn(), quadcolor[2])
            axis3d.set_xlim(-5, 5)
            axis3d.set_ylim(-5, 5)
            axis3d.set_zlim(0, 10)
            axis3d.set_xlabel('South [m]')
            axis3d.set_ylabel('East [m]')
            axis3d.set_zlabel('Up [m]')
            axis3d.set_title("Time %.3f s" % t)
            pl.pause(0.001)
            pl.draw()

            if t >= 250:

                pl.figure(1)
                pl.clf()
                ani.draw2d(1, X, fc, quadcolor)
                ani.draw_edges(1, X, fc, -1)
                pl.xlabel('South [m]')
                pl.ylabel('West [m]')
                pl.title('2D Map')
                pl.grid()
                pl.pause(0.001)
                pl.draw()

    # Log
    q1_log.xyz_h[it, :] = q1.xyz
    q1_log.att_h[it, :] = q1.att
    q1_log.w_h[it, :] = q1.w
    q1_log.v_ned_h[it, :] = q1.v_ned
    q1_log.xi_g_h[it] = q1.xi_g
    q1_log.xi_CD_h[it] = q1.xi_CD
    q1_log.kfx[it] = q1.fx.x[:, 0]
    q1_log.kfy[it] = q1.fy.x[:, 0]
    q1_log.b[it] = b[0:2]
    q1_log.b_est[it] = b_est[0:2]

    q1_log.Px[it] = q1.fx.P
    q1_log.Py[it] = q1.fy.P

    q1_log.V[it] = V[0:2]



    ###
    q2_log.xyz_h[it, :] = q2.xyz
    q2_log.att_h[it, :] = q2.att
    q2_log.w_h[it, :] = q2.w
    q2_log.v_ned_h[it, :] = q2.v_ned
    q2_log.xi_g_h[it] = q2.xi_g
    q2_log.xi_CD_h[it] = q2.xi_CD
    q2_log.kfx[it] = q2.fx.x[:, 0]
    q2_log.kfy[it] = q2.fy.x[:, 0]
    q2_log.b[it] = b[2:4]
    q2_log.b_est[it] = b_est[2:4]

    q2_log.U[it] = U[2:4]
    q2_log.U_bc[it] = U_bc[2:4]

    q2_log.Px[it] = q2.fx.P
    q2_log.Py[it] = q2.fy.P

    q1_log.V[it] = V[2:4]


    ###
    q3_log.xyz_h[it, :] = q3.xyz
    q3_log.att_h[it, :] = q3.att
    q3_log.w_h[it, :] = q3.w
    q3_log.v_ned_h[it, :] = q3.v_ned
    q3_log.xi_g_h[it] = q3.xi_g
    q3_log.xi_CD_h[it] = q3.xi_CD
    q3_log.kfx[it] = q3.fx.x[:, 0]
    q3_log.kfy[it] = q3.fy.x[:, 0]
    q3_log.b[it] = b[4:6]
    q3_log.b_est[it] = b_est[4:6]

    q3_log.Px[it] = q3.fx.P
    q3_log.Py[it] = q3.fy.P

    q1_log.V[it] = V[4:6]


    it += 1

    # Stop if crash
    if q1.crashed == 1:
        break

### kalman filter plot ###
## x-direction
# pl.figure(3)
# pl.subplot(421)
# pl.plot(time, q1_log.xyz_h[:, 0], label="q1_X")
# pl.plot(time, q1_log.kfx[:, 0], label="q1_X_KF")
# pl.plot(time, q2_log.xyz_h[:, 0], label="q2_X")
# pl.plot(time, q2_log.kfx[:, 0], label="q2_X_KF")
# pl.plot(time, q3_log.xyz_h[:, 0], label="q3_X")
# pl.plot(time, q3_log.kfx[:, 0], label="q3_X_KF")
# #pl.xlabel("Time [s]")
# pl.ylabel("Position [m]")
# pl.grid()
# pl.legend()

# pl.subplot(423)
# pl.plot(time, q1_log.v_ned_h[:, 0], label="q1_V_x")
# pl.plot(time, q1_log.kfx[:, 1], label="q1_V_x_KF")
# pl.plot(time, q2_log.v_ned_h[:, 0], label="q2_V_x")
# pl.plot(time, q2_log.kfx[:, 1], label="q2_V_x_KF")
# pl.plot(time, q3_log.v_ned_h[:, 0], label="q3_V_x")
# pl.plot(time, q3_log.kfx[:, 1], label="q3_V_x_KF")
# #pl.xlabel("Time [s]")
# pl.ylabel("Velocity [m/s]")
# pl.grid()
# pl.legend()

# pl.subplot(425)
# #pl.plot(time, q1_log.v_ned_h[:, 0], label="q1_A_x")
# pl.plot(time, q1_log.kfx[:, 2], label="q1_A_x_KF")
# #pl.plot(time, q2_log.v_ned_h[:, 0], label="q2_A_x")
# pl.plot(time, q2_log.kfx[:, 2], label="q2_A_x_KF")
# #pl.plot(time, q3_log.v_ned_h[:, 0], label="q3_A_x")
# pl.plot(time, q3_log.kfx[:, 2], label="q3_A_x_KF")
# #pl.xlabel("Time [s]")
# pl.ylabel("Acceleration [m/s^2]")
# pl.grid()
# pl.legend()

# pl.subplot(427)
# pl.plot(time, q1_log.b[:,0], label="bx1")
# pl.plot(time, q1_log.b_est[:,0], label="bx1_KF")
# pl.plot(time, q2_log.b[:,0], label="bx2")
# pl.plot(time, q2_log.b_est[:,0], label="bx2_KF")
# pl.plot(time, q3_log.b[:,0], label="bx3")
# pl.plot(time, q3_log.b_est[:,0], label="bx3_KF")
# pl.xlabel("Time [s]")
# pl.ylabel("Biases [m/s]")
# pl.grid()
# pl.legend()

# pl.subplot(422)
# pl.plot(time, np.sqrt(q1_log.Px[:, 0, 0]), label="q1_X_P")
# pl.plot(time, np.sqrt(q2_log.Px[:, 0, 0]), label="q2_X_P")
# pl.plot(time, np.sqrt(q3_log.Px[:, 0, 0]), label="q3_X_P")
# #pl.xlabel("Time [s]")
# pl.ylabel("sigma [m]")
# pl.grid()
# pl.legend()

# pl.subplot(424)
# pl.plot(time, np.sqrt(q1_log.Px[:, 1, 1]), label="q1_V_P")
# pl.plot(time, np.sqrt(q2_log.Px[:, 1, 1]), label="q2_V_P")
# pl.plot(time, np.sqrt(q3_log.Px[:, 1, 1]), label="q3_V_P")
# #pl.xlabel("Time [s]")
# pl.ylabel("sigma [m/s]")
# pl.grid()
# pl.legend()

# pl.subplot(426)
# pl.plot(time, np.sqrt(q1_log.Px[:, 2, 2]), label="q1_A_P")
# pl.plot(time, np.sqrt(q2_log.Px[:, 2, 2]), label="q2_A_P")
# pl.plot(time, np.sqrt(q3_log.Px[:, 2, 2]), label="q3_A_P")
# #pl.xlabel("Time [s]")
# pl.ylabel("sigma [m/s^2]")
# pl.grid()
# pl.legend()

# pl.subplot(428)
# pl.plot(time, np.sqrt(q1_log.Px[:, 1, 1]), label="q1_bias_P")
# pl.plot(time, np.sqrt(q2_log.Px[:, 1, 1]), label="q2_bias_P")
# pl.plot(time, np.sqrt(q3_log.Px[:, 1, 1]), label="q3_bias_P")
# pl.xlabel("Time [s]")
# pl.ylabel("sigma [m/s]")
# pl.grid()
# pl.legend()

# ### kalman filter plot ###
# ## y-direction
# pl.figure(4)
# pl.subplot(421)
# pl.plot(time, q1_log.xyz_h[:, 1], label="q1_Y")
# pl.plot(time, q1_log.kfy[:, 0], label="q1_Y_KF")
# pl.plot(time, q2_log.xyz_h[:, 1], label="q2_Y")
# pl.plot(time, q2_log.kfy[:, 0], label="q2_Y_KF")
# pl.plot(time, q3_log.xyz_h[:, 1], label="q3_Y")
# pl.plot(time, q3_log.kfy[:, 0], label="q3_Y_KF")
# #pl.xlabel("Time [s]")
# pl.ylabel("Position [m]")
# pl.grid()
# pl.legend()

# pl.subplot(423)
# pl.plot(time, q1_log.v_ned_h[:, 1], label="q1_V_y")
# pl.plot(time, q1_log.kfy[:, 1], label="q1_V_y_KF")
# pl.plot(time, q2_log.v_ned_h[:, 1], label="q2_V_y")
# pl.plot(time, q2_log.kfy[:, 1], label="q2_V_y_KF")
# pl.plot(time, q3_log.v_ned_h[:, 1], label="_q3_V_y")
# pl.plot(time, q3_log.kfy[:, 1], label="q3_V_y_KF")
# pl.xlabel("Time [s]")
# pl.ylabel("Velocity [m/s]")
# pl.grid()
# pl.legend()

# pl.subplot(425)
# #pl.plot(time, q1_log.v_ned_h[:, 1], label="q1_A_y")
# pl.plot(time, q1_log.kfy[:, 2], label="q1_A_y_KF")
# #pl.plot(time, q2_log.v_ned_h[:, 1], label="q2_A_y")
# pl.plot(time, q2_log.kfy[:, 2], label="q2_A_y_KF")
# #pl.plot(time, q3_log.v_ned_h[:, 1], label="_q3_A_y")
# pl.plot(time, q3_log.kfy[:, 2], label="q3_A_y_KF")
# #pl.xlabel("Time [s]")
# pl.ylabel("Velocity [m/s]")
# pl.grid()
# pl.legend()

# pl.subplot(427)
# pl.plot(time, q1_log.b[:,1], label="by1")
# pl.plot(time, q1_log.b_est[:,1], label="by1_KF")
# pl.plot(time, q2_log.b[:,1], label="by2")
# pl.plot(time, q2_log.b_est[:,1], label="by2_KF")
# pl.plot(time, q3_log.b[:,1], label="by3")
# pl.plot(time, q3_log.b_est[:,1], label="by3_KF")
# pl.xlabel("Time [s]")
# pl.ylabel("Biases [m/s]")
# pl.grid()
# pl.legend()

# pl.subplot(422)
# pl.plot(time, np.sqrt(q1_log.Py[:, 0, 0]), label="q1_X_P")
# pl.plot(time, np.sqrt(q2_log.Py[:, 0, 0]), label="q2_X_P")
# pl.plot(time, np.sqrt(q3_log.Py[:, 0, 0]), label="q3_X_P")
# #pl.xlabel("Time [s]")
# pl.ylabel("sigma [m]")
# pl.grid()
# pl.legend()

# pl.subplot(424)
# pl.plot(time, np.sqrt(q1_log.Py[:, 1, 1]), label="q1_V_P")
# pl.plot(time, np.sqrt(q2_log.Py[:, 1, 1]), label="q2_V_P")
# pl.plot(time, np.sqrt(q3_log.Py[:, 1, 1]), label="q3_V_P")
# #pl.xlabel("Time [s]")
# pl.ylabel("sigma [m/s]")
# pl.grid()
# pl.legend()

# pl.subplot(426)
# pl.plot(time, np.sqrt(q1_log.Py[:, 2, 2]), label="q1_A_P")
# pl.plot(time, np.sqrt(q2_log.Py[:, 2, 2]), label="q2_A_P")
# pl.plot(time, np.sqrt(q3_log.Py[:, 2, 2]), label="q3_A_P")
# #pl.xlabel("Time [s]")
# pl.ylabel("sigma [m/s^2]")
# pl.grid()
# pl.legend()

# pl.subplot(428)
# pl.plot(time, np.sqrt(q1_log.Py[:, 1, 1]), label="q1_bias_P")
# pl.plot(time, np.sqrt(q2_log.Py[:, 1, 1]), label="q2_bias_P")
# pl.plot(time, np.sqrt(q3_log.Py[:, 1, 1]), label="q3_bias_P")
# #pl.xlabel("Time [s]")
# pl.ylabel("sigma [m/2]")
# pl.grid()
# pl.legend()


# pl.figure(1)
# pl.title("2D Position [m]")
# pl.plot(q1_log.xyz_h[:, 0], q1_log.xyz_h[:, 1], label="q1", color=quadcolor[0])
# pl.scatter(q1_log.xyz_h[-1, 0], q1_log.xyz_h[-1, 1], marker='+', color=quadcolor[0])
# pl.plot(q2_log.xyz_h[:, 0], q2_log.xyz_h[:, 1], label="q2", color=quadcolor[1])
# pl.scatter(q2_log.xyz_h[-1, 0], q2_log.xyz_h[-1, 1], marker='+', color=quadcolor[1])
# pl.plot(q3_log.xyz_h[:, 0], q3_log.xyz_h[:, 1], label="q3", color=quadcolor[2])
# pl.scatter(q3_log.xyz_h[-1, 0], q3_log.xyz_h[-1, 1], marker='+', color=quadcolor[2])
# pl.xlabel("East")
# pl.ylabel("South")
# pl.legend()

# ### Plots for report ##########################

# #pl.figure(5)
# #pl.plot(time, q1_log.v_ned_h[:, 0], label="q1_Vx")
# #pl.plot(time, q1_log.kfx[:, 1], label="q1_Vx_est")
# #pl.plot(time, q2_log.v_ned_h[:, 0], label="q2_Vx")
# #pl.plot(time, q2_log.kfx[:, 1], label="q2_Vx_est")
# #pl.plot(time, q3_log.v_ned_h[:, 0], label="q3_Vx")
# #pl.plot(time, q3_log.kfx[:, 1], label="q3_Vx_est")
# #pl.xlabel("Time [s]")
# #pl.ylabel("Velocity [m/s]")
# #pl.xlim(0, 1000)
# #pl.grid()
# #pl.legend()

# #pl.figure(6)
# #pl.plot(time, q1_log.v_ned_h[:, 1], label="q1_Vy")
# #pl.plot(time, q1_log.kfy[:, 1], label="q1_Vy_est")
# #pl.plot(time, q2_log.v_ned_h[:, 1], label="q2_Vy")
# #pl.plot(time, q2_log.kfy[:, 1], label="q2_Vy_est")
# #pl.plot(time, q3_log.v_ned_h[:, 1], label="q3_Vy")
# #pl.plot(time, q3_log.kfy[:, 1], label="q3_Vy_est")
# #pl.xlabel("Time [s]")
# #pl.ylabel("Velocity [m/s]")
# #pl.xlim(0, 1000)
# #pl.grid()
# #pl.legend()

# #pl.figure(7)
# #pl.plot(time, q1_log.b[:,0], label="q1_bx")
# #pl.plot(time, q1_log.b_est[:,0], label="q1_bx_est")
# #pl.plot(time, q2_log.b[:,0], label="q2_bx")
# #pl.plot(time, q2_log.b_est[:,0], label="q2_bx_est")
# #pl.plot(time, q3_log.b[:,0], label="q3_bx")
# #pl.plot(time, q3_log.b_est[:,0], label="q3_bx_est")
# #pl.xlabel("Time [s]")
# #pl.ylabel("Velocity command bias [m/s]")
# #pl.xlim(0, 1000)
# #pl.grid()
# #pl.legend()

# #pl.figure(8)
# #pl.plot(time, q1_log.b[:,1], label="q1_by")
# #pl.plot(time, q1_log.b_est[:,1], label="q1_by_est")
# #pl.plot(time, q2_log.b[:,1], label="q2_by")
# #pl.plot(time, q2_log.b_est[:,1], label="q2_by_est")
# #pl.plot(time, q3_log.b[:,1], label="q3_by")
# #pl.plot(time, q3_log.b_est[:,1], label="q3_by_est")
# #pl.xlabel("Time [s]")
# #pl.ylabel("Velocity command bias [m/s]")
# #pl.xlim(0, 1000)
# #pl.grid()
# #pl.legend()

# #pl.figure(9)
# #pl.plot(time, np.sqrt(q1_log.Px[:, 1, 1]), label="q1_$\sigma$_bx_est")
# #pl.plot(time, np.sqrt(q2_log.Px[:, 1, 1]), label="q2_$\sigma$_bx_est")
# #pl.plot(time, np.sqrt(q3_log.Px[:, 1, 1]), label="q3_$\sigma$_bx_est")
# #pl.xlabel("Time [s]")
# #pl.ylabel("sigma [m/s]")
# #pl.xlim(0, 1000)
# #pl.grid()
# #pl.legend()

# #pl.figure(10)
# #pl.plot(time, np.sqrt(q1_log.Py[:, 1, 1]), label="q1_$\sigma$_by_est")
# #pl.plot(time, np.sqrt(q2_log.Py[:, 1, 1]), label="q2_$\sigma$_by_est")
# #pl.plot(time, np.sqrt(q3_log.Py[:, 1, 1]), label="q3_$\sigma$_by_est")
# #pl.xlabel("Time [s]")
# #pl.ylabel("sigma [m/s]")
# #pl.xlim(0, 1000)
# #pl.grid()
# #pl.legend()

# xpl = 15
# xlimits0 = np.linspace(-xpl, xpl, 100000)

# pl.figure(11)
# pgauss0 = mlab.normpdf(xlimits0, q2_log.b_est[0,0], np.sqrt(q2_log.Px[0, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss0), label="q2_$\sigma$_bx_est init")
# pgauss100 = mlab.normpdf(xlimits0, q2_log.b_est[100,0], np.sqrt(q2_log.Px[100, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss100), label="q2_$\sigma$_bx_est 100 it")
# pgauss200 = mlab.normpdf(xlimits0, q2_log.b_est[200,0], np.sqrt(q2_log.Px[200, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss200), label="q2_$\sigma$_bx_est 200 it")
# pgauss300 = mlab.normpdf(xlimits0, q2_log.b_est[300,0], np.sqrt(q2_log.Px[300, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss300), label="q2_$\sigma$_bx_est 300 it")
# pgauss1000 = mlab.normpdf(xlimits0, q2_log.b_est[1000,0], np.sqrt(q2_log.Px[1000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss1000), label="q2_$\sigma$_bx_est 1000 it")
# pgauss2000 = mlab.normpdf(xlimits0, q2_log.b_est[2000,0], np.sqrt(q2_log.Px[2000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss2000), label="q2_$\sigma$_bx_est 2000 it")
# pgauss10000 = mlab.normpdf(xlimits0, q2_log.b_est[10000,0], np.sqrt(q2_log.Px[10000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss10000), label="q2_$\sigma$_bx_est 10000 it")
# pl.xlabel("[m/s]")
# pl.xlim(-5,1)
# pl.grid()
# pl.legend()



# pl.figure(21)
# pgauss0 = scs.norm.pdf(xlimits0, q2_log.b_est[0,0], np.sqrt(q2_log.Px[0, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss0), label="q2_$\sigma$_bx_est init")
# pgauss100 = scs.norm.pdf(xlimits0, q2_log.b_est[100,0], np.sqrt(q2_log.Px[100, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss100)/6, label="q2_$\sigma$_bx_est 100 it")
# pgauss200 = scs.norm.pdf(xlimits0, q2_log.b_est[200,0], np.sqrt(q2_log.Px[200, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss200)/10, label="q2_$\sigma$_bx_est 200 it")
# pgauss300 = scs.norm.pdf(xlimits0, q2_log.b_est[300,0], np.sqrt(q2_log.Px[300, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss300)/16, label="q2_$\sigma$_bx_est 300 it")
# pgauss1000 = scs.norm.pdf(xlimits0, q2_log.b_est[1000,0], np.sqrt(q2_log.Px[1000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss1000)/80, label="q2_$\sigma$_bx_est 1000 it")
# pgauss2000 = scs.norm.pdf(xlimits0, q2_log.b_est[2000,0], np.sqrt(q2_log.Px[2000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss2000)/160, label="q2_$\sigma$_bx_est 2000 it")
# pgauss10000 = scs.norm.pdf(xlimits0, q2_log.b_est[10000,0], np.sqrt(q2_log.Px[10000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss10000)/900, label="q2_$\sigma$_bx_est 10000 it")
# pl.xlabel("[m/s]")
# pl.xlim(-5,1)
# pl.grid()
# pl.legend()

# pl.figure(12)
# pgauss0 = mlab.normpdf(xlimits0, q2_log.b_est[0,1], np.sqrt(q2_log.Py[0, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss0), label="q2_$\sigma$_by_est init")
# pgauss100 = mlab.normpdf(xlimits0, q2_log.b_est[100,1], np.sqrt(q2_log.Py[100, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss100), label="q2_$\sigma$_by_est 100 it")
# pgauss200 = mlab.normpdf(xlimits0, q2_log.b_est[200,1], np.sqrt(q2_log.Py[200, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss200), label="q2_$\sigma$_by_est 200 it")
# pgauss300 = mlab.normpdf(xlimits0, q2_log.b_est[300,1], np.sqrt(q2_log.Py[300, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss300), label="q2_$\sigma$_by_est 300 it")
# pgauss1000 = mlab.normpdf(xlimits0, q2_log.b_est[1000,1], np.sqrt(q2_log.Py[1000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss1000), label="q2_$\sigma$_by_est 1000 it")
# pgauss2000 = mlab.normpdf(xlimits0, q2_log.b_est[2000,1], np.sqrt(q2_log.Py[2000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss2000), label="q2_$\sigma$_by_est 2000 it")
# pgauss10000 = mlab.normpdf(xlimits0, q2_log.b_est[10000,1], np.sqrt(q2_log.Py[10000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss10000), label="q2_$\sigma$_by_est 10000 it")
# pl.xlabel("[m/s]")
# pl.xlim(-3,2)
# pl.grid()
# pl.legend()

# pl.figure(22)
# pgauss0 = mlab.normpdf(xlimits0, q2_log.b_est[0,1], np.sqrt(q2_log.Py[0, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss0), label="q2_$\sigma$_by_est init")
# pgauss100 = mlab.normpdf(xlimits0, q2_log.b_est[100,1], np.sqrt(q2_log.Py[100, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss100)/6, label="q2_$\sigma$_by_est 100 it")
# pgauss200 = mlab.normpdf(xlimits0, q2_log.b_est[200,1], np.sqrt(q2_log.Py[200, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss200)/10, label="q2_$\sigma$_by_est 200 it")
# pgauss300 = mlab.normpdf(xlimits0, q2_log.b_est[300,1], np.sqrt(q2_log.Py[300, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss300)/16, label="q2_$\sigma$_by_est 300 it")
# pgauss1000 = mlab.normpdf(xlimits0, q2_log.b_est[1000,1], np.sqrt(q2_log.Py[1000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss1000)/80, label="q2_$\sigma$_by_est 1000 it")
# pgauss2000 = mlab.normpdf(xlimits0, q2_log.b_est[2000,1], np.sqrt(q2_log.Py[2000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss2000)/160, label="q2_$\sigma$_by_est 2000 it")
# pgauss10000 = mlab.normpdf(xlimits0, q2_log.b_est[10000,1], np.sqrt(q2_log.Py[10000, 1, 1]))
# pl.plot(xlimits0, discrete_bayes.normalize(pgauss10000)/900, label="q2_$\sigma$_by_est 10000 it")
# pl.xlabel("[m/s]")
# pl.xlim(-3,2)
# pl.grid()
# pl.legend()

# pl.figure(13)
# pl.plot(time, q1_log.xyz_h[:, 0], label="q1_X")
# pl.plot(time, q2_log.xyz_h[:, 0], label="q2_X")
# pl.plot(time, q3_log.xyz_h[:, 0], label="q3_X")
# pl.xlim(left = 800, right = 2000)
# pl.ylim(bottom = 1250)
# pl.xlabel("Time [s]")
# pl.ylabel("Position [m]")
# pl.grid()
# pl.legend()

# pl.figure(14)
# pl.plot(time, q1_log.xyz_h[:, 1], label="q1_Y")
# pl.plot(time, q2_log.xyz_h[:, 1], label="q2_Y")
# pl.plot(time, q3_log.xyz_h[:, 1], label="q3_Y")
# pl.xlim(left = 800, right = 2000)
# pl.ylim(bottom = 300)
# pl.xlabel("Time [s]")
# pl.ylabel("Position [m]")
# pl.grid()
# pl.legend()

# pl.figure(15)
# pl.plot(time, q2_log.U_bc[:, 0], label="q2_U_x")
# pl.plot(time, q2_log.v_ned_h[:, 0], label="q2_V_x")
# pl.xlabel("Time [s]")
# pl.ylabel("input/actual velocity [m/s]")
# pl.xlim(left = 800, right = 2000)
# pl.grid()
# pl.legend()

# pl.figure(16)
# pl.plot(time, q2_log.U_bc[:, 1], label="q2_U_y")
# pl.plot(time, q2_log.v_ned_h[:, 1], label="q2_V_y")
# pl.xlabel("Time [s]")
# pl.ylabel("input/actual velocity [m/s]")
# pl.xlim(left = 800, right = 2000)
# pl.grid()
# pl.legend()





pl.pause(0)
