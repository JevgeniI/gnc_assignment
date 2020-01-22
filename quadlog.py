#!/usr/bin/python
import numpy as np
import quadrotor as quad

class quadlog:
    def __init__(self, time):
        self.att_h = np.zeros((time.size, 3))
        self.pqr_h = np.zeros((time.size, 3))
        self.xyz_h = np.zeros((time.size, 3))
        self.v_ned_h = np.zeros((time.size, 3))
        self.w_h = np.zeros((time.size, 4))
        self.xi_g_h = np.zeros(time.size)
        self.xi_CD_h = np.zeros(time.size)

        self.kfx = np.zeros((time.size, 3))
        self.kfy = np.zeros((time.size, 3))
        self.b = np.zeros((time.size, 2))
        self.b_est = np.zeros((time.size, 2))

        self.U = np.zeros((time.size, 2))
        self.U_bc = np.zeros((time.size, 2))

        self.Px = np.zeros((time.size, 3, 3))
        self.Py = np.zeros((time.size, 3, 3))

        self.V = np.zeros((time.size, 2))



