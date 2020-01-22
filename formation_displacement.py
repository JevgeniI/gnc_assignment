#!/usr/bin/python
import numpy as np

class formation_displacement:
    def __init__(self, p_star):
        self.p_star = p_star
        self.U = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def u_vel(self, p):

        e1x = (p[0]-p[2])-(self.p_star[0]-self.p_star[2])
        e1y = (p[1]-p[3])-(self.p_star[1]-self.p_star[3])
        e2x = (p[2]-p[4])-(self.p_star[2]-self.p_star[4])
        e2y = (p[3]-p[5])-(self.p_star[3]-self.p_star[5])

        self.U[0] = -e1x
        self.U[1] = -e1y
        self.U[2] = e1x-e2x
        self.U[3] = e1y-e2y
        self.U[4] = e2x
        self.U[5] = e2y

        self.max = 1.0
        self.min = -self.max

        if self.U[0] > self.max:
            self.U[0] = self.max
        if self.U[1] > self.max:
            self.U[1] = self.max
        if self.U[2] > self.max:
            self.U[2] = self.max
        if self.U[3] > self.max:
            self.U[3] = self.max
        if self.U[4] > self.max:
            self.U[4] = self.max
        if self.U[5] > self.max:
           self.U[5] = self.max


        if self.U[0] < self.min:
            self.U[0] = self.min
        if self.U[1] < self.min:
            self.U[1] = self.min
        if self.U[2] < self.min:
            self.U[2] = self.min
        if self.U[3] < self.min:
            self.U[3] = self.min
        if self.U[4] < self.min:
            self.U[4] = self.min
        if self.U[5] < self.min:
           self.U[5] = self.min

        return self.U

