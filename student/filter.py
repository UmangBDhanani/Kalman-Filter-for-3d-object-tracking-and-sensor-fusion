# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state
        self.dt = params.dt
        self.q = params.q

    def F(self):
        ############
        # system matrix F

        dt = self.dt
        F = np.identity(6)
        F[0, 3], F[1, 4], F[2, 5] = dt, dt, dt
        return F

        ############ 

    def Q(self):
        ############
        ##  process noise covariance Q

        #Q = np.zeros((6, 6)).astype(np.uint8)
        #mat_Q = np.matrix(Q)

        q = self.q
        dt = self.dt
        q1 = (q/3)*dt**3
        q2 = (q/2)*dt**2
        q3 = q*dt
        mat_Q = np.matrix([[q1,0,0,q2,0,0],
                           [0,q1,0,0,q2,0],
                           [0,0,q1,0,0,q2],
                           [q2,0,0,q3,0,0],
                           [0,q2,0,0,q3,0],
                           [0,0,q2,0,0,q3]])

        return mat_Q

        ############ 

    def predict(self, track):
        ############
        ## predict state x and estimation error covariance P to next timestep, save x and P in track

        x = track.x
        F = self.F()
        P = track.P
        Q = self.Q()
        x = F * x
        P = F * P * F.transpose() + Q
        track.set_x(x)
        track.set_P(P)

        ############ 

    def update(self, track, meas):
        ############
        ## update state x and covariance P with associated measurement, save x and P in track

        x = track.x
        H = meas.sensor.get_H(x)
        P = track.P
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)

        K = P * H.transpose() * np.linalg.inv(S)
        x = x + K * gamma
        I = np.identity(self.dim_state)
        P = (I - K * H) * P
        track.set_x(x)
        track.set_P(P)

        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        ## residual gamma

        x = track.x
        z = meas.z
        h_x = meas.sensor.get_hx(x)

        return z - h_x

        ############ 

    def S(self, track, meas, H):
        ############
        ## covariance of residual S

        R = meas.R
        P = track.P

        return H * P * H.transpose() + R

        ############ 