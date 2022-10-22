# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # the following only works for at most one track and one measurement
        tracks = len(track_list)
        measurements = len(meas_list)
        self.unassigned_tracks = list(range(tracks))
        self.unassigned_meas = list(range(measurements))
        self.association_matrix = np.zeros([tracks, measurements])
        self.association_matrix = np.asmatrix(self.association_matrix)
        for i,track in enumerate(track_list):
            for j,meas in enumerate(meas_list):
                MH_distance = self.MHD(track, meas, KF)
                if self.gating(MH_distance, meas.sensor):
                    self.association_matrix[i,j] = MH_distance
                else:
                    self.association_matrix[i,j] = np.inf


        '''
        self.association_matrix = np.matrix([]) # reset matrix
        self.unassigned_tracks = [] # reset lists
        self.unassigned_meas = []

        if len(meas_list) > 0:
            self.unassigned_meas = [0]
        if len(track_list) > 0:
            self.unassigned_tracks = [0]
        if len(meas_list) > 0 and len(track_list) > 0:
            self.association_matrix = np.matrix([[0]])
        '''
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        # the following only works for at most one track and one measurement
        #update_track = 0
        #update_meas = 0
        mat = self.association_matrix
        if np.min(mat) == np.inf:
            return np.nan, np.nan

        indices = np.unravel_index(np.argmin(mat, axis=None), mat.shape)
        track_index = indices[0]
        meas_index = indices[1]
        new_association_mat = np.delete(np.delete(mat, indices[0], axis=0), indices[1], axis=1)
        self.association_matrix = new_association_mat
        update_track = self.unassigned_tracks[track_index]
        update_meas = self.unassigned_meas[meas_index]
        
        # remove from list
        self.unassigned_tracks.remove(update_track) 
        self.unassigned_meas.remove(update_meas)

            
        ############
        # END student code
        ############ 
        return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        if sensor.name == "lidar":
            dof = 2
        elif sensor.name == "camera":
            dof = 1
        limit = chi2.ppf(params.gating_threshold, df = dof)
        if MHD < limit:
            return True
        else:
            return False
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
        H = meas.sensor.get_H(track.x)
        S = H*track.P*H.transpose() + meas.R
        gamma = KF.gamma(track, meas)
        MHD = gamma.transpose()*np.linalg.inv(S)*gamma
        return MHD

        #         gamma = KF.gamma(track, meas)
        #         return gamma.transpose() * np.linavg.inv(KF.S) * gamma

        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
        # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)