import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import parameters.simulation_parameters as P
import numpy as np
import matplotlib
matplotlib.use('TkAgg')


class Visualizer:
    def __init__(self):
        # FIGURE INITIALIZATION
        self.fig = None
        self.plane = None
        self.force = None
        self.moment = None
        self.ppp = None
        self.uvw = None
        self.ang = None
        self.pqr = None
        self.verts = None
        self.defP = None
        self.thr = None
        self.g_lim = P.g_lim

        # TIME INITIALIZATION
        self.t = 1
        self.ts = P.ts_simulation

        # ARRAY INITIALIZATION
        self.t_array = np.array([])
        self.pppData = np.array([])
        self.angData = np.array([])
        self.uvwData = np.array([])
        self.pqrData = np.array([])
        self.fData = np.array([])
        self.lmnData = np.array([])
        self.defLData = np.array([])
        self.vaData = np.array([])
        self.hRef = np.array([])
        self.chRef = np.array([])

    def initialization(self):
        self.fig = plt.figure()
        self.plane = self.fig.add_subplot(1, 1, 1, projection='3d')

        # PLANE SUBPLOT
        self.plane.set_xlabel(r'X (m)')
        self.plane.set_ylabel(r'Y (m)')
        self.plane.set_zlabel(r'Z (m)')
        self.plane.set_xlim([-self.g_lim, self.g_lim])
        self.plane.set_ylim([-self.g_lim, self.g_lim])
        self.plane.set_zlim([-self.g_lim, self.g_lim])

        plt.pause(P.pause)

    def update(self, verts, verts2, data, f, lmn, defLData, ref, t):
        self.verts = verts
        self.verts2 = verts2

        if self.t == 1:
            self.t_array = np.array([t])
            self.defLData = defLData
            self.fData = f
            self.lmnData = lmn
            self.pppData = np.array([[data[0, 0]], [data[1, 0]], [data[2, 0]]])
            self.uvwData = np.array([[data[3, 0]], [data[4, 0]], [data[5, 0]]])
            self.angData = np.array([[data[6, 0]], [data[7, 0]], [data[8, 0]]])
            self.pqrData = np.array([[data[9, 0]], [data[10, 0]], [data[11, 0]]])
            self.vaData = np.array([[ref[0]], [ref[1]]])
            self.hRef = np.array([ref[2]])
            self.chRef = np.array([ref[3]])
        else:
            self.t_array = np.append(self.t_array, t)
            self.defLData = np.append(self.defLData, defLData, 1)
            self.fData = np.append(self.fData, f, 1)
            self.lmnData = np.append(self.lmnData, lmn, 1)
            self.pppData = np.append(self.pppData, np.array([[data[0, 0]], [data[1, 0]], [data[2, 0]]]), 1)
            self.uvwData = np.append(self.uvwData, np.array([[data[3, 0]], [data[4, 0]], [data[5, 0]]]), 1)
            self.angData = np.append(self.angData, np.array([[data[6, 0]], [data[7, 0]], [data[8, 0]]]), 1)
            self.pqrData = np.append(self.pqrData, np.array([[data[9, 0]], [data[10, 0]], [data[11, 0]]]), 1)
            self.vaData = np.append(self.vaData, np.array([[ref[0]], [ref[1]]]), 1)
            self.hRef = np.append(self.hRef, ref[2])
            self.chRef = np.append(self.chRef, ref[3])

        # PLANE SUBPLOT
        self.plane.cla()
        self.plane.set_xlabel(r'X (m)')
        self.plane.set_ylabel(r'Y (m)')
        self.plane.set_zlabel(r'Z (m)')
        faces = np.arange(0, len(self.verts)).reshape(-1, 3)
        poly3d = Poly3DCollection([self.verts[polygon] for polygon in faces], facecolors='red', linewidths=.25,
                                  alpha=0.5)
        self.plane.add_collection3d(poly3d)
        faces = np.arange(0, len(self.verts2)).reshape(-1, 3)
        poly3d = Poly3DCollection([self.verts2[polygon] for polygon in faces], facecolors='red', linewidths=.25,
                                  alpha=0.5)
        self.plane.add_collection3d(poly3d)
        self.plane.set_xlim([-self.g_lim + data[0, 0], self.g_lim + data[0, 0]])
        self.plane.set_ylim([-self.g_lim + data[1, 0], self.g_lim + data[1, 0]])
        self.plane.set_zlim([-self.g_lim - data[2, 0], self.g_lim - data[2, 0]])

        plt.pause(self.ts)
        self.t += 1