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
        self.plane = self.fig.add_subplot(1, 3, 1, projection='3d')
        self.thr = self.fig.add_subplot(1, 12, 5)
        self.defP = self.fig.add_subplot(3, 4, 3)
        self.force = self.fig.add_subplot(3, 4, 7)
        self.moment = self.fig.add_subplot(3, 4, 11)
        self.ppp = self.fig.add_subplot(4, 4, 4)
        self.uvw = self.fig.add_subplot(4, 4, 8)
        self.ang = self.fig.add_subplot(4, 4, 12)
        self.pqr = self.fig.add_subplot(4, 4, 16)
        self.fig.subplots_adjust(right=.975, top=.95, left=.025, bottom=.05, hspace=.5)

        # PLANE SUBPLOT
        self.plane.set_xlabel(r'X (m)')
        self.plane.set_ylabel(r'Y (m)')
        self.plane.set_zlabel(r'Z (m)')
        self.plane.set_xlim([-self.g_lim, self.g_lim])
        self.plane.set_ylim([-self.g_lim, self.g_lim])
        self.plane.set_zlim([-self.g_lim, self.g_lim])

        # THROTTLE SUBPLOT
        self.thr.set_title('Throttle')
        self.thr.set_ylabel('% / 100')
        self.thr.set_ylim(0, 1)
        self.thr.set_xticks([])

        # DEFLECTION SUBPLOT
        self.defP.set_title('Deflection')
        self.defP.set_xlabel('Time (s)')
        self.defP.set_ylabel('Deflection (Deg)')
        self.defP.grid(True)

        # FORCE SUBPLOT
        self.force.set_title('Force')
        self.force.set_xlabel('Time (s)')
        self.force.set_ylabel('Newton (N)')
        self.force.grid(True)

        # MOMENT SUBPLOT
        self.moment.set_title('Moment')
        self.moment.set_xlabel('Time (s)')
        self.moment.set_ylabel('Newton-Meter (Nm)')
        self.moment.grid(True)

        # PPP SUBPLOT
        self.ppp.set_title(r'$p_{n}, p_{e}, p_{d}$')
        self.ppp.set_xlabel('Time (s)')
        self.ppp.set_ylabel('Position (m)')
        self.ppp.grid(True)

        # UVW SUBPLOT
        self.uvw.set_title(r'$u, v, w$')
        self.uvw.set_xlabel('Time (s)')
        self.uvw.set_ylabel('Ground Velocity (m/s)')
        self.uvw.grid(True)

        # ANG SUBPLOT
        self.ang.set_title(r'$\phi, \theta, \psi$')
        self.ang.set_xlabel('Time (s)')
        self.ang.set_ylabel('Angles (rad)')
        self.ang.grid(True)

        # PQR SUBPLOT
        self.pqr.set_title(r'$p, q, r$')
        self.pqr.set_xlabel('Time (s)')
        self.pqr.set_ylabel('Body Angles (rad)')
        self.pqr.grid(True)

        plt.pause(P.pause)

    def update(self, verts, data, f, lmn, defLData, ref, t):
        self.verts = verts

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
        self.plane.set_xlim([-self.g_lim + data[0, 0], self.g_lim + data[0, 0]])
        self.plane.set_ylim([-self.g_lim + data[1, 0], self.g_lim + data[1, 0]])
        self.plane.set_zlim([-self.g_lim + data[2, 0], self.g_lim + data[2, 0]])

        # THROTTLE SUBPLOT
        self.thr.cla()
        self.thr.set_title('Throttle')
        self.thr.set_ylabel('% / 100')
        self.thr.bar(0, self.defLData[3, -1], width=0.5, color='r')
        self.thr.set_ylim(0, 1)
        self.thr.set_xticks([])

        # DEFLECTION SUBPLOT
        self.defP.cla()
        self.defP.set_title('Deflection')
        self.defP.set_xlabel('Time (s)')
        self.defP.set_ylabel('Deflection (rad)')
        self.defP.plot(self.t_array, self.defLData[1], label='a', color='r')
        self.defP.plot(self.t_array, self.defLData[0], label='e', color='g')
        self.defP.plot(self.t_array, self.defLData[2], label='r', color='b')
        self.defP.legend(loc='upper right')
        self.defP.grid(True)

        # FORCE SUBPLOT
        self.force.cla()
        self.force.set_title('Force')
        self.force.set_xlabel('Time (s)')
        self.force.set_ylabel('Newton (N)')
        self.force.plot(self.t_array, self.fData[0], label=r'$f_{x}$', color='r')
        self.force.plot(self.t_array, self.fData[1], label=r'$f_{y}$', color='g')
        self.force.plot(self.t_array, self.fData[2], label=r'$f_{z}$', color='b')
        self.force.legend(loc='upper right')
        self.force.grid(True)

        # MOMENT SUBPLOT
        self.moment.cla()
        self.moment.set_title('Moment')
        self.moment.set_xlabel('Time (s)')
        self.moment.set_ylabel('Newton-Meter (Nm)')
        self.moment.plot(self.t_array, self.lmnData[0], label=r'l', color='r')
        self.moment.plot(self.t_array, self.lmnData[1], label=r'm', color='g')
        self.moment.plot(self.t_array, self.lmnData[2], label=r'n', color='b')
        self.moment.legend(loc='upper right')
        self.moment.grid(True)

        # PPP SUBPLOT
        self.ppp.cla()
        self.ppp.set_title(r'$p_{n}, p_{e}, p_{d}$')
        self.ppp.set_xlabel('Time (s)')
        self.ppp.set_ylabel('Position (m)')
        self.ppp.plot(self.t_array, self.pppData[0], label=r'$p_{n}$', color='r')
        self.ppp.plot(self.t_array, self.pppData[1], label=r'$p_{e}$', color='g')
        self.ppp.plot(self.t_array, self.pppData[2], label=r'$p_{d}$', color='b')
        self.ppp.plot(self.t_array, self.hRef, label=r'$p_{d}$ ref', color='m', alpha=0.25, linestyle='--')
        self.ppp.legend(loc='upper right')
        self.ppp.grid(True)

        # UVW SUBPLOT
        self.uvw.cla()
        self.uvw.set_title(r'$u, v, w$')
        self.uvw.set_xlabel('Time (s)')
        self.uvw.set_ylabel('Ground Velocity (m/s)')
        self.uvw.plot(self.t_array, self.uvwData[0], label=r'u', color='r')
        self.uvw.plot(self.t_array, self.uvwData[1], label=r'v', color='g')
        self.uvw.plot(self.t_array, self.uvwData[2], label=r'w', color='b')
        self.uvw.plot(self.t_array, self.vaData[0], label=r'Va', color='c')
        self.uvw.plot(self.t_array, self.vaData[1], label=r'Va ref', color='m', alpha=0.25, linestyle='--')
        self.uvw.legend(loc='upper right')
        self.uvw.grid(True)

        # ANG SUBPLOT
        self.ang.cla()
        self.ang.set_title(r'$\phi, \theta, \psi$')
        self.ang.set_xlabel('Time (s)')
        self.ang.set_ylabel('Angles (rad)')
        self.ang.plot(self.t_array, self.angData[0], label=r'$\phi$', color='r')
        self.ang.plot(self.t_array, self.angData[1], label=r'$\theta$', color='g')
        self.ang.plot(self.t_array, self.angData[2], label=r'$\psi$', color='b')
        self.ang.plot(self.t_array, self.chRef, label=r'$\chi$ ref', color='m', alpha=0.25, linestyle='--')
        self.ang.legend(loc='upper right')
        self.ang.grid(True)

        # PQR SUBPLOT
        self.pqr.cla()
        self.pqr.set_title(r'$p, q, r$')
        self.pqr.set_xlabel('Time (s)')
        self.pqr.set_ylabel('Body Angles (rad)')
        self.pqr.plot(self.t_array, self.pqrData[0], label=r'p', color='r')
        self.pqr.plot(self.t_array, self.pqrData[1], label=r'q', color='g')
        self.pqr.plot(self.t_array, self.pqrData[2], label=r'r', color='b')
        self.pqr.legend(loc='upper right')
        self.pqr.grid(True)

        plt.pause(self.ts)
        self.t += 1