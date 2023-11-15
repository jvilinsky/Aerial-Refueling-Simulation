
import sys
sys.path.append('.')# one directory up
from math import cos, sin
import numpy as np
#import matplotlib.pyplot as plt
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from mpl_toolkits.mplot3d import Axes3D
from tools.rotations import Quaternion2Euler, Quaternion2Rotation, Euler2Rotation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider, Button

class sliders():
    def __init__(self):

        
        
        self.flag_init = True
        self.fig = plt.figure(2)
        self.xforce = plt.axes([0.01, 0.25, 0.0225, 0.63])
        self.roll=0
        self.xforce_slider = Slider(
            ax=self.xforce,
            label='fx',
            valmin=-3,
            valmax=3,
            valstep=0.01,
            valinit=0,
            orientation='vertical'
        )
        self.xforce_slider.on_changed(self.update)

        self.yforce = plt.axes([0.05, 0.25, 0.0225, 0.63])
        self.yforce_slider = Slider(
            ax=self.yforce,
            label="fy",
            valmin=-3,
            valmax=3,
            valinit=0,
            valstep=0.01,
            orientation="vertical"
        )
        self.yforce_slider.on_changed(self.update)

        self.zforce = plt.axes([0.09, 0.25, 0.0225, 0.63])# location of the slider on figure
        self.zforce_slider = Slider(
            ax=self.zforce,
            label="fz",
            valmin=-3,
            valmax=3,
            valinit=0,
            valstep=0.01,
            orientation="vertical"
        )
        self.zforce_slider.on_changed(self.update)

        self.xmoment = plt.axes([0.25, 0.09, 0.65, 0.03])
        self.xmoment_slider = Slider(
            ax=self.xmoment,
            label='Mx',
            valmin=-0.5,
            valmax=0.5,
            valinit=0,
            valstep=0.01
        )
        self.xmoment_slider.on_changed(self.update)

        self.ymoment = plt.axes([0.25, 0.05, 0.65, 0.03])
        self.ymoment_slider = Slider(
            ax=self.ymoment,
            label='My',
            valmin=-0.5,
            valmax=0.5,
            valinit=0,
            valstep=0.01
        )
        self.ymoment_slider.on_changed(self.update)

        self.zmoment = plt.axes([0.25, 0.01, 0.65, 0.03])
        self.zmoment_slider = Slider(
            ax=self.zmoment,
            label='Mz',
            valmin=-0.5,
            valmax=0.5,
            valinit=0,
            valstep=0.01
        )
        self.zmoment_slider.on_changed(self.update)

    def update(self,val):
    
        
        self.xforce=self.xforce_slider.val
        self.yforce=self.yforce_slider.val
        self.zforce=self.zforce_slider.val
        self.xmoment=self.xmoment_slider.val
        self.ymoment=self.ymoment_slider.val
        self.zmoment=self.zmoment_slider.val
        #plt.pause(0.001)
        self.fig.canvas.draw_idle()    
        
        
        
        #self.update(state0)
    