
import sys
sys.path.append('.')# one directory up
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button

class ControlSurface_Sliders():
    def __init__(self):

        
        
        self.flag_init = True
        self.fig = plt.figure(2)
        self.delta_e = plt.axes([0.01, 0.25, 0.0225, 0.63])
        self.delta_e_slider = Slider(
            ax=self.delta_e,
            label='d_e',
            valmin=-3,
            valmax=3,
            valstep=0.01,
            valinit=0,
            orientation='vertical'
        )
        self.delta_e_slider.on_changed(self.update)

        self.delta_r = plt.axes([0.05, 0.25, 0.0225, 0.63])
        self.delta_r_slider = Slider(
            ax=self.delta_r,
            label="d_r",
            valmin=-3,
            valmax=3,
            valinit=0,
            valstep=0.01,
            orientation="vertical"
        )
        self.delta_r_slider.on_changed(self.update)

        self.delta_a = plt.axes([0.09, 0.25, 0.0225, 0.63])# location of the slider on figure
        self.delta_a_slider = Slider(
            ax=self.delta_a,
            label="d_a",
            valmin=-3,
            valmax=3,
            valinit=0,
            valstep=0.01,
            orientation="vertical"
        )
        self.delta_a_slider.on_changed(self.update)

        self.delta_t = plt.axes([0.25, 0.09, 0.65, 0.03])
        self.delta_t_slider = Slider(
            ax=self.delta_t,
            label='Throttle',
            valmin=0,
            valmax=1,
            valinit=1.0,
            valstep=0.01
        )
        self.delta_t_slider.on_changed(self.update)

        

    def update(self,val):
    
        
        self.delta_e=self.delta_e_slider.val
        self.delta_r=self.delta_r_slider.val
        self.delta_a=self.delta_a_slider.val
        self.delta_t=self.delta_t_slider.val

        #plt.pause(0.001)
        self.fig.canvas.draw_idle()    
        
        
        
        #self.update(state0)
    