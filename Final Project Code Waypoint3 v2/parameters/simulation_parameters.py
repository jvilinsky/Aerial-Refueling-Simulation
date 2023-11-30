import sys
sys.path.append('..')
import numpy as np

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 1/70  # smallest time step for simulation
start_time = 0.  # start time for simulation
end_time = 400.  # end time for simulation

ts_plotting = 1/10  # refresh rate for plots

ts_video = 0.1  # write rate for video

ts_control = ts_simulation  # sample rate for the controller

#Flight Dynamics Parameters
pn = 0.0
pe = 0.0
pd = -1

u = 0.1
v = 0.0
w = 0.0

phi = 0.0
theta = 0.0
psi = 0.0

p = 0.0
q = 0.0
r = 0.0

#Physics properties
m = 13.5 #kg
Jx = 0.824
Jy = 1.135
Jz = 1.759
#Jxz = 0.120
Jxz = 0
g = 9.806650

# aerodynamic parameters
S_wing = 0.55
b = 2.90
c = 0.19
S_prop = 0.2027
rho = 1.2682
e = 0.9
AR = b ** 2 / S_wing
C_L_0 = 0.23
C_D_0 = 0.043
C_m_0 = 0.0135
C_L_alpha = 5.61
C_D_alpha = 0.030
C_m_alpha = -2.74
C_L_q = 7.95
C_D_q = 0.0
C_m_q = -38.21
C_L_delta_e = 0.13
C_D_delta_e = 0.0135
C_m_delta_e = -0.99
M = 50
alpha0 = 0.47
epsilon = 0.16
C_D_p = 0.0
C_Y_0 = 0.0
C_ell_0 = 0.0
C_n_0 = 0.0
C_Y_beta = -0.98
C_ell_beta = -0.13
C_n_beta = 0.073
C_Y_p = 0.0
C_ell_p = -0.51  # ell=p
C_n_p = -0.069
C_Y_r = 0.0
C_ell_r = 0.25
C_n_r = -0.095
C_Y_delta_a = 0.075
C_ell_delta_a = 0.17
C_n_delta_a = -0.011
C_Y_delta_r = 0.19
C_ell_delta_r = 0.0024
C_n_delta_r = -0.069
C_prop = 1
k_motor = 80  # 80
k_T_p = 0
k_omega = 0
#ambient wind conditions
w_ns = 0.0
w_es = 0.0
w_ds = 0.0
t = 'light' #Parameter for turbulance, can be either 'light' or 'moderate'

Va0 = 0.0


###Model Variables###
##f16 model##
#model = 'F16.stl'
#scale = 0.5
##F117##
#model = 'F117.stl'
#scale = 0.02

##Model 1##
model1 = 'KC130_boom.stl'
scale1 = 0.02
##Model 2##
model2 = 'F117.stl'
scale2 = 0.02

g_lim = 15
pause = 0.01
