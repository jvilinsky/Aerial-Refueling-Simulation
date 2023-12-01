
import sys
sys.path.append('.')# one directory up
import numpy as np
import parameters.simulation_parameters as P
from viewers.Ass5Animation import flight_animation
from viewers.Ass5Plots import FlightCharts
import matplotlib as plt
from dynamics.FlightDynamics import FlightDynamics as Dynamics
from dynamics.FlightDynamics import ForcesMoments as Forces_Moments 
from dynamics.TF_Statespace import transferfunctions as TF
from dynamics.ComputeTrim import ComputeTrim
from dynamics.Autopilot import Auto_Pilot
from dynamics.Autopilot2 import Auto_Pilot2
from tools.rotations import Euler2Rotation as rot
from viewers.VertexCreator import Vertexes
#from viewers.Animation_wPlot import Visualizer
from viewers.Animation_woPlot import Visualizer
from dynamics.FlightDynamics2 import FlightDynamics2 as Dynamics2



#instantiate plots
flightcharts = FlightCharts()
#flight_anim = flight_animation()
dynamics = Dynamics()
dynamics2 = Dynamics2()
FM = Forces_Moments()
trim = ComputeTrim()
TransferFunctions = TF()
Autopilot = Auto_Pilot()
Autopilot2 = Auto_Pilot2()
Animation = Visualizer()
Animation.initialization()


# initialize the simulation time
t = P.start_time

##################For Model 1#####################
vertexes = Vertexes(P.model1)
verts = vertexes.stl_to_vertices() * P.scale1
#For F16:
#verts = np.matmul(verts, rot(0, 0, -np.pi / 2))
#For F117:
#verts = np.matmul(verts, rot(0, 0, np.pi/4))
#For c130:
verts = np.matmul(verts, rot(0, 0, np.pi/2))


#Initialize Variables

Va = 0.0
da = 0.0
dt = 0.0
de = 0.0
dr = 0.0
delta = np.array([
    [de],
    [dt],
    [da],
    [dr]
])

#Trim conditions
Va = 35
Y = 0
#Y = np.deg2rad(10)
R = np.inf

#Finding Trim
x_trim, u_trim = trim.compute_trim(Va,Y,R)
xtemp = x_trim[np.newaxis] ; utemp = u_trim[np.newaxis]
X_trim = xtemp.transpose() ; U_trim = utemp.transpose()



#Redifine state and delta found from trim
dynamics._state = X_trim
delta = U_trim

#Find Gains for autopilot
TransferFunctions.compute_tf_models(X_trim, U_trim)
gains = TransferFunctions.statespace(X_trim, U_trim)


##################For Model 2#####################
vertexes2 = Vertexes(P.model2)
verts2 = vertexes2.stl_to_vertices() * P.scale2
#For F16:
verts2 = np.matmul(verts2, rot(0, 0, -np.pi / 2))
#For F117:
#verts2 = np.matmul(verts2, rot(0, 0, np.pi/4))


#Initialize Variables

Va = 0.0
da = 0.0
dt = 0.0
de = 0.0
dr = 0.0
delta = np.array([
    [de],
    [dt],
    [da],
    [dr]
])


#Trim conditions
Va2 = 35
Y = 0
#Y = np.deg2rad(10)
R = np.inf

#Finding Trim
x_trim, u_trim = trim.compute_trim(Va2,Y,R)
xtemp = x_trim[np.newaxis] ; utemp = u_trim[np.newaxis]
X_trim2 = xtemp.transpose() ; U_trim = utemp.transpose()
X_trim2[0] = X_trim2[0]+10 # starting position of Model 2
X_trim2[1] = X_trim2[1]+8
#X_trim2[2] = X_trim2[2]+5

#Redifine state and delta found from trim
dynamics2._state2 = X_trim2
delta2 = U_trim

#Find Gains for autopilot
TransferFunctions.compute_tf_models(X_trim2, U_trim)
gains2 = TransferFunctions.statespace(X_trim2, U_trim)


 

##################Main simulation loop######################
#plt.pyplot.pause(10)
print("Press Command-Q to exit...")
while t < P.end_time:
    t_next_plot = t+P.ts_plotting
    while t < t_next_plot:
        #temp values
        Va_c = 35
        
        h_c = 100
    
        chi_c = 0
        # if t > 8:
        #     chi_c = np.deg2rad(45)
    
        
        ##Model 1##
        phi = dynamics._state.item(6)
        theta = dynamics._state.item(7)
        chi = dynamics._state.item(8)
        p = dynamics._state.item(9)
        q = dynamics._state.item(10)
        r = dynamics._state.item(11)
        h = dynamics._state.item(2)
        pn_1 = dynamics._state.item(0)
        pe_1 = dynamics._state.item(1)
        
        u_state = np.array([t,phi,theta,chi,p,q,r,Va,h,Va_c,h_c,chi_c])

        deltaTemp ,T1,T2,T3,T4 = Autopilot.autopilot(gains, u_state) 
        delta = np.array([[deltaTemp.item(0)],
                        [deltaTemp.item(1)],
                        [deltaTemp.item(2)],
                        [deltaTemp.item(3)]])
        
        fm, Va = FM.compute(dynamics._state,delta)
        forceT1 = fm[(0,1,2),0] ; forceT2 = forceT1[np.newaxis] ; force = forceT2.transpose()
        lmnT1 = fm[(3,4,5),0] ; lmnT2 = lmnT1[np.newaxis] ; lmn = lmnT2.transpose()

        y = dynamics.update(fm)


        ##Model 2##
        phi = dynamics2._state2.item(6)
        theta = dynamics2._state2.item(7)
        chi2 = dynamics2._state2.item(8)
        p = dynamics2._state2.item(9)
        q = dynamics2._state2.item(10)
        r = dynamics2._state2.item(11)
        h = dynamics2._state2.item(2)
        pn_2 = dynamics2._state2.item(0)
        pe_2 = dynamics2._state2.item(1)

        if t < 10:
            Va_c2 = 38
        else:
            Va_c2 = 35

    
        chi_c2 = 0
        h_c2 = 97

        
        Pnd = pn_2 - pn_1
        Ped = pe_1 - pe_2
        angle = np.arctan(Ped/(Pnd+0.000001))
        angle = angle - angle*(abs(1-Ped)/Ped)
        angle = (angle + np.pi) % (2*np.pi) - np.pi
        
        if t > 10:
            chi_c2 = angle
            
        Peda = abs(Ped)
        if 3 <= Peda and Peda < 10:
            max_angle = np.deg2rad(5)
            chi_c2 =  np.clip(angle, -max_angle, max_angle)
        if 1.5 <= Peda and Peda < 3:
            max_angle = np.deg2rad(2)
            chi_c2 =  np.clip(angle, -max_angle, max_angle)
        if 0.7 <= Peda and Peda < 1.5:
            max_angle = np.deg2rad(0.5)
            chi_c2 =  np.clip(angle, -max_angle, max_angle)
        if 0.6 <= Peda and Peda < 0.7:
            max_angle = np.deg2rad(0.1)
            chi_c2 =  np.clip(angle, -max_angle, max_angle)
        if 0 <= Ped and Ped < 0.6:
            chi_c2 = chi_c

        if 15 < t and t < 19:
            Va_c2 = 33


        u_state = np.array([t,phi,theta,chi2,p,q,r,Va2,h,Va_c2,h_c2,chi_c2])

        deltaTemp,T1,T2,T3,T4 = Autopilot2.autopilot(gains2, u_state) 
        delta2 = np.array([[deltaTemp.item(0)],
                        [deltaTemp.item(1)],
                        [deltaTemp.item(2)],
                        [deltaTemp.item(3)]])
        
        fm2, Va2 = FM.compute(dynamics2._state2,delta2)
        forceT1 = fm2[(0,1,2),0] ; forceT2 = forceT1[np.newaxis] ; force2 = forceT2.transpose()
        lmnT1 = fm2[(3,4,5),0] ; lmnT2 = lmnT1[np.newaxis] ; lmn2 = lmnT2.transpose()

        y2 = dynamics2.update(fm2)

        t += P.ts_simulation

    ##Model 1##
    dPack = np.array([Va, Va_c, h_c, chi_c])
    ppp = np.array([dynamics._state.item(0), dynamics._state.item(1), -dynamics._state.item(2)])
    ph, th, ps = dynamics._state[(6, 7, 8), 0].flatten()
    r = rot(-ph, th, -ps)
    rVerts = np.matmul(r, verts.T).T
    tVerts = rVerts + ppp

    ##Model 2##
    dPack = np.array([Va2, Va_c, h_c, chi_c])
    ppp2 = np.array([dynamics2._state2.item(0), dynamics2._state2.item(1), -dynamics2._state2.item(2)])
    ph2, th2, ps2 = dynamics2._state2[(6, 7, 8), 0].flatten()
    r2 = rot(-ph2, th2, -ps2)
    rVerts2 = np.matmul(r2, verts2.T).T
    tVerts2 = rVerts2 + ppp2

    Animation.update(tVerts, tVerts2, dynamics._state, force, lmn, delta, dPack, t)
    
    print('Ped=',Ped,' Pnd=',Pnd)
    #flight_anim.update(dynamics._state) 
    flightcharts.update(t, dynamics._state, dynamics2._state2, Va ,Va2, chi, chi2, Va_c, Va_c2, chi_c, chi_c2, h_c)

    
