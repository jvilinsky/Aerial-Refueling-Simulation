import numpy as np
from parameters import simulation_parameters as P
import control.matlab as mat
from control.matlab import tf, lsim
from tools.rotations import Euler2Rotation


class FlightDynamics:
    def __init__(self):
        self.ts_simulation = P.ts_simulation
        # set initial states based on parameter file
        # _state is the 12x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
        self.jx = P.Jx
        self.jy = P.Jy
        self.jz = P.Jz
        self.jxz = P.Jxz
        self.mass = P.m
        self.gravity = P.g
        self._state = np.array([[P.pn],
                                [P.pe],
                                [P.pd],
                                [P.u],
                                [P.v],
                                [P.w],
                                [P.phi],
                                [P.theta],
                                [P.psi],
                                [P.p],
                                [P.q],
                                [P.r]])

    ###################################
    # public functions
    def update(self, forces_moments):
        '''
            Integrate the differential equations defining dynamics. 
            Inputs are the forces and moments on the aircraft.
            Ts is the time step between function calls.
        '''

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        # print(self._derivatives)
        k1 = self.f(self._state, forces_moments)
        k2 = self.f(self._state + time_step / 2. * k1, forces_moments)
        k3 = self.f(self._state + time_step / 2. * k2, forces_moments)
        k4 = self.f(self._state + time_step * k3, forces_moments)

        self._state += time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # # normalize the quaternion
        # e0 = self._state.item(6)
        # e1 = self._state.item(7)

        # e2 = self._state.item(8)
        # e3 = self._state.item(9)
        # normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        # self._state[6][0] = self._state.item(6)/normE
        # self._state[7][0] = self._state.item(7)/normE
        # self._state[8][0] = self._state.item(8)/normE
        # self._state[9][0] = self._state.item(9)/normE

        # update the message class for the true state
        # self._update_true_state()

    ###################################
    # private functions
    def f(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = state.item(0)
        # east = state.item(1)
        # down = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        # e3 = state.item(9)
        p = state.item(9)
        q = state.item(10)
        r = state.item(11)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l_in = forces_moments.item(3)
        m_in = forces_moments.item(4)
        n_in = forces_moments.item(5)
        body_vel = np.array([u, v, w]).T
        inertial_vel = Euler2Rotation(phi, theta, psi) @ body_vel
        # print(body_vel)
        # print(R_b_w.shape)
        m = P.m
        g = P.g
        ForceVecBody = 1 / m * np.array([[fx], [fy], [fz]], dtype=float)
        jx = self.jx
        jy = self.jy
        jz = self.jz
        jxz = self.jxz
        gamma = jx * jz - jxz ** 2
        gamma1 = (jxz * (jx - jy + jz)) / gamma
        gamma2 = (jz * (jz - jy) + jxz ** 2) / gamma
        gamma3 = jz / gamma
        gamma4 = jxz / gamma
        gamma5 = (jz - jx) / jy
        gamma6 = jxz / jy
        gamma7 = ((jx - jy) * jx + jxz ** 2) / gamma
        gamma8 = jx / gamma
        # gravity_vec=np.array([0, 0, g]).T
        # print(ThrustVecBody.shape)

        # print(temp1)

        # Rgb=rotation_matrix_Gyro2Body(phi, theta)

        # position kinematics
        # pos_dot =
        north_dot = inertial_vel[0]
        east_dot = inertial_vel[1]
        down_dot = inertial_vel[2]

        # position dynamics # body frame # cross product is missi
        temp1 = np.array([[r * v - q * w],
                          [p * w - r * u],
                          [q * u - p * v]]) + ForceVecBody
        # print(temp1)
        u_dot = temp1[0][0]
        v_dot = temp1[1][0]
        w_dot = temp1[2][0]

        # rotational kinematics
        ang_vel = np.array([[p], [q], [r]], dtype=float)
        Rgb = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]])
        temp2 = Rgb @ ang_vel
        # print('temp2=',temp2)
        phi_dot = temp2[0][0]
        theta_dot = temp2[1][0]
        psi_dot = temp2[2][0]

        # rotatonal dynamics
        temp3 = np.array([[gamma1 * p * q - gamma2 * q * r], [gamma5 * p * r - gamma6 * (p ** 2 - r ** 2)],
                          [gamma7 * p * q - gamma1 * q * r]]) + np.array(
            [[gamma3 * l_in + gamma4 * n_in], [(1 / jy) * m_in], [gamma4 * l_in + gamma8 * n_in]], dtype=float)
        p_dot = temp3[0][0]
        q_dot = temp3[1][0]
        r_dot = temp3[2][0]
        # print(temp3)

        # collect the derivative of the states
        x_dot = np.array([[north_dot], [east_dot], [down_dot], [u_dot], [v_dot], [w_dot],
                          [phi_dot], [theta_dot], [psi_dot], [p_dot], [q_dot], [r_dot]], dtype=float)
        # print(x_dot)
        return x_dot

class ForcesMoments:

    def __init__(self):
        self.P=P
        self.Ts=P.ts_simulation

    def compute(self, state, delta):
        # Inertial parameters
        jx= self.P.Jx
        jy= self.P.Jy
        jz= self.P.Jz
        jxz= self.P.Jxz
        gravity=self.P.g
        mass=self.P.m
        Va0=self.P.Va0

        ## aerodynamic parameters
        S_wing        = self.P.S_wing
        b             = self.P.b
        c             = self.P.c
        S_prop        = self.P.S_prop
        rho           = self.P.rho
        e             = self.P.e
        AR            = self.P.AR
        C_L_0         = self.P.C_L_0
        C_D_0         = self.P.C_D_0
        C_m_0         = self.P.C_m_0
        C_L_alpha     = self.P.C_L_alpha
        C_D_alpha     = self.P.C_D_alpha
        C_m_alpha     = self.P.C_m_alpha
        C_L_q         = self.P.C_L_q
        C_D_q         = self.P.C_D_q
        C_m_q         = self.P.C_m_q
        C_L_delta_e   = self.P.C_L_delta_e
        C_D_delta_e   = self.P.C_D_delta_e
        C_m_delta_e   = self.P.C_m_delta_e
        M             = self.P.M 
        alpha0        = self.P.alpha0
        epsilon       = self.P.epsilon
        C_D_p         = self.P.C_D_p
        C_Y_0         = self.P.C_Y_0
        C_ell_0       = self.P.C_ell_0
        C_n_0         = self.P.C_n_0
        C_Y_beta      = self.P.C_Y_beta
        C_ell_beta    = self.P.C_ell_beta
        C_n_beta      = self.P.C_n_beta 
        C_Y_p         = self.P.C_Y_p
        C_ell_p       = self.P.C_ell_p
        C_n_p         = self.P.C_n_p
        C_Y_r         = self.P.C_Y_r
        C_ell_r       = self.P.C_ell_r
        C_n_r         = self.P.C_n_r
        C_Y_delta_a   = self.P.C_Y_delta_a
        C_ell_delta_a = self.P.C_ell_delta_a
        C_n_delta_a   = self.P.C_n_delta_a
        C_Y_delta_r   = self.P.C_Y_delta_r
        C_ell_delta_r = self.P.C_ell_delta_r
        C_n_delta_r   = self.P.C_n_delta_r
        C_prop        = self.P.C_prop
        k_motor       = self.P.k_motor

        
        
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        #e3 = state.item(9)
        p = state.item(9)
        q = state.item(10)
        r = state.item(11)
        # delta=np.array([[delta_a], [delta_e], [delta_r], [delta_t]],dtype=float)
        delta_a = delta.item(1)
        delta_e = delta.item(0)
        delta_r = delta.item(2)
        delta_t = delta.item(3)

        Va = np.sqrt(u ** 2 + v ** 2 + w ** 2)
        #Va = 35
        alpha = np.arctan2(w, u)
        beta = np.arctan2(v, Va)

        qbar = 0.5*rho*Va**2
        ca    = np.cos(alpha)
        sa    = np.sin(alpha)
    
        #compute gravitaional forces
        f_x = mass*gravity*np.sin(theta)
        #f_x = -mass*gravity*np.sin(theta)
        f_y =  mass*gravity*np.cos(theta)*np.sin(phi)
        f_z =  mass*gravity*np.cos(theta)*np.cos(phi)
        
        # compute Lift and Drag forces
        tmp1 = np.exp(-M*(alpha-alpha0))
        tmp2 = np.exp(M*(alpha+alpha0))
        sigma = (1+tmp1+tmp2)/((1+tmp1)*(1+tmp2))
        CL = (1-sigma)*(C_L_0+C_L_alpha*alpha)
        
        
        CD = C_D_0 + 1/(np.pi*e*AR)*(C_L_0+C_L_alpha*alpha)**2
        CL = CL +np.sign(alpha)* sigma*2*sa*sa*ca
        
        
        # compute aerodynamic forces
        f_x = f_x + qbar*S_wing*(-CD*ca + CL*sa)
        f_x = f_x + qbar*S_wing*(-C_D_q*ca + C_L_q*sa)*c*q/(2*Va)
        
        f_y = f_y + qbar*S_wing*(C_Y_0 + C_Y_beta*beta)
        f_y = f_y + qbar*S_wing*(C_Y_p*p + C_Y_r*r)*b/(2*Va)
        
        f_z = f_z + qbar*S_wing*(-CD*sa - CL*ca)
        f_z = f_z + qbar*S_wing*(-C_D_q*sa - C_L_q*ca)*c*q/(2*Va)
        
        #compute aerodynamic torques
        
        tau_phi = qbar*S_wing*b*(C_ell_0 + C_ell_beta*beta)
        tau_phi = tau_phi + qbar*S_wing*b*(C_ell_p*p + C_ell_r*r)*b/(2*Va)

        tau_theta = qbar*S_wing*c*(C_m_0 + C_m_alpha*alpha)
        tau_theta = tau_theta + qbar*S_wing*c*C_m_q*c*q/(2*Va)

        
        tau_psi = qbar*S_wing*b*(C_n_0 + C_n_beta*beta)
        tau_psi = tau_psi + qbar*S_wing*b*(C_n_p*p + C_n_r*r)*b/(2*Va)

        # compute control forces
        f_x = f_x + qbar*S_wing*(-C_D_delta_e*ca+C_L_delta_e*sa)*delta_e
        f_y = f_y + qbar*S_wing*(C_Y_delta_a*delta_a + C_Y_delta_r*delta_r)
        f_z = f_z + qbar*S_wing*(-C_D_delta_e*sa-C_L_delta_e*ca)*delta_e
        
        # compute control torques
        tau_phi = tau_phi + qbar*S_wing*b*(C_ell_delta_a*delta_a + C_ell_delta_r*delta_r)
        tau_theta = tau_theta + qbar*S_wing*c*C_m_delta_e*delta_e
        tau_psi = tau_psi + qbar*S_wing*b*(C_n_delta_a*delta_a + C_n_delta_r*delta_r)
        
        # compute propulsion forces
        motor_temp = k_motor**2*delta_t**2-Va**2
        f_x = f_x + 0.5*rho*S_prop*C_prop*motor_temp
        
        # fx=0.1
        # fy=0
        # fz=0
        # t_phi=0
        # t_theta=0
        # t_psi=0
        f_m=np.array([[f_x], [f_y], [f_z], [tau_phi], [tau_theta], [tau_psi]],dtype=float)
        

        
        return f_m, Va
       
    
    def gust_block(self,state,Va):

        #import states
        pd = state[2,0]
        u = state[3,0]
        v = state[4,0]
        w = state[5,0]
        phi = state[6,0]
        theta = state[7,0]
        psi = state[8,0]

        #ambient wind in inertial frame given as inputs
        V_amb = np.array([
            [self.w_ns],
            [self.w_es],
            [self.w_ds]
        ])

        Lu = Lv = 200
        Lw = 50
        sigma_u = sigma_v = 1.06
        sigma_w = 0.7        
        #Dryden gust model parameters Table
        if pd <= 50 and self.t == 'light':
            Lu = Lv = 200
            Lw = 50
            sigma_u = sigma_v = 1.06
            sigma_w = 0.7
        if pd <= 50 and self.t == 'moderate':
            Lu = Lv = 200
            Lw = 50
            sigma_u = sigma_v = 2.12
            sigma_w = 1.4
        if pd > 50 and self.t == 'light':
            Lu = Lv = 533
            Lw = 533
            sigma_u = sigma_v = 1.5
            sigma_w = 1.5
        if pd > 50 and self.t == 'moderate':
            Lu = Lv = 533
            Lw = 533
            sigma_u = sigma_v = 3.0
            sigma_w = 3.0                                    
        
        #Creating gust forces
        t = np.linspace(0,1000) 

        au = sigma_u * np.sqrt(2 * Va/(Lu))
        av = sigma_v * np.sqrt(3 * Va / (Lv))
        aw = sigma_w * np.sqrt(3 * Va / (Lw))

        #print(u)
        Hu = mat.tf([0, au], [1, Va / Lu])
        Hv = mat.tf([av, av * Va / (np.sqrt(3) * Lv)], [1, 2 * Va / Lv, (Va / Lv)**2])
        Hw = mat.tf([aw, aw * Va / (np.sqrt(3) * Lw)], [1, 2 * Va / Lw, (Va / Lw)**2])

        wn_u = np.random.normal(0, 1, 1)
        wn_v = np.random.normal(0, 1, 1)
        wn_w = np.random.normal(0, 1, 1)

        y_u, T, x_u = mat.lsim(Hu, wn_u[0], t, 0)
        y_v, T, x_v = mat.lsim(Hv, wn_v[0], t, 0)
        y_w, T, x_w = mat.lsim(Hw, wn_w[0], t, 0)

        y_gust = np.array([[y_u[1]], [y_v[1]], [y_w[1]]]) #gust forces

        #Creation of final components of wind
        
        #Rotation matrix from initial frame to body frame
        R = np.array([
            [np.cos(theta)*np.cos(psi),np.cos(theta)*np.sin(psi),-np.sin(theta)],
            [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi),np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi),np.sin(phi)*np.cos(theta)],
            [np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi),np.sin(phi)*np.sin(theta)*np.sin(psi)-np.cos(phi)*np.cos(psi),np.sin(phi)*np.cos(theta)]
        ])

        #combination of steady and gust terms
        V_w = (R @ V_amb)+y_gust
        #print(V_w)

        return V_w
    