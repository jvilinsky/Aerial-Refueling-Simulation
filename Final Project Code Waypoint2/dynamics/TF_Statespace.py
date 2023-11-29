import numpy as np
import parameters.simulation_parameters as P
import control.matlab as mat
from control.matlab import tf, lsim

class transferfunctions:

    def compute_tf_models(self, x_trim, u_trim):
        v = x_trim[4,0]

        de = u_trim[0,0]
        dt = u_trim[1,0]
        da = u_trim[2,0]
        dr = u_trim[3,0]
        
        Va_trim = np.sqrt(x_trim[3,0]**2 +x_trim[4,0]**2 +x_trim[5,0]**2)
        alpha_trim = np.arctan(x_trim[5,0]/x_trim[3,0])
        theta_trim = x_trim[7,0]
        #[d_e,d_t,d_a,d_r] u_trim
        #$ define transfer function constants
        G = P.Jx*P.Jz-P.Jxz**2
        G3 = P.Jz/G
        G4 = P.Jxz/G
        C_p_p = G3*P.C_ell_p+G4+P.C_n_p
        a_phi1   = -(1/2)*P.rho*Va_trim**2*P.S_wing*P.b*C_p_p*(P.b/(2*Va_trim))
        C_p_d_a = G3*P.C_ell_delta_a+G4*P.C_n_delta_a
        a_phi2   = (1/2)*P.rho*Va_trim**2*P.S_wing*P.b*C_p_d_a
        self.a_phi1 = a_phi1
        self.a_phi2 = a_phi2

        a_theta1 = -((P.rho*Va_trim**2*P.c*P.S_wing)/(2*P.Jy))*P.C_m_q*(P.c/(2*Va_trim))
        a_theta2 = -((P.rho*Va_trim**2*P.c*P.S_wing)/(2*P.Jy))*P.C_m_alpha
        a_theta3 = ((P.rho*Va_trim**2*P.c*P.S_wing)/(2*P.Jy))*P.C_m_delta_e
        self.a_theta1 = a_theta1
        self.a_theta2 = a_theta2
        self.a_theta3 = a_theta3
    
        a_V1     = ((P.rho*Va_trim*P.S_wing)/P.m)*(P.C_D_0+P.C_D_alpha+P.C_D_delta_e*de)+((P.rho*P.S_prop)/P.m)*P.C_prop*Va_trim
        a_V2     = ((P.rho*P.S_prop)/P.m)*P.C_prop*P.k_motor**2*dt
        a_V3     = P.g
        self.a_V1 = a_V1
        self.a_V2 = a_V2

        B = np.arcsin(v / Va_trim)
        a_beta1     = -((P.rho*Va_trim*P.S_wing)/(2*P.m*np.cos(B)))*P.C_Y_beta
        a_beta2     = ((P.rho*Va_trim*P.S_wing)/(2*P.m*np.cos(B)))*P.C_Y_delta_r
        
        # define transfer functions
        T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0])
        T_chi_phi       = tf([P.g/Va_trim],[1,0])
        T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2])
        T_h_theta       = tf([Va_trim],[1,0])
        T_h_Va          = tf([theta_trim],[1,0])
        T_Va_delta_t    = tf([a_V2],[1,a_V1])
        T_Va_theta      = tf([-a_V3],[1,a_V1])
        T_beta_delta_r     = tf([a_beta2],[1,a_beta1])
        
        return(T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta,  T_h_Va, T_Va_delta_t, T_Va_theta, T_Va_theta, T_beta_delta_r)
    
    def statespace(self, x_trim, u_trim):

            pn = x_trim.item(0)
            pe = x_trim.item(1)
            pd = x_trim.item(2)
            u = x_trim.item(3)
            v = x_trim.item(4)
            w = x_trim.item(5)
            phi = x_trim.item(6)
            theta = x_trim.item(7)
            psi = x_trim.item(8)
            p = x_trim.item(9)
            q = x_trim.item(10)
            r = x_trim.item(11)

            d_e = u_trim[0,0]
            d_a = u_trim[1,0]
            d_r = u_trim[2,0]
            d_t = u_trim[3,0]
            Va = np.sqrt(u**2 + v**2 + w**2)
            aoa = np.arctan(w/u)
            Beta = np.arctan(v/Va)

            gamma = P.Jx * P.Jz - P.Jxz ** 2
            gamma1 = (P.Jxz * (P.Jx - P.Jy + P.Jz)) / gamma
            gamma2 = (P.Jz * (P.Jz - P.Jy) + P.Jxz ** 2) / gamma
            gamma3 = P.Jz / gamma
            gamma4 = P.Jxz / gamma
            gamma5 = (P.Jz - P.Jx) / P.Jy
            gamma6 = P.Jxz / P.Jy
            gamma7 = ((P.Jx - P.Jy) * P.Jx + P.Jxz ** 2) / gamma
            gamma8 = P.Jx / gamma

            C_p_0 = gamma3*P.C_ell_0 + gamma4*P.C_n_0
            C_p_Beta = gamma3*P.C_ell_beta + gamma4*P.C_n_beta
            C_p_p = gamma3*P.C_ell_p + gamma4*P.C_n_p
            C_p_r = gamma3*P.C_ell_r + gamma4*P.C_n_r
            C_p_delta_a = gamma3*P.C_ell_delta_a + gamma4*P.C_n_delta_a
            C_p_delta_r = gamma3*P.C_ell_delta_r + gamma4*P.C_n_delta_r
            C_r_0 = gamma4*P.C_ell_0 + gamma8*P.C_n_0
            C_r_Beta = gamma4*P.C_ell_beta + gamma8*P.C_n_beta
            C_r_p = gamma4*P.C_ell_p + gamma8*P.C_n_p
            C_r_r = gamma4*P.C_ell_r + gamma8*P.C_n_r
            C_r_delta_a = gamma4*P.C_ell_delta_a + gamma8*P.C_n_delta_a
            C_r_delta_r = gamma4*P.C_ell_delta_r + gamma8*P.C_n_delta_r



            Y_v = ((P.rho*P.S_wing*v)/(4*P.M*Va))*(P.C_Y_p*p + P.C_Y_r*r) + ((P.rho*P.S_wing*v)/P.M)*(P.C_Y_0 + P.C_Y_beta*Beta + P.C_Y_delta_a*d_a + P.C_Y_delta_r*d_r) + ((P.rho*P.S_wing*P.C_Y_beta)/(2*P.M))*np.sqrt(u**2 + w**2)
            Y_p = w + ((P.rho*Va*P.S_wing*P.b)/(4*P.M))*P.C_Y_p
            Y_r = -u + ((P.rho*Va*P.S_wing*P.b)/(4*P.M))*P.C_Y_r
            Y_delta_a = ((P.rho*Va**2*P.S_wing)/(2*P.M)) * P.C_Y_delta_a
            Y_delta_r = ((P.rho*Va**2*P.S_wing)/(2*P.M)) * P.C_Y_delta_r
            L_v = ((P.rho*P.S_wing*P.b**2*v)/(4*Va))*(C_p_p*p + C_p_r*r) + (P.rho*P.S_wing*P.b*v)*(C_p_0 + C_p_Beta*Beta + C_p_delta_a*d_a + C_p_delta_r*d_r) + (P.rho*P.S_wing*P.b*C_p_Beta/2)*np.sqrt(u**2 + w**2)
            L_p = gamma1*q + (P.rho*Va*P.S_wing*P.b**2/4)*C_p_p
            L_r = -gamma2*q + (P.rho*Va*P.S_wing*P.b**2/4)*C_p_r
            L_delta_a = (P.rho*Va**2*P.S_wing*P.b/2)*C_p_delta_a
            L_delta_r = (P.rho*Va**2*P.S_wing*P.b/2)*C_p_delta_r
            N_v = ((P.rho*P.S_wing*P.b**2*v)/(4*Va))*(C_r_p*p + C_r_r*r) + (P.rho*P.S_wing*P.b*v)*(C_r_0 + C_r_Beta*Beta + C_r_delta_a*d_a + C_r_delta_r*d_r) + (P.rho*P.S_wing*P.b*C_r_Beta/2)*np.sqrt(u**2 + w**2)
            N_p = gamma7*q + (P.rho*Va*P.S_wing*P.b**2 / 4)*C_r_p
            N_r = -gamma1*q + (P.rho*Va*P.S_wing*P.b**2 / 4)*C_r_r
            N_delta_a = (P.rho*Va**2*P.S_wing*P.b / 2)*C_r_delta_a
            N_delta_r = (P.rho*Va**2*P.S_wing*P.b / 2)*C_r_delta_r

            C_D = P.C_D_0 + (P.C_D_alpha * aoa)
            C_L = P.C_L_0 + (P.C_L_alpha * aoa)
            C_x_a = -P.C_D_alpha * np.cos(aoa) + P.C_L_alpha * np.sin(aoa)
            C_x_0 = -P.C_D_0 * np.cos(aoa) + P.C_L_0 * np.sin(aoa)
            C_x_d_e = -P.C_D_delta_e * np.cos(aoa) + P.C_L_delta_e * np.sin(aoa)
            C_x_q = -P.C_D_q * np.cos(aoa) + P.C_L_q * np.sin(aoa)
            C_Z = -C_D * np.sin(aoa) - C_L * np.cos(aoa)
            C_Z_q = -P.C_D_q * np.sin(aoa) - P.C_L_q * np.cos(aoa)
            C_Z_delta_e = -P.C_D_delta_e * np.sin(aoa) - P.C_L_delta_e * np.cos(aoa)
            C_Z_0 = -P.C_D_0 * np.sin(aoa) - P.C_L_0 * np.cos(aoa)
            C_Z_alpha = - P.C_D_alpha * np.sin(aoa) - P.C_L_alpha * np.cos(aoa)

            X_u = ((u * P.rho * P.S_wing) / P.M) * (C_x_0 + (C_x_a * d_a) + (C_x_d_e * d_e)) - (
                        (P.rho * P.S_wing * w * C_x_a) / (2 * P.M)) + (
                            (P.rho * P.S_wing * P.c * C_x_q * u * q) / (4 * P.M * Va)) - (
                            (P.rho * P.S_prop * P.C_prop *u) / P.M)
            X_w = -q + ((w * P.rho * P.S_wing) / P.M) * (C_x_0 + (C_x_a * d_a) + (C_x_d_e * d_e)) + (
                        (P.rho * P.S_wing * P.c * C_x_q * w * q) / (4 * P.M * Va)) + (
                            (P.rho * P.S_wing * u * C_x_a) / (2 * P.M)) - (
                            (P.rho * P.S_prop * P.C_prop * w) / P.M)
            X_q = -w + ((P.rho * Va * P.S_wing * C_x_q * P.c) / (4 * P.M))
            X_delta_e = (P.rho * (Va ** 2) * P.S_wing * C_x_d_e) / (2 * P.M)
            X_delta_t = (P.rho * P.S_prop * P.C_prop * (P.k_motor ** 2) * d_t) / P.M
            Z_u = q + ((u * P.rho * P.S_wing) / (P.M)) * (
                        C_Z_0 + (C_Z_alpha * aoa) + (C_Z_delta_e * d_e)) - (
                            (P.rho * P.S_wing * C_Z_alpha *w) / (2 * P.M)) + (
                            (u * P.rho * P.S_wing * C_Z_q * P.c * q) / (4 * P.M * Va))
            Z_w = ((w * P.rho * P.S_wing) / (P.M)) * (C_Z_0 + (C_Z_alpha * aoa) + (C_Z_delta_e * d_e)) + (
                        (P.rho * P.S_wing * C_Z_alpha * u) / (2 * P.M)) + (
                            (w * P.rho * P.S_wing * C_Z_q * P.c * q) / (4 * P.M * Va))
            Z_q = u + (P.rho * Va * P.S_wing * C_Z_q * P.c) / (4 * P.M)
            Z_delta_e = (P.rho * (Va ** 2) * P.S_wing * C_Z_delta_e) / (2 * P.M)
            M_u = ((u * P.rho * P.S_wing * P.c) / P.Jy) * (
                        P.C_m_0 + (P.C_m_alpha * aoa) + (P.C_m_delta_e * d_e)) - (
                            (P.rho * P.S_wing * P.c * P.C_m_alpha * w) / (2 * P.Jy)) + (
                            (P.rho * P.S_wing * (P.c ** 2) * P.C_m_q * q * u) / (4 * P.Jy * Va))
            M_w = ((w * P.rho * P.S_wing * P.c) / P.Jy) * (P.C_m_0 + P.C_m_alpha * aoa + P.C_m_delta_e * d_e) + (
                            (P.rho * P.S_wing * P.c * P.C_m_alpha * u) / (2 * P.Jy)) + (
                            (P.rho * P.S_wing * P.c ** 2 * P.C_m_q * q * w) / (4 * P.Jy * Va))
            M_q = (P.rho * Va * P.c ** 2 * P.S_wing * P.C_m_q) / (4 * P.Jy)
            M_delta_e = (P.rho * (Va ** 2) * P.S_wing * P.c * P.C_m_delta_e) / (2 * P.Jy)

            Alat = np.array([[Y_v, Y_p, Y_r, P.g*np.cos(theta)*np.cos(phi), 0],
                        [L_v, L_p, L_r, 0, 0], [N_v, N_p, N_r, 0, 0],
                        [0, 1, np.cos(phi)*np.tan(theta), q*np.cos(phi)*np.tan(theta)-r*np.sin(phi)*np.tan(theta), 0],
                        [0, 0, np.cos(phi)*(1/np.cos(theta)), p*np.cos(phi)*(1/np.cos(theta)) - r*np.sin(phi)*(1/np.cos(theta)), 0]])
            Blat = np.array([[Y_delta_a, Y_delta_r], [L_delta_a, L_delta_r],[N_delta_a, N_delta_r], [0, 0], [0, 0]])

            Along = np.array([[X_u, X_w, X_q, -P.g*np.cos(theta), 0],
                            [Z_u, Z_w, Z_q, -P.g*np.sin(theta), 0],
                            [M_u, M_w, M_q, 0, 0],
                            [0, 0, 1, 0, 0],
                            [np.sin(theta), -np.cos(theta), 0, u*np.cos(theta) + w*np.sin(theta), 0]])

            Blong = np.array([[X_delta_e, X_delta_t], [Z_delta_e, 0], [M_delta_e, 0], [0, 0], [0, 0]])

            elatvalue, elatvect = np.linalg.eig(Alat)
            elongvalue, elongvect = np.linalg.eig(Along)


            Lat_a1 = elatvalue[2].real
            Lat_a2 = elatvalue[2].imag

            Long_a1 = elongvalue[2].real
            Long_a2 = elongvalue[2].imag
            
            Lat_w_n = np.sqrt(Lat_a1**2+Lat_a2**2) #lateral natural frequency
            Long_w_n = np.sqrt(Long_a1**2+Long_a2**2) #longitudinal natural frequency
            Lat_d = -Lat_a1/Lat_w_n #lateral damping
            Long_d = -Long_a1/Long_w_n #longitudinal damping

            zeta = .707
            # roll
            tr_roll = 0.5
            wn_roll = 2.2/tr_roll
            kp_roll = wn_roll**2/self.a_phi1
            kd_roll = (2*zeta*wn_roll - self.a_phi1)/self.a_phi2

            # Course hold
            zeta_course = .5
            # tr_course = .5
            wn_course = 1
            kp_course = (2 * zeta_course * wn_course * Va)/P.g
            ki_course = (wn_course**2 * Va)/P.g


            # Pitch Attitude Hold
            zeta_pitch = .1
            tr_pitch = .1
            wn_pitch = 2.2/tr_pitch
            kp_pitch = (wn_pitch**2 - self.a_theta2)/self.a_theta3
            kd_pitch = (2 * zeta_pitch * wn_pitch - self.a_theta1)/self.a_theta3
            ktheta_DC = (kp_pitch * self.a_theta3)/(self.a_theta2 + kp_pitch * self.a_theta3)

            # altitude from Pitch Gain
            tr_altitude = 1
            wn_altitude = 2.2 / tr_altitude
            kp_alt = (2 * zeta * wn_altitude) / (ktheta_DC * Va)
            ki_alt = wn_altitude**2 / (ktheta_DC * Va)

            # airspeed from pitch
            tr_airspeed = 1
            wn_airspeed = 2.2/tr_airspeed
            kp_airsp = (self.a_V1 - 2 * zeta * wn_airspeed) / ktheta_DC
            ki_airsp = wn_airspeed**2 / (ktheta_DC * P.g)

            # Airspeed from Throttle
            #tr_throttle = 13.85
            tr_throttle = 0.1
            zeta_throttle = 1.5
            wn_throttle = 2.2 / tr_throttle
            kp_throt = (2 * zeta_throttle * wn_throttle - self.a_V1)/self.a_V2
            ki_throt = wn_throttle**2 / self.a_V2

            gains = np.array([kp_pitch,kd_pitch,kp_course,ki_course,kp_roll,kd_roll,kp_airsp,ki_airsp,kp_throt,ki_throt,kp_alt,ki_alt])

            return gains
    
