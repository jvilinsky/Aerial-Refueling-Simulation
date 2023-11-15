import numpy as np
from parameters import simulation_parameters as P
import control.matlab as mat
from control.matlab import tf, lsim
from tools.rotations import Euler2Rotation


class FlightDynamics2:
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
        self._state2 = np.array([[P.pn],
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
        k1 = self.f(self._state2, forces_moments)
        k2 = self.f(self._state2 + time_step / 2. * k1, forces_moments)
        k3 = self.f(self._state2 + time_step / 2. * k2, forces_moments)
        k4 = self.f(self._state2 + time_step * k3, forces_moments)

        self._state2 += time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

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