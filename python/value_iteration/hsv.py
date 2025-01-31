import numpy as np
import torch

from deep_differential_network.utils import jacobian
from value_iteration.cost_functions import ArcTangent, SineQuadraticCost, QuadraticCost, QuadraticFunction
CUDA_AVAILABLE = torch.cuda.is_available()


class BaseSystem:
    def __init__(self):
        self.n_state = 0
        self.n_act = 0
        self.x_lim = []

    def check_dynamics(self, n_samples=10):
        # Checking Gradients:
        to_x_test = torch.distributions.uniform.Uniform(-self.x_lim, self.x_lim).sample((n_samples,))
        to_x_test = to_x_test.view(-1, self.n_state, 1).float().to(self.theta.device)
        to_x_test[0,:,0] = 0.
        np_x_test = to_x_test.cpu().numpy()

        np_a, np_B, np_dadx, np_dBdx = self.dyn(np_x_test, gradient=True)
        to_a, to_B, to_dadx, to_dBdx = [x.cpu().numpy() for x in self.dyn(to_x_test, gradient=True)]

        assert np.allclose(to_a, np_a, atol=1.e-5)
        assert np.allclose(to_B, np_B, atol=1.e-5)
        assert np.allclose(to_dadx, np_dadx, atol=1.e-5)
        assert np.allclose(to_dBdx, np_dBdx, atol=1.e-5)

        grad_auto_dadx = jacobian(lambda x: self.dyn(x)[-2], to_x_test).view(-1, self.n_state, self.n_state).cpu().numpy()
        grad_auto_dBdx = jacobian(lambda x: self.dyn(x)[-1], to_x_test).view(-1, self.n_state, self.n_state, self.n_act).cpu().numpy()

        assert np.allclose(to_dBdx, grad_auto_dBdx, atol=1.e-3)
        assert np.allclose(to_dadx, grad_auto_dadx, atol=1.e-3)

    def dyn(self, x):
        raise AttributeError

    def grad_dyn(self, x):
        raise AttributeError

class hsv(BaseSystem):
    name = "hsv"
    labels = ('V', 'gamma')

    def __init__(self, cuda=False, **kwargs):
        super(hsv, self).__init__()
        device = torch.device('cuda') if cuda else torch.device('cpu')

        # Define Duration:
        self.T = kwargs.get("T", 10.0)
        self.dt = kwargs.get("dt", 1./500.)

        # Do control saturation (=1) or not (=0)
        #self.dousat = True
        self.dousat = False

        # Do input transformation for V(x) to take out height h (=1) or not (=0)
        self.doVxtrans = True

        # Define the System:
        self.n_state = 5
        self.n_act = 2
        self.n_joint = 1
        self.n_parameter = 3

        # Continuous Joints:
        # Right now only one continuous joint is supported
        self.wrap, self.wrap_i = False, 0

        # Rad <-> deg
        D2R = np.pi / 180.
        R2D = 180. / np.pi
        self.D2R = D2R
        self.R2D = R2D
        # Velocity units
        #self.Vscl = 1./1000.    # V -- kft/s
        self.Vscl = 1. / 100.  # V -- kft/s

        # State Constraints:
        self.x_start = torch.tensor([self.Vscl*100., 0., 0., 0., 0.])     # Central IC for generating trajectories to plot during eval
        self.x_start_var = torch.tensor([self.Vscl*1.e-6, 1.e-6, 1.e-6, 1.e-6, 1.e-6])   # Variance for generating trajectories to plot during eval
        self.x_lim = torch.tensor([self.Vscl * 2000., 2.5, 15., 15., 100.])
        self.x_lim_eval = torch.tensor([self.Vscl * 1000., 5.0, 30., 30., 10000.])

        # Expected return evals sampled in +/- this
        self.x_init = torch.tensor([self.Vscl * 100., 1., 0.01, 0.01, 0.01])
        self.x_init_train = torch.tensor([self.Vscl * 150., 1.5, 5.0, 5.0, 0.01])

        # Control saturation
        self.u_lim = torch.tensor([1.5, 20.])
        self.u_min = torch.tensor([0., -self.u_lim[1]])
        self.u_max = self.u_lim


        #
        # Lookup table grid parameters (for evaluating value function, policy after training)
        self.x_tbl_min = - torch.tensor([self.Vscl * 250., 5., 30., 40., 0.01])
        self.x_tbl_max = - self.x_tbl_min
        self.x_tbl_nxpts = torch.tensor([9, 7, 7, 7, 2]).int()
        # 2D Lookup parameters
        self.x_tbl_min2 = torch.tensor([self.Vscl*-100., -1.])
        self.x_tbl_max2 = -self.x_tbl_min2
        self.x_tbl_nxpts2 = torch.tensor([150, 150]).int()

        # Thresholds to check when evaluating success percentage (post-transformation units)
        self.threshmat_x = torch.tensor([
            [0., 10. * self.Vscl, 50.],
            [1., 0.1, 50.],
            [0., 1. * self.Vscl, 25.],
            [1., 0.01, 25.]])



        # ***********************
        #
        # PLOT SETTINGS
        #
        #
        self.psett_y_ticks = [-1., -0.5, 0.0, 0.5, 1.]
        self.psett_x_ticks = [self.Vscl *-100., self.Vscl *-50., self.Vscl *0.0, self.Vscl *+50., self.Vscl *+100.]
        self.psett_y_tick_label = [r"$-1$", r"$-0.5$", r"$0$", r"$+0.5$", r"$+1$"]
        self.psett_xlabel = (r"Velocity $V$ [kft/s]", r"FPA $\gamma$ [deg]")
        self.psett_ulabel = (r"Throttle $\delta_{T}$ [-]", r"Elevator $\delta_{E}$ [deg]")

        # Trim state:
        self.Ve = self.Vscl*1.5060285e+04        # Trim arispeed (ft/s)
        self.ge = 0.            # Trim FPA (deg)
        self.ae = 1.770370827541618        # Trim AOA (deg)
        self.thetae = self.ge + self.ae     # Trim pitch (deg)
        self.he = 110000.       # Trim altitude (ft)

        # Define Dynamics:
        self.g = 32.1741        # Gravitational field constant (ft/s^2)
        self.mu = 1.39e16       # Gravitational constant (ft^3/s^2)
        self.RE = 20903500.     # Radius of earth (ft)
        self.mref = 9375.       # Vehicle mass (slug)
        self.wref = self.mref * self.g      # Vehicle weight (lb)
        self.Iyy = 7.e6         # Pitch-axis moment of inertia (slug-ft^2)
        self.S = 3603.          # Reference area (ft^2)
        self.cbar = 80.         # MAC (ft)

        # Aerodynamic model constnats
        self.c_rho = 0.00238    # Constant multiplying \rho(h)
        self.c_rho_exp = -1 / 24000.    # Constant in exponential in \rho(h)
        self.c_sos_h2 = 8.99e-9     # Constant multiplying h^2 factor in a(h)
        self.c_sos_h1 = - 9.16e-4  # Constant multiplying h^1 factor in a(h)
        self.c_sos_h0 = 996.    # Constant multiplying h^0 factor in a(h)
        self.c_CLa_M0 = 0.493   # Constant multiplying M^0 factor in C_{L,\alpha}
        self.c_CLa_Mm1 = 1.91  # Constant multiplying M^-1 factor in C_{L,\alpha}
        self.c_CLdEu_a2 = -0.235580737050513  # Constant multiplying \alpha^2 factor in C_{L,\delta_E}
        self.c_CLdEu_a1 = -0.00451812931308866  # Constant multiplying \alpha^1 factor in C_{L,\delta_E}
        self.c_CLdEu_a0 = -0.0291335007132150  # Constant multiplying \alpha^0 factor in C_{L,\delta_E}
        self.c_CD = 0.0082  # Constant multiplying C_{D}
        self.c_CD_a2 = 171.  # Constant multiplying \alpha^2 factor in C_{D}
        self.c_CD_a1 = 1.15  # Constant multiplying \alpha^1 factor in C_{D}
        self.c_CD_a0 = 1.  # Constant multiplying \alpha^0 factor in C_{D}
        self.c_CD_M2 = 0.0012  # Constant multiplying M^2 factor in C_{D}
        self.c_CD_M1 = - 0.054  # Constant multiplying M^1 factor in C_{D}
        self.c_CD_M0 = 1.  # Constant multiplying M0 factor in C_{D}
        self.c_CMa = 1.e-4  # Constant multiplying C_{M,\alpha}
        self.c_CMa_M0 = 0.06  # Constant multiplying M^0 factor in C_{M,\alpha}
        self.c_CMa_Mexp = -1./3.  # Constant multiplying exponential in M in C_{M,\alpha}
        self.c_CMa_a2 = -6565.  # Constant multiplying \alpha^2 factor in C_{M,\alpha}
        self.c_CMa_a1 = 6875.  # Constant multiplying \alpha^1 factor in C_{M,\alpha}
        self.c_CMa_a0 = 1.  # Constant multiplying \alpha^0 factor in C_{M,\alpha}
        self.c_CMq_M1 = -0.025  # Constant multiplying M^1 factor in C_{M,q}
        self.c_CMq_M0 = 1.37  # Constant multiplying M^0 factor in C_{M,q}
        self.c_CMq_a2 = -6.83  # Constant multiplying \alpha^2 factor in C_{M,q}
        self.c_CMq_a1 = 0.303  # Constant multiplying \alpha^1 factor in C_{M,q}
        self.c_CMq_a0 = - 0.23  # Constant multiplying \alpha^0 factor in C_{M,q}
        self.c_CMdE = 0.0292    # Pitch moment elevator increment effectiveness constant (1/deg)
        self.c_kx = 0.0105      # Constant multiplying thrust coefficient term k(x)
        self.c_kx_M0 = 1.  # Constant multiplying M^0 factor in thrust coefficient term k(x)
        self.c_kx_Mm1 = 17.  # Constant multiplying M^-1 factor in thrust coefficient term k(x)
        self.c_CTu = 1.15  # Constant multiplying C_T



        # theta = \nu * d_0
        self.theta_min = torch.tensor([0.75, 0.75, 0.75]).to(device).view(1, self.n_parameter, 1)
        self.theta = torch.tensor([1., 1., 1.]).to(device).view(1, self.n_parameter, 1)
        self.theta_max = torch.tensor([1.25, 1.25, 1.25]).to(device).view(1, self.n_parameter, 1)

        # ***********************
        #
        # MODEL CONSTANTS
        #


        # Original State Variables (pre-transformation units, for evaluating dynamics)
        #       x =   [    V (ft/s)
        #                   \gamma (rad)
        #                   h (ft)
        #                   \alpha (rad)
        #                   q (rad/s)       ]
        #
        # New Control Variables (post-transformation units, simulation units)
        #     u = [    d_T
        #               d_E  (rad)]
        #
        # New State Variables (post-transformation units, simulation units)
        #       x' =   [    V (kft/s)
        #                   \gamma (deg)
        #                   \theta (deg)
        #                   q (deg/s)
        #                   h (ft)          ]
        #
        # New Control Variables (post-transformation units, simulation units)
        #     u' = [    d_T
        #               d_E  (deg)]

        self.sx = torch.tensor([ [self.Vscl, 0., 0., 0., 0.],
                            [0., R2D, 0., 0., 0.],
                            [0., R2D, 0., R2D, 0.],
                            [0., 0., 0., 0., R2D],
                            [0., 0., 1., 0., 0.] ])
        self.invsx = torch.tensor([ [1./self.Vscl, 0., 0., 0., 0.],
                            [0., D2R, 0., 0., 0.],
                            [0., 0., 0., 0, 1.],
                            [0., -D2R, D2R, 0., 0.],
                            [0., 0., 0., D2R, 0.] ])
        self.su = torch.diag(torch.tensor([1., R2D]))
        self.invsu = torch.diag(torch.tensor([1., D2R]))

        # V(x) state transformation to take out height mode h
        self.Vxtrans = torch.tensor([ [1., 0., 0., 0., 0.],
                            [0., 1., 0., 0., 0.],
                            [0., 0., 1., 0., 0.],
                            [0., 0., 0., 1., 0.] ])

        # Number of features for V(x)
        self.n_feature = self.Vxtrans.shape[0] if self.doVxtrans else self.n_state

        # Closed-loop design includes reference command pre-filter
        self.pf1nopf0 = True
        #self.pf1nopf0 = False
        #self.pfavec = torch.tensor([0.3, 1.])
        self.pfavec = torch.tensor([0.75, 0.5])

        # ***********************
        #
        # TRIM
        #

        # Equilibrium state: post-transformation units (V_e, \gamma_e, \theta_e, q_e, h_e)
        xe = torch.tensor([self.Ve, self.ge, self.thetae, 0., self.he])
        self.x_target = xe

        # Calculate equilibrium controls u_e = [\delta_{T,e}, \delta_{E,e}]  (-, deg)
        self.ue = torch.tensor([0.1756, -0.3947])

        # LQR Baseline:
        out = self.dyn(torch.zeros(self.n_state).numpy(), gradient=True)
        self.A = out[2].reshape((1, self.n_state, self.n_state)).transpose((0, 2, 1))
        self.B = out[1].reshape((1, self.n_state, self.n_act))

        # Test dynamics:
        self.check_dynamics()

        self.device = None
        hsv.cuda(self) if cuda else hsv.cpu(self)

    def dyn(self, x, dtheta=None, gradient=False):
        cat = torch.cat

        is_numpy = True if isinstance(x, np.ndarray) else False
        x = torch.from_numpy(x).to(self.theta.device) if isinstance(x, np.ndarray) else x
        #
        # x = torch.from_numpy(x).to(self.theta.device) if isinstance(x, np.ndarray) else x.to(self.theta.device)
        x = x.view(-1, self.n_state, 1)
        n_samples = x.shape[0]

        # Update the dynamics parameters with disturbance:
        if dtheta is not None:
            dtheta = torch.from_numpy(dtheta).float() if isinstance(dtheta, np.ndarray) else dtheta
            dtheta = dtheta.view(-1, self.n_parameter, 1)
            assert dtheta.shape[0] in (1, n_samples)

            theta = self.theta + dtheta
            theta = torch.min(torch.max(theta, self.theta_min), self.theta_max)

        else:
            theta = self.theta
            theta = theta

        # ***********************
        #
        # EXTRACT STATE VECTOR
        #

        # Perturbed lift, drag, and pitch moment coefficient parameters \nu
        nu_CL = theta[:, 0]
        nu_CD = theta[:, 1]
        nu_CMa = theta[:, 2]

        # Trim state, control
        xe = self.x_target.to(self.theta.device)
        # ue = self.ue

        # Shift state to reflect trim
        x = x + xe.view(1, self.n_state, 1)

        # DEBUGGING
        #x = x + torch.matmul(self.invsx, xe.view(1, self.n_state, 1))

        # Apply inverse transformation: deg -> rad
        sx = self.sx.to(self.theta.device)
        invsx = self.invsx.to(self.theta.device)
        invsu = self.invsu.to(self.theta.device)
        x = torch.matmul(invsx, x)

        # DEBUGGING
        #xtst = torch.tensor([1000, 1*torch.pi/180., -5000, -2*torch.pi/180., 10*torch.pi/180.])
        #x = x + xtst.view(-1,self.n_state,1)

        # Extract states (pre-transformation; i.e., in rad)
        V, g, h, alpha, q = x[:, 0], x[:, 1], x[:, 2], x[:, 3], x[:, 4]

        # Trigonometric functions
        sg = torch.sin(g)
        cg = torch.cos(g)
        sa = torch.sin(alpha)
        ca = torch.cos(alpha)

        # ***********************
        #
        # AERODYNAMIC COEFFICIENTS
        #

        # Distance from center of earth to vehicle
        r = self.RE + h

        # Air density
        rho = self.c_rho * torch.exp(self.c_rho_exp * h)

        # Speed of sound
        sos = self.c_sos_h2 * h ** 2 + self.c_sos_h1 * h + self.c_sos_h0

        # Mach number
        M = V / sos

        # Dynamic pressure
        qinf = 0.5 * rho * V ** 2

        # Lift coefficient -- WITHOUT lift-elevator effects
        CLa_a = alpha
        CLa_M = self.c_CLa_M0 + self.c_CLa_Mm1 / M
        CLa_nom = CLa_a * CLa_M
        CL = nu_CL * CLa_nom

        # Lift coefficient -- lift-elevator effects -- input gain for g(x)
        CLdEu = self.c_CLdEu_a2 * alpha ** 2 + self.c_CLdEu_a1 * alpha + self.c_CLdEu_a0

        # Drag coefficient
        CD_a = self.c_CD_a2*alpha**2 + self.c_CD_a1*alpha + self.c_CD_a0
        CD_M = self.c_CD_M2*M**2 + self.c_CD_M1*M + self.c_CD_M0
        CD_nom = self.c_CD * CD_a * CD_M
        CD = nu_CD * CD_nom

        # Pitch moment coefficient -- basic increment C_{M,\alpha}
        CMa_a = self.c_CMa_a2 * alpha**2 + self.c_CMa_a1*alpha + self.c_CMa_a0
        CMa_M = self.c_CMa_M0 - torch.exp(M * self.c_CMa_Mexp)
        CMa_nom = self.c_CMa * CMa_M * CMa_a
        CMa = nu_CMa * CMa_nom

        # Pitch moment coefficient -- rotational damping increment C_{M,q}
        CMq_a = self.c_CMq_a2 * alpha ** 2 + self.c_CMq_a1 * alpha + self.c_CMq_a0
        CMq_M = self.c_CMq_M1 * M + self.c_CMq_M0
        CMq = self.cbar / (2. * V) * q * CMq_M * CMq_a

        # Pitch moment coefficient -- elevator increment -- input gain for g(x)
        CMdEu = self.c_CMdE

        # Pitch moment coefficient -- elevator increment -- before elevator added
        CMdE0 = self.c_CMdE * ( - alpha)

        # Pitch moment coefficient -- terms pertaining to f(x)
        CM0 = CMa + CMdE0 + CMq

        # Thrust coefficient -- term k(x)
        kx = self.c_kx * (self.c_kx_M0 + self.c_kx_Mm1/M)

        # Thrust coefficient -- input gain for g(x)
        CTu = kx * (self.c_CTu)

        # ***********************
        #
        # AERODYNAMIC FORCES/MOMENTS
        #

        # Thrust T -- input gain for g(x)
        Tu = qinf * self.S * CTu

        # Lift L -- without lift-elevator effects
        L = qinf * self.S * CL
        LdE0 = L

        # Lift L -- lift-elevator effects -- input gain for g(x)
        LdEu = qinf * self.S * CLdEu

        # Drag D
        D = qinf * self.S * CD

        # Pitch moment M_{yy} -- before elevator increment
        Myy0 = qinf * self.S * self.cbar * CM0

        # Pitch moment M_{yy} -- elevator increment -- input gain for g(x)
        Myyu = qinf * self.S * self.cbar * CMdEu

        # ***********************
        #
        # FIRST DERIVATIVE TERMS
        #

        # \dot{V}
        dVu = Tu * ca / self.mref
        dV0 = - D / self.mref - self.mu * sg / (r ** 2)

        # \dot{\gamma}
        dgu = Tu * sa / (self.mref * V)
        dg0 = LdE0 / (self.mref * V) - ((self.mu - V ** 2 * r) * cg) / (V * r ** 2)

        # \dot{\gamma} -- ELEVATOR TERM \delta_{E}
        dgudE = LdEu / (self.mref * V)

        # \dot{h}
        dh = V * sg

        # \dot{\alpha}
        dau = - dgu
        da0 = q - dg0

        # \dot{q}
        dqu = Myyu / self.Iyy
        dq0 = Myy0 / self.Iyy

        # Make drift dynamics f(x) -- pre-transformation units
        fx = torch.zeros(x.shape[0], self.n_state, 1).to(self.theta.device)
        fx[:, 0, 0] = dV0.squeeze()
        fx[:, 1, 0] = dg0.squeeze()
        fx[:, 2, 0] = dh.squeeze()
        fx[:, 3, 0] = da0.squeeze()
        fx[:, 4, 0] = dq0.squeeze()

        # Make input gain matrix g(x) -- with lift-elevator effects -- pre-transformation units
        gx = torch.zeros(x.shape[0], self.n_state, self.n_act).to(self.theta.device)
        gx[:, 0, 0] = dVu.squeeze()
        gx[:, 1, 0] = dgu.squeeze()
        gx[:, 1, 1] = dgudE.squeeze()
        gx[:, 3, 0] = dau.squeeze()
        gx[:, 3, 1] = -dgudE.squeeze()
        gx[:, 4, 1] = dqu.squeeze()

        # Evaluate drift dynamics -- post-transformation units
        a = torch.matmul(sx, fx).to(self.theta.device)

        # DEBUGGING
        #a = fx

        # Evaluate input gain matrix g(x) -- post-transformation units
        B = torch.matmul(sx, torch.matmul(gx, invsu))

        assert a.shape == (n_samples, self.n_state, 1)
        assert B.shape == (n_samples, self.n_state, self.n_act)
        out = (a, B)

        if gradient:
            #zeros, ones = torch.zeros_like(x[:, 1]), torch.ones_like(x[:, 1])

            # ***********************
            #
            # PARTIAL DERIVATIVE TERMS -- AERODYNAMIC COEFFICIENTS
            #

            # d / dV -- f(x)
            dCLadV = - nu_CL * CLa_a * self.c_CLa_Mm1 / (M ** 2 * sos)
            dCDdV = nu_CD * self.c_CD * CD_a * (2*self.c_CD_M2*M/sos + self.c_CD_M1/sos)
            dCMadV = nu_CMa * self.c_CMa * CMa_a * (-self.c_CMa_Mexp * torch.exp(M*self.c_CMa_Mexp) * 1/sos)
            dCMqdV = - q * self.cbar / (2.*V**2) * CMq_M * CMq_a + q * self.cbar / (2.*V) * CMq_a * (self.c_CMq_M1/sos)
            dCM0dV = dCMadV + dCMqdV
            # d / dV -- g(x)
            dCTdV = self.c_kx * self.c_CTu * self.c_kx_Mm1 * (-1. / M ** 2 * 1 / sos)

            # d / d\alpha -- f(x)
            dCLada = nu_CL * CLa_M
            dCDda = nu_CD * self.c_CD * CD_M * (2*self.c_CD_a2*alpha + self.c_CD_a1)
            dCMada = nu_CMa * self.c_CMa * CMa_M * (2*self.c_CMa_a2*alpha + self.c_CMa_a1)
            dCMqda = q * self.cbar / (2.*V) * CMq_M * (2*self.c_CMq_a2*alpha + self.c_CMq_a1)
            dCMdEda = - self.c_CMdE
            dCM0da = dCMada + dCMqda + dCMdEda
            # d / d\alpha -- g(x)
            dCLdEuda = 2 * self.c_CLdEu_a2 * alpha + self.c_CLdEu_a1

            # d / dh -- f(x)
            drhodh = self.c_rho_exp * rho
            dsosdh = 2*self.c_sos_h2*h + self.c_sos_h1
            dMdh = -V/(sos**2) * dsosdh
            dCLadh = nu_CL * CLa_a * self.c_CLa_Mm1 * (-1/(M**2)) * dMdh
            dCDdh = nu_CD * self.c_CD * CD_a * (2*self.c_CD_M2*M*dMdh + self.c_CD_M1*dMdh)
            dCMadh = nu_CMa * self.c_CMa * CMa_a * (-self.c_CMa_Mexp * torch.exp(M * self.c_CMa_Mexp) * dMdh)
            dCMqdh = q * self.cbar / (2. * V) * CMq_a * (self.c_CMq_M1 * dMdh)
            dCM0dh = dCMadh + dCMqdh
            # d / dh -- g(x)
            dCTdh = self.c_kx * self.c_CTu * self.c_kx_Mm1 * (-1. / M ** 2 * dMdh)

            # d / dq -- f(x)
            dCMqdq = self.cbar / (2. * V) * CMq_a * CMq_M

            # ***********************
            #
            # PARTIAL DERIVATIVE TERMS -- AERODYNAMIC FORCES/MOMENTS
            #

            # d / dV -- f(x)
            dLdV_f = 0.5 * rho * self.S * (2 * V * CL + V ** 2 * dCLadV)
            dDdV_f = 0.5 * rho * self.S * (2 * V * CD + V ** 2 * dCDdV)
            dMdV_f = 0.5 * rho * self.S * self.cbar * (2 * V * CM0 + V ** 2 * dCM0dV)
            # d / dV -- g(x)
            dTudV_g = 0.5 * rho * self.S * (2 * V * CTu + V ** 2 * dCTdV)

            # d / d\alpha -- f(x)
            dLda_f = qinf * self.S * dCLada
            dDda_f = qinf * self.S * dCDda
            dMda_f = qinf * self.S * self.cbar * dCM0da

            # d / dh -- f(x)
            dLdh_f = 0.5 * V ** 2 * self.S * (drhodh * CL + rho * dCLadh)
            dDdh_f = 0.5 * V ** 2 * self.S * (drhodh * CD + rho * dCDdh)
            dMdh_f = 0.5 * V ** 2 * self.S * self.cbar * (drhodh * CM0 + rho * dCM0dh)
            # d / dh -- g(x)
            dTudh_g = 0.5 * V ** 2 * self.S * (drhodh * CTu + rho * dCTdh)

            # d / dq -- f(x)
            dMdq_f = qinf * self.S * self.cbar * dCMqdq

            # ***********************
            #
            # PARTIAL DERIVATIVE TERMS -- d f(x) / dx
            #

            # d f_1(x) / dx -- V
            a11 = (- dDdV_f) / self.mref
            a12 = - self.mu * cg / (r ** 2)
            a13 = (-  dDdh_f) / self.mref + 2 * self.mu * sg / r ** 3
            a14 = (- dDda_f) / self.mref

            # d f_2(x) / dx -- \gamma
            a21 = dLdV_f / (self.mref * V) - L / (self.mref * V ** 2) + 2 * cg / r + (self.mu - V ** 2 * r) * cg / (V * r) ** 2
            a22 = (self.mu - V ** 2 * r) * sg / (V * r ** 2)
            a23 = dLdh_f / (self.mref * V) + V * cg / r ** 2 + 2 * (self.mu - V ** 2 * r) * cg / (V * r ** 3)
            a24 = dLda_f / (self.mref * V)

            # d f_3(x) / dx -- h
            a31 = sg
            a32 = V * cg

            # d f_4(x) / dx -- \alpha
            # Negation of d f_2(x) / dx -- \gamma

            # d f_5(x) / dx -- q
            a51 = dMdV_f / self.Iyy
            a53 = dMdh_f / self.Iyy
            a54 = dMda_f / self.Iyy
            a55 = dMdq_f / self.Iyy

            # ***********************
            #
            # PARTIAL DERIVATIVE TERMS -- d g(x) / dx
            #

            # d g_{1,1}(x) / dx -- V
            dg11dV = dTudV_g / self.mref * ca
            dg11da = Tu / self.mref * (-sa)
            dg11dh = dTudh_g / self.mref * ca

            # d g_{2,1}(x) / dx -- \gamma
            dg21dV = sa / self.mref * (dTudV_g / V - Tu / V ** 2)
            dg21da = Tu / (self.mref * V) * ca
            dg21dh = dTudh_g / (self.mref * V) * sa

            # d g_{2,2}(x) / dx -- \gamma
            dg22dV = 0.5 * rho * self.S / self.mref * CLdEu
            dg22da = 0.5 * rho * V * self.S / self.mref * dCLdEuda
            dg22dh = 0.5 * V * self.S / self.mref * CLdEu * drhodh

            # d g_{5,2}(x) / dx -- q
            dg52dV = rho * V * self.S * self.cbar / self.Iyy * self.c_CMdE
            # dg52da = 0
            dg52dh = 0.5 * V ** 2 * self.S * self.cbar / self.Iyy * self.c_CMdE * drhodh

            # ***********************
            #
            # PARTIAL DERIVATIVE -- d f / d x
            #
            # BEFORE applying transformation
            #

            dfdx = torch.zeros(n_samples, self.n_state, self.n_state).to(self.theta.device)

            # V
            dfdx[:, 0, 0] = a11.squeeze()
            dfdx[:, 0, 1] = a12.squeeze()
            dfdx[:, 0, 2] = a13.squeeze()
            dfdx[:, 0, 3] = a14.squeeze()
            # \gamma
            dfdx[:, 1, 0] = a21.squeeze()
            dfdx[:, 1, 1] = a22.squeeze()
            dfdx[:, 1, 2] = a23.squeeze()
            dfdx[:, 1, 3] = a24.squeeze()
            # h
            dfdx[:, 2, 0] = a31.squeeze()
            dfdx[:, 2, 1] = a32.squeeze()
            # \alpha
            dfdx[:, 3, 0] = -a21.squeeze()
            dfdx[:, 3, 1] = -a22.squeeze()
            dfdx[:, 3, 2] = -a23.squeeze()
            dfdx[:, 3, 3] = -a24.squeeze()
            dfdx[:, 3, 4] = 1.
            # q
            dfdx[:, 4, 0] = a51.squeeze()
            dfdx[:, 4, 2] = a53.squeeze()
            dfdx[:, 4, 3] = a54.squeeze()
            dfdx[:, 4, 4] = a55.squeeze()

            # ***********************
            #
            # PARTIAL DERIVATIVE -- d g / d x
            #
            # BEFORE applying transformation
            #

            # g_1(x) -- \delta_T
            dg1dx = torch.zeros(n_samples, self.n_state, self.n_state).to(self.theta.device)
            # g_2(x) -- \delta_E
            dg2dx = torch.zeros(n_samples, self.n_state, self.n_state).to(self.theta.device)

            # g_1(x) -- V
            dg1dx[:, 0, 0] = dg11dV.squeeze()
            dg1dx[:, 0, 2] = dg11dh.squeeze()
            dg1dx[:, 0, 3] = dg11da.squeeze()
            # g_1(x) -- \gamma
            dg1dx[:, 1, 0] = dg21dV.squeeze()
            dg1dx[:, 1, 2] = dg21dh.squeeze()
            dg1dx[:, 1, 3] = dg21da.squeeze()
            # g_1(x) -- \alpha
            dg1dx[:, 3, 0] = -dg21dV.squeeze()
            dg1dx[:, 3, 2] = -dg21dh.squeeze()
            dg1dx[:, 3, 3] = -dg21da.squeeze()

            # g_2(x) -- \gamma
            dg2dx[:, 1, 0] = dg22dV.squeeze()
            dg2dx[:, 1, 2] = dg22dh.squeeze()
            dg2dx[:, 1, 3] = dg22da.squeeze()
            # g_1(x) -- \alpha
            dg2dx[:, 3, 0] = -dg22dV.squeeze()
            dg2dx[:, 3, 2] = -dg22dh.squeeze()
            dg2dx[:, 3, 3] = -dg22da.squeeze()
            # g_1(x) -- q
            dg2dx[:, 4, 0] = dg52dV.squeeze()
            dg2dx[:, 4, 2] = dg52dh.squeeze()

            # ***********************
            #
            # PARTIAL DERIVATIVE -- d a(x) / d x
            #
            # i.e., AFTER applying transformation
            #

            # \partial a / \partial x -- i.e., in post-tranformation units
            # NOTE: here we compute the TRANSPOSE of the Jacobian (as did Lutter)
            dadx = torch.matmul(sx, torch.matmul(dfdx, invsx)).to(self.theta.device)
            #dadx = dfdx        # DEBUGGING
            dadx = dadx.transpose(2, 1).to(x.device)         # Return TRANSPOSE of Jacobian

            # ***********************
            #
            # PARTIAL DERIVATIVE -- d B(x) / d x
            #
            # i.e., AFTER applying transformation
            #

            # d g_1(x') / d x'
            dg1dxp = torch.matmul(sx, torch.matmul(dg1dx, invsx))
            # d g_2(x') / d x'
            dg2dxp = torch.matmul(sx, torch.matmul(dg2dx, invsx))

            # d B_1(x') / d x'
            # i.e., after applying control transformation
            dB1dxp = dg1dxp * invsu[0, 0] + dg1dxp * invsu[1, 0]
            # d B_2(x') / d x'
            # i.e., after applying control transformation
            dB2dxp = dg1dxp * invsu[0, 1] + dg2dxp * invsu[1, 1]

            # d B(x') / d x'
            dBdx = torch.zeros((x.shape[0], self.n_state, self.n_state, self.n_act), dtype=x.dtype, device=x.device)
            # Fill in the Jacobian
            dBdx[:, :, :, 0] = dB1dxp.to(x.device)
            dBdx[:, :, :, 1] = dB2dxp.to(x.device)
            dBdx = dBdx.transpose(2, 1)  # Return TRANSPOSE of Jacobian

            assert dadx.shape == (n_samples, self.n_state, self.n_state)
            assert dBdx.shape == (n_samples, self.n_state, self.n_state, self.n_act)
            out = (a, B, dadx, dBdx)

        if is_numpy:
            out = [array.cpu().detach().numpy() for array in out]

        return out

    def grad_dyn_theta(self, x):
        is_numpy = True if isinstance(x, np.ndarray) else False
        x = torch.from_numpy(x) if isinstance(x, np.ndarray) else x
        x = x.view(-1, self.n_state, 1)
        n_samples = x.shape[0]

        # Trim state, control
        xe = self.x_target.to(self.theta.device)
        # ue = self.ue

        # Shift state to reflect trim
        x = x + xe.view(1, self.n_state, 1)

        # DEBUGGING
        #x = x + torch.matmul(self.invsx, xe.view(1, self.n_state, 1))

        # Apply inverse transformation: deg -> rad
        sx = self.sx.to(self.theta.device)
        invsx = self.invsx.to(self.theta.device)
        #invsu = self.invsu.to(self.theta.device)
        x = torch.matmul(invsx, x)

        # Extract states (pre-transformation; i.e., in rad)
        V, g, h, alpha, q = x[:, 0], x[:, 1], x[:, 2], x[:, 3], x[:, 4]

        # ***********************
        #
        # AERODYNAMIC COEFFICIENTS
        #

        # Air density
        rho = self.c_rho * torch.exp(self.c_rho_exp * h)

        # Speed of sound
        sos = self.c_sos_h2 * h ** 2 + self.c_sos_h1 * h + self.c_sos_h0

        # Mach number
        M = V / sos

        # Dynamic pressure
        qinf = 0.5 * rho * V ** 2

        # Lift coefficient -- WITHOUT lift-elevator effects
        CLa_a = alpha
        CLa_M = self.c_CLa_M0 + self.c_CLa_Mm1 / M
        CLa_nom = CLa_a * CLa_M

        # Drag coefficient
        CD_a = self.c_CD_a2 * alpha ** 2 + self.c_CD_a1 * alpha + self.c_CD_a0
        CD_M = self.c_CD_M2 * M ** 2 + self.c_CD_M1 * M + self.c_CD_M0
        CD_nom = self.c_CD * CD_a * CD_M

        # Pitch moment coefficient -- basic increment C_{M,\alpha}
        CMa_a = self.c_CMa_a2 * alpha ** 2 + self.c_CMa_a1 * alpha + self.c_CMa_a0
        CMa_M = self.c_CMa_M0 - torch.exp(M * self.c_CMa_Mexp)
        CMa_nom = self.c_CMa * CMa_M * CMa_a

        # d f_1 / d \nu_{C_{D}}
        df1dnuCD = - 1. / self.mref * qinf * self.S * CD_nom

        # d f_2 / d \nu_{C_{L}}
        df2dnuCL = 1. / (self.mref * V) * qinf * self.S * CLa_nom

        # d f_3 / d \nu_{C_{M,\alpha}}
        df5dnuCM = 1. / (self.Iyy) * qinf * self.S * self.cbar * CMa_nom

        # Calculate d f / d \theta (i.e., in pre-transformation units)
        dfdth = torch.zeros(n_samples, self.n_parameter, self.n_state).to(x.device)

        # Fill in entries
        dfdth[:, 0, 1] = df2dnuCL.squeeze()
        dfdth[:, 1, 0] = df1dnuCD.squeeze()
        dfdth[:, 2, 4] = df5dnuCM.squeeze()

        #dadth = torch.zeros(n_samples, self.n_parameter, self.n_state).to(x.device)
        dadth = torch.matmul(sx, dfdth.transpose(2,1)).to(x.device).transpose(2,1)

        # d B / d \theta
        dBdth = torch.zeros(n_samples, self.n_parameter, self.n_state, self.n_act).to(x.device)

        out = dadth, dBdth

        if is_numpy:
            out = [array.numpy() for array in out]

        return out

    def cuda(self, device=None):
        self.theta_min = self.theta_min.cuda(device=device)
        self.theta = self.theta.cuda(device=device)
        self.theta_max = self.theta_max.cuda(device=device)

        #
        self.x_target = self.x_target.cuda(device=device)
        self.ue = self.ue.cuda(device=device)
        self.Vxtrans = self.Vxtrans.cuda(device=device)

        self.u_lim = self.u_lim.cuda(device=device)
        self.u_min = self.u_min.cuda(device=device)
        self.u_max = self.u_max.cuda(device=device)
        self.x_lim = self.x_lim.cuda(device=device)
        self.device = self.theta.device
        return self

    def cpu(self):
        self.theta_min = self.theta_min.cpu()
        self.theta = self.theta.cpu()
        self.theta_max = self.theta_max.cpu()

        #
        self.x_target = self.x_target.cpu()
        self.ue = self.ue.cpu()
        self.Vxtrans = self.Vxtrans.cpu()

        self.u_lim = self.u_lim.cpu()
        self.u_min = self.u_min.cpu()
        self.u_max = self.u_max.cpu()
        self.x_lim = self.x_lim.cpu()
        self.device = self.theta.device
        return self



class hsvintaug(BaseSystem):
    name = "hsvintaug"
    labels = ('V', 'gamma', 'z_V', 'z_gamma')

    def __init__(self, cuda=False, **kwargs):
        # Init BaseSystem
        super(hsvintaug, self).__init__()
        device = torch.device('cuda') if cuda else torch.device('cpu')

        # Init hsv as a property
        self.hsv = hsv(cuda=False, **kwargs)

        # Define the System:
        self.n_state = self.hsv.n_state + 2   # x = [x_p^T z^T]^T, where z = [z_V, z_\gamma] \in R^2 is the integrator bank
        self.nxp = self.hsv.n_state           # x_p = [V, \gamma, q, \alpha]
        self.n_act = self.hsv.n_act
        self.n_joint = self.hsv.n_joint
        self.n_parameter = self.hsv.n_parameter
        # Number of features for V(x)
        self.n_feature = self.hsv.n_feature + 2

        # Closed-loop design includes integral augmentation
        self.hasintaug = True

        # Do input transformation for V(x) (=1) or not (=0)
        self.doVxtrans = self.hsv.doVxtrans

        # V(x) transformation
        self.Vxtrans = torch.block_diag(self.hsv.Vxtrans, torch.eye(2))

        self.dousat = self.hsv.dousat

        # Continuous Joints:
        # Right now only one continuous joint is supported
        self.wrap, self.wrap_i = self.hsv.wrap, self.hsv.wrap_i

        # State Constraints:
        self.x_start = torch.cat([self.hsv.x_start, torch.tensor([0., 0.])], dim=0)
        self.x_start_var = torch.cat([self.hsv.x_start_var, torch.tensor([1.e-4, 1.e-4])], dim=0)
        self.x_lim = torch.cat([self.hsv.x_lim, torch.tensor([self.hsv.Vscl * 2500., 10.])], dim=0)

        #self.x_lim_eval = torch.cat([self.hsv.x_lim_eval, torch.tensor([self.hsv.Vscl * 1000., 10.])], dim=0)
        self.x_lim_eval = torch.cat([self.hsv.x_lim_eval, torch.tensor([self.hsv.Vscl * 2500., 10.])], dim=0)

        self.x_init = torch.cat([self.hsv.x_init, torch.tensor([self.hsv.Vscl*0.01, 0.01])], dim=0)
        self.x_init_train = torch.cat([self.hsv.x_init_train, torch.tensor([self.hsv.Vscl * 150., 2.5])], dim=0)


        self.u_lim = self.hsv.u_lim
        self.u_min = self.hsv.u_min
        self.u_max = self.hsv.u_max

        #
        # Lookup table grid parameters (for evaluating value function, policy after training)
        self.x_tbl_min = torch.cat([self.hsv.x_tbl_min, torch.tensor([self.hsv.Vscl * -1000., -5.])], dim=0)

        self.x_tbl_max = - self.x_tbl_min
        self.x_tbl_nxpts = torch.cat([self.hsv.x_tbl_nxpts, torch.tensor([7, 5])], dim=0).int()
        # 2D Lookup parameters
        self.x_tbl_min2 = self.hsv.x_tbl_min2
        self.x_tbl_max2 = self.hsv.x_tbl_max2
        self.x_tbl_nxpts2 = self.hsv.x_tbl_nxpts2

        # Thresholds to check when evaluating success percentage
        self.threshmat_x = self.hsv.threshmat_x

        # ***********************
        #
        # PLOT SETTINGS
        #
        #
        self.psett_y_ticks = self.hsv.psett_y_ticks
        self.psett_x_ticks = self.hsv.psett_x_ticks
        self.psett_y_tick_label = self.hsv.psett_y_tick_label
        self.psett_xlabel = self.hsv.psett_xlabel
        self.psett_ulabel = self.hsv.psett_ulabel


        # theta = \nu, where \nu is the uncertainty in d (Distance c.g. lies forward of wheel axles (m))
        # i.e., d = \nu * d_0
        self.theta_min = self.hsv.theta_min
        self.theta = self.hsv.theta
        self.theta_max = self.hsv.theta_max

        # Closed-loop design includes reference command pre-filter
        self.pf1nopf0 = self.hsv.pf1nopf0
        self.pfavec = self.hsv.pfavec

        # ***********************
        #
        # TRIM
        #

        self.x_target = torch.cat([self.hsv.x_target, torch.tensor([0., 0.])], dim=0)

        self.su = self.hsv.su
        self.invsu = self.hsv.invsu
        self.sx = torch.block_diag(self.hsv.sx, torch.eye(2))
        self.invsx = torch.block_diag(self.hsv.invsx, torch.eye(2))

        # Calculate equilibrium controls u_e
        self.ue = self.hsv.ue

        # LQR Baseline:
        out = self.dyn(torch.zeros(self.n_state).numpy(), gradient=True)
        self.A = out[2].reshape((1, self.n_state, self.n_state)).transpose((0, 2, 1))
        self.B = out[1].reshape((1, self.n_state, self.n_act))

        # Test dynamics:
        self.check_dynamics()

        self.device = None
        hsvintaug.cuda(self) if cuda else hsvintaug.cpu(self)

    def dyn(self, x, dtheta=None, gradient=False):
        cat = torch.cat

        is_numpy = True if isinstance(x, np.ndarray) else False
        # x = torch.from_numpy(x).to(self.theta.device) if isinstance(x, np.ndarray) else x
        #
        x = torch.from_numpy(x).to(self.theta.device) if isinstance(x, np.ndarray) else x.to(self.theta.device)
        x = x.view(-1, self.n_state, 1)
        n_samples = x.shape[0]

        # Update the dynamics parameters with disturbance:
        if dtheta is not None:
            dtheta = torch.from_numpy(dtheta).float() if isinstance(dtheta, np.ndarray) else dtheta
            dtheta = dtheta.view(-1, self.n_parameter, 1)
            assert dtheta.shape[0] in (1, n_samples)

            theta = self.theta + dtheta
            theta = torch.min(torch.max(theta, self.theta_min), self.theta_max)

        else:
            theta = self.theta
            theta = theta

        # ***********************
        #
        # FIRST DERIVATIVES (WITHOUT CONTROL)
        #

        # Entire plant state
        txp = x[:, 0:self.nxp].view(-1, self.nxp, 1)

        # Outputs: V, \gamma
        typ = x[:, 0:self.n_act].view(-1, self.n_act, 1)



        # Evaluate plant dynamics
        if gradient:
            ap, Bp, dapdxp, dBpdxp = self.hsv.dyn(txp, dtheta=dtheta, gradient=gradient)
        else:
            ap, Bp = self.hsv.dyn(txp, dtheta=dtheta, gradient=gradient)

        a = torch.cat([ap, typ], dim=1).view(-1, self.n_state, 1)

        B = torch.zeros(n_samples, self.n_state, self.n_act).to(self.theta.device)
        B[:, 0:self.nxp, :] = Bp


        assert a.shape == (n_samples, self.n_state, 1)
        assert B.shape == (n_samples, self.n_state, self.n_act)
        out = (a, B)

        if gradient:
            # Zero vectors of appropriate sizes
            # zeros, ones = torch.zeros_like(x[:, 1]), torch.ones_like(x[:, 1])

            # \partial a / \partial x
            # NOTE: here we compute the TRANSPOSE of the Jacobian (as did Lutter)
            dadx = torch.zeros(n_samples, self.n_state, self.n_state).to(self.theta.device)
            # NOTE: Need to reverse transpose on dapdxp = (d a_p / d x_p)^T
            dadx[:, 0:self.nxp, 0:self.nxp] = dapdxp.transpose(2, 1)
            dadx[:, self.nxp, 0] = 1.
            dadx[:, self.nxp+1, 1] = 1.
            dadx = dadx.transpose(2, 1)         # Return TRANSPOSE of Jacobian

            dBdx = torch.zeros((n_samples, self.n_state, self.n_state, self.n_act), dtype=x.dtype, device=x.device)
            dBdx[:, 0:self.nxp, 0:self.nxp, :] = dBpdxp.transpose(2, 1)
            dBdx = dBdx.transpose(2, 1)  # Return TRANSPOSE of Jacobian

            assert dadx.shape == (n_samples, self.n_state, self.n_state)
            assert dBdx.shape == (n_samples, self.n_state, self.n_state, self.n_act)
            out = (a, B, dadx, dBdx)

        if is_numpy:
            out = [array.cpu().detach().numpy() for array in out]

        return out

    def grad_dyn_theta(self, x):
        is_numpy = True if isinstance(x, np.ndarray) else False
        x = torch.from_numpy(x) if isinstance(x, np.ndarray) else x
        x = x.view(-1, self.n_state, 1)
        n_samples = x.shape[0]

        # Plant state
        xp = x[:, 0:self.nxp]

        # Evaluate plant gradients
        dapdth, dBpdth = self.hsv.grad_dyn_theta(xp)

        dadth = torch.zeros(n_samples, self.n_parameter, self.n_state).to(x.device)
        # Set plant gradients
        dadth[:, :, 0:self.nxp] = dapdth

        dBdth = torch.zeros(n_samples, self.n_parameter, self.n_state, self.n_act).to(x.device)
        # Set plant gradients
        dBdth[:, :, 0:self.nxp, :] = dBpdth


        out = dadth, dBdth

        if is_numpy:
            out = [array.numpy() for array in out]

        return out

    def cuda(self, device=None):
        self.theta_min = self.theta_min.cuda(device=device)
        self.theta = self.theta.cuda(device=device)
        self.theta_max = self.theta_max.cuda(device=device)

        #
        self.hsv.cuda()
        self.x_target = self.x_target.cuda(device=device)
        self.Vxtrans = self.Vxtrans.cuda(device=device)

        self.u_lim = self.u_lim.cuda(device=device)
        self.u_min = self.u_min.cuda(device=device)
        self.u_max = self.u_max.cuda(device=device)
        self.x_lim = self.x_lim.cuda(device=device)
        self.device = self.theta.device
        return self

    def cpu(self):
        self.theta_min = self.theta_min.cpu()
        self.theta = self.theta.cpu()
        self.theta_max = self.theta_max.cpu()

        #
        self.hsv.cpu()
        self.x_target = self.x_target.cpu()

        self.ue = self.ue.cpu()
        self.Vxtrans = self.Vxtrans.cpu()

        self.u_lim = self.u_lim.cpu()
        self.u_min = self.u_min.cpu()
        self.u_max = self.u_max.cpu()
        self.x_lim = self.x_lim.cpu()
        self.device = self.theta.device
        return self


class hsvquad(hsv):
    name = "hsv_quad"

    def __init__(self, Q, R, cuda=False, **kwargs):

        # Create the dynamics:
        super(hsv, self).__init__(cuda=cuda, **kwargs)
        #self.u_lim = torch.tensor([2.5, ])     # NOT SURE WHY LUTTER HAD THIS LINE

        # Change the Parameters:
        # self.Q = np.diag(np.array([1.e+0, 1.0e-1]))
        # self.R = np.array([[5.e-1]])

        assert Q.size == self.n_feature and np.all(Q > 0.0)
        self.Q = np.diag(Q).reshape((self.n_feature, self.n_feature))

        assert R.size == self.n_act and np.all(R > 0.0)
        self.R = np.diag(R).reshape((self.n_act, self.n_act))

        # Create the Reward Function:
        self.q = QuadraticCost(self.Q, cuda=cuda)
        self.r = QuadraticFunction(self.R, cuda=cuda, domain=(-100., +100.))


    def rwd(self, x, u):
        # -- Compute the feature:
        if self.doVxtrans:
            z = torch.matmul(self.Vxtrans, x)
        else:
            z = x
        return self.q(z) + self.r(u)

    def cuda(self, device=None):
        super(hsvquad, self).cuda(device=device)
        self.q.cuda(device=device)
        return self

    def cpu(self):
        super(hsvquad, self).cpu()
        self.q.cpu()
        return self


class hsvintaugquad(hsvintaug):
    name = "hsvintaug_quad"

    def __init__(self, Q, R, cuda=False, **kwargs):

        # Create the dynamics:
        super(hsvintaugquad, self).__init__(cuda=cuda, **kwargs)
        #self.u_lim = torch.tensor([2.5, ])     # NOT SURE WHY LUTTER HAD THIS LINE

        # Change the Parameters:
        # self.Q = np.diag(np.array([1.e+0, 1.0e-1]))
        # self.R = np.array([[5.e-1]])

        assert Q.size == self.n_feature and np.all(Q > 0.0)
        self.Q = np.diag(Q).reshape((self.n_feature, self.n_feature))

        assert R.size == self.n_act and np.all(R > 0.0)
        self.R = np.diag(R).reshape((self.n_act, self.n_act))

        # Create the Reward Function:
        self.q = QuadraticCost(self.Q, cuda=cuda)

        self.r = QuadraticFunction(self.R, cuda=cuda, domain=(-100., +100.))


    def rwd(self, x, u):
        # -- Compute the feature:
        if self.doVxtrans:
            z = torch.matmul(self.Vxtrans, x)
        else:
            z = x
        return self.q(z) + self.r(u)

    def cuda(self, device=None):
        super(hsvintaugquad, self).cuda(device=device)
        self.q.cuda(device=device)
        return self

    def cpu(self):
        super(hsvintaugquad, self).cpu()
        self.q.cpu()
        return self

if __name__ == "__main__":
    from deep_differential_network.utils import jacobian

    # GPU vs. CPU:
    cuda = True

    # Seed the test:
    seed = 20
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

    # Create system:
    sys = hsv()

    n_samples = 10
    x_lim = torch.from_numpy(sys.x_lim).float() if isinstance(sys.x_lim, np.ndarray) else sys.x_lim
    x_test = torch.distributions.uniform.Uniform(-x_lim, x_lim).sample((n_samples,))
    # x_test = torch.tensor([np.pi / 2., 0.5]).view(1, sys.n_state, 1)

    dtheta = torch.zeros(1, sys.n_parameter, 1)

    if cuda:
        sys, x_test, dtheta = sys.cuda(), x_test.cuda(), dtheta.cuda()

    ###################################################################################################################
    # Test dynamics gradient w.r.t. state:
    dadx_shape = (n_samples, sys.n_state, sys.n_state)
    dBdx_shape = (n_samples, sys.n_state, sys.n_state, sys.n_act)

    a, B, dadx, dBdx = sys.dyn(x_test, gradient=True)

    dadx_auto = torch.cat([jacobian(lambda x: sys.dyn(x)[0], x_test[i:i+1]) for i in range(n_samples)], dim=0)
    dBdx_auto = torch.cat([jacobian(lambda x: sys.dyn(x)[1], x_test[i:i+1]) for i in range(n_samples)], dim=0)

    err_a = (dadx_auto.view(dadx_shape) - dadx).abs().sum() / n_samples
    err_B = (dBdx_auto.view(dBdx_shape) - dBdx).abs().sum() / n_samples
    assert err_a <= 1.e-5 and err_B <= 1.e-6

    ###################################################################################################################
    # Test dynamics gradient w.r.t. model parameter:
    dadp_shape = (n_samples, sys.n_parameter, sys.n_state)
    dBdp_shape = (n_samples, sys.n_parameter, sys.n_state, sys.n_act)

    dadp, dBdp = sys.grad_dyn_theta(x_test)

    dadp_auto = torch.cat([jacobian(lambda x: sys.dyn(x_test[i], dtheta=x)[0], dtheta) for i in range(n_samples)], dim=0)
    dBdp_auto = torch.cat([jacobian(lambda x: sys.dyn(x_test[i], dtheta=x)[1], dtheta) for i in range(n_samples)], dim=0)

    err_a = (dadp_auto.view(dadp_shape) - dadp).abs().sum() / n_samples
    err_B = (dBdp_auto.view(dBdp_shape) - dBdp).abs().sum() / n_samples
    assert err_a <= 1.e-5 and err_B <= 1.e-6


