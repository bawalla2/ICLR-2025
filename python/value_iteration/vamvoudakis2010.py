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

class vamvoudakis2010(BaseSystem):
    name = "vamvoudakis2010"
    labels = ('x_1', 'x_2')

    def __init__(self, cuda=False, **kwargs):
        super(vamvoudakis2010, self).__init__()
        device = torch.device('cuda') if cuda else torch.device('cpu')

        # Define Duration:
        self.T = kwargs.get("T", 10.0)
        self.dt = kwargs.get("dt", 1./500.)

        # Do input transformation for V(x) (=1) or not (=0)
        # NOTE: Not implemented on this system
        self.doVxtrans = False

        # Do control saturation (=1) or not (=0)
        #self.dousat = True
        self.dousat = False

        # Define the System:
        self.n_state = 2
        self.n_act = 1
        self.n_joint = 1
        self.n_parameter = 1

        # Continuous Joints:
        # Right now only one continuous joint is supported
        self.wrap, self.wrap_i = False, 0


        # State Constraints:
        self.x_start = torch.tensor([1., 0.])     # Central IC for generating trajectories to plot during eval
        self.x_start_var = torch.tensor([1.e-6, 1.e-6])   # Variance for generating trajectories to plot during eval
        self.x_lim = torch.tensor([2.5, 2.5])
        self.x_lim_eval = torch.tensor([100., 100.])
        # Expected return evals sampled in +/- this
        self.x_init = torch.tensor([1., 1.])
        self.x_init_train = self.x_lim

        # Control saturation
        self.u_lim = torch.tensor([100.])
        self.u_min = -self.u_lim
        self.u_max = self.u_lim


        #
        # Lookup table grid parameters (for evaluating value function, policy after training)
        self.x_tbl_min = - torch.tensor([-1., -1.])
        self.x_tbl_max = - self.x_tbl_min
        self.x_tbl_nxpts = torch.tensor([10, 10]).int()
        # 2D Lookup parameters
        self.x_tbl_min2 = torch.tensor([-1., -1.])
        self.x_tbl_max2 = -self.x_tbl_min2
        self.x_tbl_nxpts2 = torch.tensor([150, 150]).int()

        # Thresholds to check when evaluating success percentage (post-transformation units)
        self.threshmat_x = torch.tensor([
            [0., 0.1, 5.]])

        # ***********************
        #
        # PLOT SETTINGS
        #
        #
        self.psett_y_ticks = [-1., -0.5, 0.0, 0.5, 1.]
        self.psett_x_ticks = [-1., -0.5, 0.0, 0.5, 1.]
        self.psett_y_tick_label = [r"$-1$", r"$-0.5$", r"$0$", r"$+0.5$", r"$+1$"]
        self.psett_xlabel = (r"$x_1$", r"$x_2$")
        self.psett_ulabel = (r"$u$")

        # theta = \nu * d_0
        self.theta_min = torch.tensor([0.75]).to(device).view(1, self.n_parameter, 1)
        self.theta = torch.tensor([1.]).to(device).view(1, self.n_parameter, 1)
        self.theta_max = torch.tensor([1.25]).to(device).view(1, self.n_parameter, 1)

        # ***********************
        #
        # MODEL CONSTANTS
        #

        # Number of features
        self.n_feature = self.n_state

        self.sx = torch.eye(self.n_state)
        self.invsx = torch.eye(self.n_state)
        self.su = torch.eye(self.n_act)
        self.invsu = torch.eye(self.n_act)


        # ***********************
        #
        # TRIM
        #

        # Equilibrium state x_e
        xe = torch.tensor([0., 0.])
        self.x_target = xe

        # Equilibrium controls u_e
        self.ue = torch.tensor([0.])

        # LQR Baseline:
        out = self.dyn(torch.zeros(self.n_state).numpy(), gradient=True)
        self.A = out[2].reshape((1, self.n_state, self.n_state)).transpose((0, 2, 1))
        self.B = out[1].reshape((1, self.n_state, self.n_act))

        # Test dynamics:
        self.check_dynamics()

        self.device = None
        vamvoudakis2010.cuda(self) if cuda else vamvoudakis2010.cpu(self)

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

        # Trim state, control
        xe = self.x_target.to(self.theta.device)
        # ue = self.ue

        # Shift state to reflect trim
        x = x + xe.view(1, self.n_state, 1)

        # Extract states
        x1, x2, = x[:, 0], x[:, 1]

        # ***********************
        #
        # DYNAMICS
        #

        c2x1p2 = torch.cos(2. * x1) + 2.

        f22xnom = - 0.5 * x2 * (1. - c2x1p2 ** 2)

        f1x = - x1 + x2
        f2x = -0.5 * x1 + theta[:, 0] * f22xnom

        # Make drift dynamics f(x) -- pre-transformation units
        fx = torch.zeros(x.shape[0], self.n_state, 1).to(self.theta.device)
        fx[:, 0, 0] = f1x.squeeze()
        fx[:, 1, 0] = f2x.squeeze()

        # Make input gain matrix g(x) -- with lift-elevator effects -- pre-transformation units
        gx = torch.zeros(x.shape[0], self.n_state, self.n_act).to(self.theta.device)
        gx[:, 1, 0] = c2x1p2.squeeze()

        # Evaluate drift dynamics -- post-transformation units
        a = fx.to(self.theta.device)

        # Evaluate input gain matrix g(x) -- post-transformation units
        B = gx

        assert a.shape == (n_samples, self.n_state, 1)
        assert B.shape == (n_samples, self.n_state, self.n_act)
        out = (a, B)

        if gradient:
            #zeros, ones = torch.zeros_like(x[:, 1]), torch.ones_like(x[:, 1])

            # ***********************
            #
            # PARTIAL DERIVATIVE TERMS -- d f(x) / dx
            #

            # d f_1(x) / dx
            a11 = torch.tensor([- 1.])
            a12 = torch.tensor([1.])

            # d f_2(x) / dx
            a21_nonlin_nom = - 0.5 * x2 * (-2. * c2x1p2 * 2. * (-torch.sin(2. * x1)))
            a22_nonlin_nom = - 0.5 * (1 - c2x1p2 ** 2)
            a21 = -0.5 + theta[:, 0] * a21_nonlin_nom
            a22 = theta[:, 0] * a22_nonlin_nom


            # ***********************
            #
            # PARTIAL DERIVATIVE TERMS -- d g(x) / dx
            #

            # d g_{2,1}(x) / dx
            dg21dx1 = - 2. * torch.sin(2. * x1)



            # ***********************
            #
            # PARTIAL DERIVATIVE -- d f / d x
            #
            # BEFORE applying transformation
            #

            dfdx = torch.zeros(n_samples, self.n_state, self.n_state).to(self.theta.device)

            # x_1
            dfdx[:, 0, 0] = a11.squeeze()
            dfdx[:, 0, 1] = a12.squeeze()
            # x_2
            dfdx[:, 1, 0] = a21.squeeze()
            dfdx[:, 1, 1] = a22.squeeze()


            # ***********************
            #
            # PARTIAL DERIVATIVE -- d g / d x
            #
            # BEFORE applying transformation
            #

            # g_1(x)
            dg1dx = torch.zeros(n_samples, self.n_state, self.n_state).to(self.theta.device)

            # g_1(x)
            dg1dx[:, 1, 0] = dg21dx1.squeeze()


            # ***********************
            #
            # PARTIAL DERIVATIVE -- d a(x) / d x
            #
            # i.e., AFTER applying transformation
            #

            # \partial a / \partial x -- i.e., in post-tranformation units
            # NOTE: here we compute the TRANSPOSE of the Jacobian (as did Lutter)
            dadx = dfdx.to(self.theta.device)
            dadx = dadx.transpose(2, 1).to(x.device)         # Return TRANSPOSE of Jacobian

            # ***********************
            #
            # PARTIAL DERIVATIVE -- d B(x) / d x
            #

            # d B(x) / d x
            dBdx = torch.zeros((x.shape[0], self.n_state, self.n_state, self.n_act), dtype=x.dtype, device=x.device)
            # Fill in the Jacobian
            dBdx[:, :, :, 0] = dg1dx.to(x.device)
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

        # Extract states
        x1, x2, = x[:, 0], x[:, 1]

        # For d f_2 / d \theta
        c2x1p2 = torch.cos(2. * x1) + 2.
        f22xnom = - 0.5 * x2 * (1. - c2x1p2 ** 2)


        # Calculate d f / d \theta (i.e., in pre-transformation units)
        dfdth = torch.zeros(n_samples, self.n_parameter, self.n_state).to(x.device)

        # Fill in entries
        dfdth[:, 0, 1] = f22xnom.squeeze()

        dadth = dfdth

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

        self.u_lim = self.u_lim.cpu()
        self.u_min = self.u_min.cpu()
        self.u_max = self.u_max.cpu()
        self.x_lim = self.x_lim.cpu()
        self.device = self.theta.device
        return self





class vamvoudakis2010quad(vamvoudakis2010):
    name = "vamvoudakis2010_quad"

    def __init__(self, Q, R, cuda=False, **kwargs):

        # Create the dynamics:
        super(vamvoudakis2010quad, self).__init__(cuda=cuda, **kwargs)

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
        super(vamvoudakis2010quad, self).cuda(device=device)
        self.q.cuda(device=device)
        return self

    def cpu(self):
        super(vamvoudakis2010quad, self).cpu()
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
    sys = vamvoudakis2010()

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


