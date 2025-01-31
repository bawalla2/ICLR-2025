import time
import torch
import numpy as np

#from value_iteration.eval_pol_undiscounted import eval_pol_undiscounted

try:
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm

except ImportError:
    pass

from deep_differential_network.replay_memory import PyTorchReplayMemory, PyTorchTestMemory
from value_iteration.update_value_function import update_value_function, eval_memory, policy
from value_iteration.value_function import ValueFunctionMixture
from value_iteration.sample_rollouts import sample_data, ValueFunPolicy
from value_iteration.utils import linspace, add_nan

from value_iteration.utils import trange

# -- for saving python arrays to .mat files
from scipy.io import savemat

def sim_policy(val_fun, sys, x0, config):

    with torch.no_grad():

        T = config.get("T", 10.)
        fs = config.get("fs", 100.)

        dt = 1. / fs

        n_steps = int(T / dt)
        n_sim = int(x0.shape[0])

        doclip =  config.get("doclip", False)
        save_xu_data = not config.get("delete_xu_data", False)
        downsample = int(1. / dt / config.get("fs_return", 50.))
        device = val_fun.device

        x_lim_eval = sys.x_lim_eval if hasattr(sys, 'x_lim_eval') else sys.x_lim
        x_lim_eval = x_lim_eval.float().to(device).view(1, sys.n_state, 1)
        u_lim = sys.u_lim.float().to(device).view(1, sys.n_act, 1)

        # Determine modeling error parameters
        theta_mode = config.get("theta_mode", "const")
        if theta_mode == 'const':
            dtheta = config.get("dtheta", torch.zeros(1, sys.n_parameter, 1)).to(device)
        else:
            dtheta = None

        # Check for pre-filter
        pf1nopf0 = config['pfavec'] is not None
        if pf1nopf0:
            pfavec = torch.tensor([float(xi) for xi in config['pfavec'].split(',')]).view(1, sys.n_act, 1).to(device)
            pfICtype = config['pfICtype'] if config['pfICtype'] is not None else '0'
            if pfICtype == '0':
                pfIC = torch.zeros(1, sys.n_act, 1).to(device)
            else:
                pfIC = x0[:, 0:sys.n_act, :].to(device)


        # Reference command settings
        hasrt_Avec = config['rt_Avec'] is not None
        rt_Avec = torch.tensor([float(xi) for xi in config['rt_Avec'].split(',')]).view(1, sys.n_act, 1) if hasrt_Avec else torch.zeros(1, sys.n_act, 1)
        rt_Avec = rt_Avec.to(device)
        hasrt = hasrt_Avec or pf1nopf0

        # Check for x thresholds (=1) or not (=0)
        do_thresh_x = config.get('do_thresh_x', False)
        if do_thresh_x:
            threshmat_x = sys.threshmat_x
            numthresh_x = threshmat_x.shape[0]
            successmat_x = torch.zeros(n_sim, numthresh_x, 1)
        else:
            successmat_x = torch.zeros(1).view(1, 1, 1)
        successmat_x = successmat_x.to(device)


        # Initiate value-function policy:
        pi = ValueFunPolicy(sys, val_fun)

        # -- check if control saturation applied
        dousat = sys.dousat if hasattr(sys, 'dousat') else False

        if dousat:
            u_min = torch.from_numpy(sys.u_min).float() if isinstance(sys.u_min, np.ndarray) else sys.u_min
            u_min = u_min.to(val_fun.device).view(1, sys.n_act, 1)
            u_max = torch.from_numpy(sys.u_max).float() if isinstance(sys.u_max, np.ndarray) else sys.u_max
            u_max = u_max.to(val_fun.device).view(1, sys.n_act, 1)

        # Init storage
        #x, a, V, dVdx, dVdt, B, u, r = [], [], [], [], [], [], [], []
        x, u, r, tvec = [], [], [], []

        # DEBUGGING
        #x0[0,:,:] = x_init

        xn = x0

        xn = torch.min(torch.max(xn, -x_lim_eval), x_lim_eval)

        # Check if closed-loop design includes integral augmentation
        hasintaug = sys.hasintaug if hasattr(sys, 'hasintaug') else False

        # If pre-filter used, append pre-filter ICs
        if pf1nopf0:
            xn_sim = torch.cat((xn, pfIC), dim=1)

        t = 0.0

        if save_xu_data:
            if pf1nopf0:
                x.append(xn_sim)
            else:
                x.append(xn)

        tvec.append(torch.tensor(t))        

        for i in trange(int(n_steps), prefix=f"Sample Datapoints", ncols=100):

            # Compute dynamics:
            a_t, B_t = sys.dyn(xn, dtheta=dtheta)
            #a.append(a_t)
            #B.append(B_t)

            # Get pre-filter states
            if pf1nopf0:
                trfp = xn_sim[:, sys.n_state:sys.n_state + sys.n_act, :]
            else:
                trfp = rt_Avec

            # Insert reference command
            if hasrt:
                ty = xn[:, 0:sys.n_act, :]
                te = trfp - ty
                tyxn = xn.clone().detach()
                tyxn[:, 0:sys.n_act, :] = - te
                if hasintaug:
                    a_t[:, sys.n_state-sys.n_act:sys.n_state, :] = - te
            else:
                tyxn = xn


            # Compute optimal action:
            #Vi, dVidx, ui = pi(x[-1], B[-1])
            Vi, dVidx, ui = pi(tyxn, B_t)

            #if dousat:
            #    ui = torch.min(torch.max(ui, u_min), u_max)
            if doclip:
                ui = torch.min(torch.max(ui, -u_lim), u_lim)

            #ui = torch.min(torch.max(ui, -u_lim), u_lim)
            if save_xu_data:
                u.append(ui)

            #V.append(Vi)
            #dVdx.append(dVidx)
            ui_p = ui + sys.ue.to(device).view(1, sys.n_act, 1)

            # Compute reward:
            #r.append(-dt * sys.rwd(xn, ui))
            r.append(-dt * sys.rwd(tyxn, ui))

            # Compute next step:
            xd = (a_t + torch.matmul(B_t, ui_p)).view(-1, sys.n_state, 1)
            xn = xn + dt * xd

            # Compute pre-filter dynamics:
            if pf1nopf0:
                pfdot = torch.matmul(torch.diag(pfavec.view(sys.n_act)), rt_Avec - trfp)
                xn_sim = torch.cat((xn, trfp + dt * pfdot), dim=1)

            # Compute dVdt
            #dVdt.append(torch.matmul(dVidx.transpose(dim0=1, dim1=2), xd))

            if sys.wrap:
                xn[:, sys.wrap_i] = torch.remainder(xn[:, sys.wrap_i] + np.pi, 2 * np.pi) - np.pi

            # Clip state
            if doclip:
                xn = torch.min(torch.max(xn, -x_lim_eval), x_lim_eval)

            t += dt

            # Append state, time
            if save_xu_data:
                if pf1nopf0:
                    x.append(xn_sim)
                else:
                    x.append(xn)

            tvec.append(torch.tensor(t))


        # Rewards:
        r = torch.stack(r, dim=0)
        R = torch.sum(r, dim=0).squeeze(0)

        # Stack x, u, t data
        if save_xu_data:
            x = torch.stack(x, dim=0)
            u = torch.stack(u, dim=0)
        else:
            if pf1nopf0:
                x = xn_sim.view(1, -1, sys.n_state + sys.n_act, 1)
            else:
                x = xn.view(1, -1, sys.n_state, 1)
            u = ui.view(1, -1, sys.n_act, 1)
        tvec = torch.stack(tvec, dim=0).view(-1, 1, 1)

        # Check x thresholds
        if do_thresh_x:
            for i in range(0, numthresh_x):
                currthreshind = int(threshmat_x[i, 0])
                currthresh = threshmat_x[i, 1]
                currt = threshmat_x[i, 2]
                # Get the minimum time index >= current time
                gtt = (tvec.view(-1) >= currt).nonzero()
                if gtt.numel() == 0:
                    currtind = -1
                else:
                    currtind = int(gtt[0])
                # Perform check
                successmat_x[:, i, :] = (x[currtind, :, currthreshind, :] <= currthresh) & (x[currtind, :, currthreshind, :] >= -currthresh)

        # Down sample:
        if save_xu_data:
            x = x[:-1:downsample]
            u = u[::downsample]
            tvec = tvec[:-1:downsample]
        r = r[::downsample]

        # Transpose output
        x = x.transpose(0, 1)
        u = u.transpose(0, 1)
        r = r.transpose(0, 1)

        out = [xi for xi in [x, u, r, R, tvec, successmat_x]]

        return out

def init_sys(hyper):
    cuda = torch.cuda.is_available()
    # Build the dynamical sys:
    Q = np.array([float(x) for x in hyper['state_cost'].split(',')])
    R = np.array([float(x) for x in hyper['action_cost'].split(',')])
    sys = hyper['system_class'](Q, R, cuda=cuda, **hyper)
    return sys

def init_value_funct(hyper, sys, state_dict):
    cuda = torch.cuda.is_available()
    # Construct Value Function:
    feature = torch.zeros(sys.n_state)
    if sys.wrap:
        feature[sys.wrap_i] = 1.0
    val_fun_kwargs = {'feature': feature}
    val_fun = ValueFunctionMixture(sys.n_feature, **val_fun_kwargs, **hyper)
    val_fun.load_state_dict(state_dict)
    val_fun = val_fun.cuda() if cuda else val_fun.cpu()
    return val_fun