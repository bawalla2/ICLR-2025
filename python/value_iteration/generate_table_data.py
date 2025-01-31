import sys
import pathlib
currpath = str(pathlib.Path(__file__).parent.resolve())
#sys.path.append(currpath + f"/../")

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
from value_iteration.sim_policy import sim_policy, init_sys, init_value_funct

# -- for saving python arrays to .mat files
from scipy.io import savemat

def gen_table_data(n_eval, hyper, system, value_fun, runtime_vec, step_i):

    ################################################################################################################
    ################################################################################################################
    ################################################################################################################
    ################################################################################################################

    # *************************************************************************
    #
    # EVALUATE VALUE FUNCTION, POLICY FOR LOOKUP TABLE -- 2-D LOOKUP
    #
    # *************************************************************************

    cuda = torch.cuda.is_available()
    alg_name = "rFVI" if hyper['robust'] else "cFVI"

    # Initiate value-function policy:
    pi = ValueFunPolicy(system, value_fun)

    # Display generating data
    print("Generating 2D Lookup Table Data:")

    # Get lookup table grid params
    x_tbl_min2 = torch.from_numpy(system.x_tbl_min2).float() if isinstance(system.x_tbl_min2,
                                                                           np.ndarray) else system.x_tbl_min2
    x_tbl_max2 = torch.from_numpy(system.x_tbl_max2).float() if isinstance(system.x_tbl_max2,
                                                                           np.ndarray) else system.x_tbl_max2
    x_tbl_nxpts2 = torch.from_numpy(system.x_tbl_nxpts2).float() if isinstance(system.x_tbl_nxpts2,
                                                                               np.ndarray) else system.x_tbl_nxpts2

    # Sample the testing data
    tbl_grid = [linspace(x_tbl_min2[i].item(), x_tbl_max2[i].item(), x_tbl_nxpts2[i]) for i in range(2)]
    xtbl_grid = torch.meshgrid(tbl_grid, indexing='ij')
    xtbl_grid_cat = torch.cat([x.reshape(-1, 1) for x in xtbl_grid], dim=1).view(-1, 2, 1)
    if system.n_state > 2:
        xtbl_grid_cat_zeros = torch.cat([xtbl_grid_cat.view(-1, 2),
                                         torch.zeros(xtbl_grid_cat.shape[0], system.n_state - 2)],
                                        dim=1).view(-1, system.n_state, 1)
    else:
        xtbl_grid_cat_zeros = xtbl_grid_cat
    xtbl_grid_cat = xtbl_grid_cat.cuda() if cuda else xtbl_grid_cat
    xtbl_grid_cat_zeros = xtbl_grid_cat_zeros.cuda() if cuda else xtbl_grid_cat_zeros

    # Evaluate dynamics
    ax_tbl, Bx_tbl = system.dyn(xtbl_grid_cat_zeros)

    # Data storage
    V_tbl = torch.zeros(*x_tbl_nxpts2, 1)
    u_tbl = torch.zeros(*x_tbl_nxpts2, system.n_act)

    V_tbl = V_tbl.cuda() if cuda else V_tbl
    u_tbl = u_tbl.cuda() if cuda else u_tbl

    # Evaluation batch size
    n_samp_tble = xtbl_grid_cat.shape[0]
    n_batch = int(np.ceil(n_samp_tble / float(n_eval)))

    # Store data
    for scnt in range(n_batch):
        # Current range of indices
        indsl = np.linspace(scnt * n_eval, (scnt + 1) * n_eval - 1, num=n_eval, dtype=int)
        indsl = indsl[indsl < n_samp_tble]

        # Current states in the grid
        # xscnt = xtbl_grid_cat[scnt, :].view(-1, system.n_state, 1)
        # xscnt = xtbl_grid_cat[indsl].view(-1, system.n_state, 1)

        # Compute the value function, policy
        Vx, _, ux = pi(xtbl_grid_cat_zeros[indsl].view(-1, system.n_state, 1),
                       Bx_tbl[indsl].view(-1, system.n_state, system.n_act))
        # V_tbl_cat, dV0dx_tbl_cat, u_tbl_cat, dudx_tbl_cat = policy(xtbl_grid_cat[scnt, :], Bx_tbl[scnt, :], system.r, value_fun)

        # Get current linear index in the grid coords
        # inds = np.unravel_index(scnt, x_tbl_nxpts.numpy())
        inds = np.unravel_index(indsl, x_tbl_nxpts2.numpy())

        # Store the data in the appropriate grid location
        V_tbl[inds] = Vx.detach().view(-1, 1)
        u_tbl[inds] = ux.detach().view(-1, system.n_act)

    # Save data
    model_filei = currpath + f"/../data/{alg_name}_{system.name}_step_{step_i + 1:03d}_tbl_data_2D"
    model_file = currpath + f"/../data/{alg_name}_{system.name}_tbl_data_2D"
    mdict = {"x_tbl_min": x_tbl_min2, "x_tbl_max": x_tbl_max2, "x_tbl_nxpts": x_tbl_nxpts2, "V_tbl": V_tbl,
             "u_tbl": u_tbl, "runtime_vec": runtime_vec}
    mdictmat = {"x_tbl_min": x_tbl_min2.cpu().numpy(), "x_tbl_max": x_tbl_max2.cpu().numpy(),
                "x_tbl_nxpts": x_tbl_nxpts2.numpy(), "V_tbl": V_tbl.cpu().numpy(),
                "u_tbl": u_tbl.cpu().numpy(), "runtime_vec": runtime_vec.cpu().numpy()}
    torch.save(mdict, model_filei + '.torch')
    torch.save(mdict, model_file + '.torch')
    # Save data to .mat file
    savemat(model_filei + '.mat', mdictmat)
    savemat(model_file + '.mat', mdictmat)

    # *************************************************************************
    #
    # EVALUATE VALUE FUNCTION, POLICY FOR LOOKUP TABLE -- n-D LOOKUP
    #
    # *************************************************************************



    # Display generating data
    print("Generating n-D Lookup Table Data:")

    # Get lookup table grid params
    x_tbl_min = torch.from_numpy(system.x_tbl_min).float() if isinstance(system.x_tbl_min,
                                                                         np.ndarray) else system.x_tbl_min
    x_tbl_max = torch.from_numpy(system.x_tbl_max).float() if isinstance(system.x_tbl_max,
                                                                         np.ndarray) else system.x_tbl_max
    x_tbl_nxpts = torch.from_numpy(system.x_tbl_nxpts).float() if isinstance(system.x_tbl_nxpts,
                                                                             np.ndarray) else system.x_tbl_nxpts

    # Sample the testing data
    tbl_grid = [linspace(x_tbl_min[i].item(), x_tbl_max[i].item(), x_tbl_nxpts[i]) for i in
                range(system.n_state)]
    xtbl_grid = torch.meshgrid(tbl_grid, indexing='ij')
    xtbl_grid_cat = torch.cat([x.reshape(-1, 1) for x in xtbl_grid], dim=1).view(-1, system.n_state, 1)
    xtbl_grid_cat = xtbl_grid_cat.cuda() if cuda else xtbl_grid_cat

    # Evaluate dynamics
    ax_tbl, Bx_tbl = system.dyn(xtbl_grid_cat)

    # Data storage
    V_tbl = torch.zeros(*x_tbl_nxpts, 1)
    u_tbl = torch.zeros(*x_tbl_nxpts, system.n_act)

    V_tbl = V_tbl.cuda() if cuda else V_tbl
    u_tbl = u_tbl.cuda() if cuda else u_tbl

    # Evaluation batch size
    n_samp_tble = xtbl_grid_cat.shape[0]
    n_batch = int(np.ceil(n_samp_tble / float(n_eval)))

    # Store data
    for scnt in range(n_batch):
        # Current range of indices
        indsl = np.linspace(scnt * n_eval, (scnt + 1) * n_eval - 1, num=n_eval, dtype=int)
        indsl = indsl[indsl < n_samp_tble]

        # Current states in the grid
        # xscnt = xtbl_grid_cat[scnt, :].view(-1, system.n_state, 1)
        # xscnt = xtbl_grid_cat[indsl].view(-1, system.n_state, 1)

        # Compute the value function, policy
        Vx, _, ux = pi(xtbl_grid_cat[indsl].view(-1, system.n_state, 1),
                       Bx_tbl[indsl].view(-1, system.n_state, system.n_act))
        # V_tbl_cat, dV0dx_tbl_cat, u_tbl_cat, dudx_tbl_cat = policy(xtbl_grid_cat[scnt, :], Bx_tbl[scnt, :], system.r, value_fun)

        # Get current linear index in the grid coords
        # inds = np.unravel_index(scnt, x_tbl_nxpts.numpy())
        inds = np.unravel_index(indsl, x_tbl_nxpts.numpy())

        # Store the data in the appropriate grid location
        V_tbl[inds] = Vx.detach().view(-1, 1)
        u_tbl[inds] = ux.detach().view(-1, system.n_act)

    # Save data
    model_filei = currpath + f"/../data/{alg_name}_{system.name}_step_{step_i + 1:03d}_tbl_data"
    model_file = currpath + f"/../data/{alg_name}_{system.name}_tbl_data"
    mdict = {"x_tbl_min": x_tbl_min, "x_tbl_max": x_tbl_max, "x_tbl_nxpts": x_tbl_nxpts, "V_tbl": V_tbl,
             "u_tbl": u_tbl, "runtime_vec": runtime_vec}
    mdictmat = {"x_tbl_min": x_tbl_min.cpu().numpy(), "x_tbl_max": x_tbl_max.cpu().numpy(),
                "x_tbl_nxpts": x_tbl_nxpts.numpy(), "V_tbl": V_tbl.cpu().numpy(),
                "u_tbl": u_tbl.cpu().numpy(), "runtime_vec": runtime_vec.cpu().numpy()}
    torch.save(mdict, model_filei + '.torch')
    torch.save(mdict, model_file + '.torch')
    # Save data to .mat file
    savemat(model_filei + '.mat', mdictmat)
    savemat(model_file + '.mat', mdictmat)

    print("DONE Generating Lookup Table Data")

if __name__ == "__main__":

    # Args
    seed = 0
    #iter = 50
    iter = 25
    system_name = 'vamvoudakis2010_quad'
    #system_name = 'Pendulum_QuadCost'
    #system_name = 'hsvintaug_quad'
    #alg_name = 'cFVI'
    alg_name = 'rFVI'
    n_eval = 1000

    model_file = f"../data/{system_name}/{alg_name}_seed_{seed:03d}_step_{iter:03d}.torch"

    # Load data
    data = torch.load(model_file, map_location=torch.device('cpu'))

    # Extract data
    hyper = data['hyper']
    state_dict = data['state_dict']
    runtime_vec = data['runtime_vec']

    # Build the dynamical system:
    sys = init_sys(hyper)

    # Construct Value Function:
    val_fun = init_value_funct(hyper, sys, state_dict)
    device = val_fun.device

    gen_table_data(n_eval, hyper, sys, val_fun, runtime_vec, iter-1)

