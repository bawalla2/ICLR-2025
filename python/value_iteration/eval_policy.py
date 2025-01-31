import sys
import pathlib
currpath = str(pathlib.Path(__file__).parent.resolve())
currpath_base = currpath + f"/../"
sys.path.append(currpath_base)

import torch
import argparse
import numpy as np

try:
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm

except ImportError:
    pass

from value_iteration.sim_policy import sim_policy, init_sys, init_value_funct
from value_iteration.utils import linspace, add_nan
from scipy.io import savemat

#if __name__ == "__main__":
def _eval_policy(arg_list):
    parser = argparse.ArgumentParser()
    parser.add_argument("-eval_mode", dest='eval_mode', type=str)
    parser.add_argument("-x0", dest='x0', type=str)
    parser.add_argument("-file_data", dest='file_data', type=str)
    parser.add_argument("-file_out", dest='file_out', type=str)
    parser.add_argument("-T", dest='T', type=float)
    parser.add_argument("-fs", dest='fs', type=float)
    parser.add_argument("-fs_return", dest='fs_return', type=float)
    parser.add_argument("-n_sim", dest='n_sim', type=int)
    parser.add_argument("-grid_2D_min", dest='grid_2D_min', type=str)
    parser.add_argument("-grid_2D_max", dest='grid_2D_max', type=str)
    parser.add_argument("-grid_2D_npts", dest='grid_2D_npts', type=str)
    parser.add_argument("-theta_mode", dest='theta_mode', type=str)
    parser.add_argument("-dtheta", dest='dtheta', type=str)
    parser.add_argument("-seed", dest='seed', type=int)
    parser.add_argument("-delete_xu_data", dest='delete_xu_data', action='store_true')
    parser.add_argument("-do_thresh_x", dest='do_thresh_x', action='store_true')
    parser.add_argument("-do_clip", dest='do_clip', action='store_true')
    parser.add_argument("-rt_Avec", dest='rt_Avec', type=str)
    parser.add_argument("-pfavec", dest='pfavec', type=str)
    parser.add_argument("-pfICtype", dest='pfICtype', type=str)
    args = parser.parse_args(arg_list)

    cuda = torch.cuda.is_available()

    # Unpack args
    eval_mode = args.eval_mode
    file_data = args.file_data
    file_out = args.file_out
    T = args.T
    fs = args.fs
    fs_return = args.fs_return
    theta_mode = args.theta_mode
    delete_xu_data = args.delete_xu_data
    do_thresh_x = args.do_thresh_x
    do_clip = args.do_clip

    # Load data
    data = torch.load(file_data, map_location=torch.device('cpu'))

    # Save data (=1) or not (=0)
    savedata = file_out is not None

    # Extract data
    hyper = data['hyper']
    state_dict = data['state_dict']

    # Build the dynamical system:
    sys = init_sys(hyper)

    # Construct Value Function:
    val_fun = init_value_funct(hyper, sys, state_dict)
    device = val_fun.device

    # ICs
    if eval_mode == 'single':
        n_sim = 1
        x0 = torch.tensor([float(xi) for xi in args.x0.split(',')]).view(1, sys.n_state, 1).to(device)
    elif eval_mode == 'rand_U':
        n_sim = args.n_sim
        seed = args.seed
        torch.manual_seed(seed)
        x_init = sys.x_init.float().view(1, sys.n_state, 1)
        dist_x = torch.distributions.uniform.Uniform(-x_init, x_init)
        x0 = dist_x.sample((n_sim,)).view(-1, sys.n_state, 1).to(device)
    elif eval_mode == 'grid_2D':
        grid_2D_min = torch.tensor([float(xi) for xi in args.grid_2D_min.split(',')]) if args.grid_2D_min is not None else sys.x_tbl_min2
        grid_2D_max = torch.tensor([float(xi) for xi in args.grid_2D_max.split(',')]) if args.grid_2D_max is not None else sys.x_tbl_max2
        grid_2D_npts = torch.tensor([int(xi) for xi in args.grid_2D_npts.split(',')]) if args.grid_2D_npts is not None else sys.x_tbl_nxpts2
        #grid_2D_min = torch.from_numpy(grid_2D_min).float() if isinstance(grid_2D_min) else grid_2D_min
        # Sample the testing data
        tbl_grid = [linspace(grid_2D_min[i], grid_2D_max[i], grid_2D_npts[i]) for i in range(2)]
        xtbl_grid = torch.meshgrid(tbl_grid, indexing='ij')
        xtbl_grid_cat = torch.cat([x.reshape(-1, 1) for x in xtbl_grid], dim=1).view(-1, 2, 1)
        if sys.n_state > 2:
            xtbl_grid_cat_zeros = torch.cat([xtbl_grid_cat.view(-1, 2),
                                             torch.zeros(xtbl_grid_cat.shape[0], sys.n_state - 2)],
                                            dim=1).view(-1, sys.n_state, 1)
        else:
            xtbl_grid_cat_zeros = xtbl_grid_cat
        xtbl_grid_cat = xtbl_grid_cat.cuda() if cuda else xtbl_grid_cat
        xtbl_grid_cat_zeros = xtbl_grid_cat_zeros.cuda() if cuda else xtbl_grid_cat_zeros
        x0 = xtbl_grid_cat_zeros
        n_sim = x0.shape[0]
    else:
        raise Exception("Please choose a valid evaluation type")

    n_sim = int(n_sim)

    # Modeling error parameters \theta
    if theta_mode == 'const':
        dtheta = torch.tensor([float(xi) for xi in args.dtheta.split(',')])

    # Check for pre-filter
    pf1nopf0 = args.pfavec is not None

    # Configuration for sampling trajectories from the system
    run_config = {'T': T, 'fs': fs, 'fs_return': fs_return, 'theta_mode': theta_mode, 'delete_xu_data': delete_xu_data,
                  'rt_Avec': args.rt_Avec, 'pfavec': args.pfavec, 'pfICtype': args.pfICtype, 'do_thresh_x': do_thresh_x,
                  'doclip': do_clip}
    if theta_mode == 'const':
        run_config['dtheta'] = dtheta

    with torch.no_grad():

        out = sim_policy(val_fun, sys, x0, run_config)

        x = out[0]
        u = out[1]
        r = out[2]
        R = out[3].squeeze()
        tvec = out[4]
        successmat_x = out[5]
        #R_mean = torch.mean(R).item()
        #R_std = torch.std(R).item()

        # Shift x, u by trim, apply transformations
        x[:, :, 0:sys.n_state, :] = torch.matmul(sys.invsx.to(device), x[:, :, 0:sys.n_state, :] + sys.x_target.to(device).view(1,sys.n_state,1))
        u = torch.matmul(sys.invsu.to(device), u + sys.ue.to(device).view(1,sys.n_act,1))

        # Reshape output if is a single simulation
        if eval_mode == 'single':
            if pf1nopf0:
                x = x.view(-1, sys.n_state + sys.n_act)
            else:
                x = x.view(-1, sys.n_state)
            u = u.view(-1, sys.n_act)

        if savedata:

            # Cast to numpy for saving
            x_np = x.cpu().numpy()
            u_np = u.cpu().numpy()
            R_np = R.cpu().numpy()
            tvec_np = tvec.cpu().numpy()
            successmat_x_np = successmat_x.cpu().numpy()

            # Format output
            if eval_mode == 'grid_2D':
                R_np = np.reshape(R_np, grid_2D_npts.cpu().numpy())
                if do_thresh_x:
                    successmat_x_np = np.reshape(successmat_x_np, torch.cat([grid_2D_npts.view(-1, 1), torch.tensor(int(sys.threshmat_x.shape[0])).view(-1, 1)]).squeeze().cpu().numpy())
                mdictmat = {'R': R_np, 'successmat_x': successmat_x_np}
            else:
                mdictmat = {'x': x_np, 'u': u_np, 'R': R_np, 'tvec': tvec_np, 'successmat_x': successmat_x_np}

            savemat(file_out, mdictmat)

        # Set output
        out = [xi for xi in [x, u, r, R, tvec, successmat_x]]

        return out

if __name__ == "__main__":

    _eval_policy(sys.argv[1::])
