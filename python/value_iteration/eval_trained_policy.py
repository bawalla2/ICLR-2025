import sys
import pathlib
currpath = str(pathlib.Path(__file__).parent.resolve())
currpath_base = currpath + f"/../"
sys.path.append(currpath_base)

import torch
import argparse
import numpy as np
import time
from scipy.io import savemat


from value_iteration.sim_policy import sim_policy, init_sys, init_value_funct

def eval_trained_policy(arg_list):
    parser = argparse.ArgumentParser()
    parser.add_argument("-system_name", dest='system_name', type=str)
    parser.add_argument("-alg_name", dest='alg_name', type=str)
    parser.add_argument("-seed_start", dest='seed_start', type=str)
    parser.add_argument("-seed_end", dest='seed_end', type=str)
    parser.add_argument("-iter_eval", dest='iter_eval', type=str)
    parser.add_argument("-file_out", dest='file_out', type=str)
    parser.add_argument("-T", dest='T', type=float)
    parser.add_argument("-fs", dest='fs', type=float)
    parser.add_argument("-fs_return", dest='fs_return', type=float)
    parser.add_argument("-n_sim", dest='n_sim', type=int)
    parser.add_argument("-theta_mode", dest='theta_mode', type=str)
    parser.add_argument("-dtheta", dest='dtheta', type=str)
    parser.add_argument("-do_thresh_x", dest='do_thresh_x', action='store_true')
    parser.add_argument("-rt_Avec", dest='rt_Avec', type=str)
    parser.add_argument("-pfavec", dest='pfavec', type=str)
    parser.add_argument("-pfICtype", dest='pfICtype', type=str)
    parser.add_argument("-verbose", dest='verbose', action='store_true')
    args = parser.parse_args(arg_list)

    cuda = torch.cuda.is_available()

    # Unpack args
    system_name = args.system_name
    alg_name = args.alg_name
    file_out = args.file_out
    T = args.T
    fs = args.fs
    fs_return = args.fs_return
    theta_mode = args.theta_mode
    do_thresh_x = args.do_thresh_x
    seed_start = int(args.seed_start) if args.seed_start is not None else int(0)
    seed_end = int(args.seed_end)
    iter_eval = int(args.iter_eval)
    n_sim = args.n_sim
    verbose = args.verbose

    # Path to first seed evaluated
    model_file = currpath_base + f"data/{system_name}/{alg_name}_seed_{seed_start:03d}_step_{iter_eval:03d}.torch"
    # Load data
    data = torch.load(model_file, map_location=torch.device('cpu'))

    # Get hyperparameters
    hyper = data['hyper']
    # Build the dynamical system:
    sys = init_sys(hyper)
    # State initialization
    x_init = sys.x_init.float().view(1, sys.n_state, 1)


    # Modeling error parameters \theta
    if theta_mode == 'const':
        dtheta = torch.tensor([float(xi) for xi in args.dtheta.split(',')])

    # Check for pre-filter
    pf1nopf0 = args.pfavec is not None

    # Configuration for sampling trajectories from the system
    run_config = {'T': T, 'fs': fs, 'fs_return': fs_return, 'theta_mode': theta_mode, 'delete_xu_data': False,
                  'rt_Avec': args.rt_Avec, 'pfavec': args.pfavec, 'pfICtype': args.pfICtype, 'do_thresh_x': do_thresh_x}
    if theta_mode == 'const':
        run_config['dtheta'] = dtheta

    # Number of seeds evaluated
    n_seed = seed_end - seed_start + 1

    # Data storage
    Rmat = torch.zeros(n_seed, n_sim)
    if do_thresh_x:
        threshmat_x = sys.threshmat_x
        numthresh_x = threshmat_x.shape[0]
        successmat_x = torch.zeros(n_seed, n_sim, numthresh_x)

    # Evaluate policies
    for seedcnt in range(seed_start, seed_end+1):

        if verbose:
            print("\n\n################################################\n\n")
            print(f"EVALUATING FOR SEED NUMBER: {seedcnt:02d}")
            print("\n\n################################################\n\n")

        # Init the RNG seed
        torch.manual_seed(seedcnt + 100)

        t0 = time.perf_counter()
        # Path to current state dict data
        model_file = currpath_base + f"data/{system_name}/{alg_name}_seed_{seedcnt:03d}_step_{iter_eval:03d}.torch"
        # Load data
        data = torch.load(model_file, map_location=torch.device('cpu'))

        # Extract state dict data
        state_dict = data['state_dict']
        # Construct Value Function:
        val_fun = init_value_funct(hyper, sys, state_dict)
        device = val_fun.device

        # ICs
        dist_x = torch.distributions.uniform.Uniform(-x_init, x_init)
        x0 = dist_x.sample((n_sim,)).view(-1, sys.n_state, 1).to(device)

        # Evaluate the policy
        out = sim_policy(val_fun, sys, x0, run_config)

        # Extract data
        R = out[3].squeeze()
        succxmat = out[5]

        # Store
        Rmat[seedcnt, :] = R
        if do_thresh_x:
            successmat_x[seedcnt, :, :] = succxmat.view(-1, numthresh_x)

    # Cast to numpy for saving
    Rmat_np = Rmat.cpu().numpy()
    successmat_x_np = successmat_x.cpu().numpy()

    mdictmat = {'R': Rmat_np, 'successmat_x': successmat_x_np}

    savemat(file_out, mdictmat)


if __name__ == "__main__":

    eval_trained_policy(sys.argv[1::])
