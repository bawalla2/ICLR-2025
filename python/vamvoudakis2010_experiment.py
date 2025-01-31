import torch
import argparse
import numpy as np

from value_iteration.vamvoudakis2010 import vamvoudakis2010quad
from value_iteration.run_experiment_multi import run_experiment_multi

try:
    import matplotlib as mpl
    mpl.use("Qt5Agg")

except ImportError as e:
    pass

from value_iteration.value_function import QuadraticNetwork, TrigonometricQuadraticNetwork

if __name__ == "__main__":

    # Initialize NumPy:
    np.set_printoptions(
        suppress=True, precision=2, linewidth=500,
        formatter={'float_kind': lambda x: "{0:+08.2f}".format(x)})

    # Initialize PyTorch:
    torch.set_num_threads(1)
    torch.backends.cudnn.benchmark = True

    # rFVI (=1) or cFVI (=0)
    #rfvi1cfvi0 = True
    rfvi1cfvi0 = False

    # Do a single seed (=0) or a sweep of seeds (=1)
    #single1sweep0 = True
    single1sweep0 = False

    # For single, initialize new (=1) or load previous data (=0)
    init1load0_single = True
    #init1load0_single = False

    # For sweep, initialize new sweep (=1) or load previous data (=0)
    init1load0_sweep = True
    #init1load0_sweep = False

    model_path = None
    if rfvi1cfvi0 and not init1load0_single:
        model_path = 'data/rFVI_vamvoudakis2010_quad.torch'

    if not rfvi1cfvi0 and not init1load0_single:
        model_path = 'data/cFVI_vamvoudakis2010_quad.torch'


    # Define Hyper-parameters:
    hyper = {
        # Learning Mode:
        'mode': 'DP',
        'robust': rfvi1cfvi0,

        # Value Function:
        'val_class': QuadraticNetwork,
        'checkpoint': model_path,
        'plot': True,
        'doeval': False,

        # System Specification:
        'system_class': vamvoudakis2010quad,

        # Post Vscl change
        'state_cost': '1., 1.',
        'action_cost': '1.',

        'eps': 6.5e-1,  # eps = 1 => \gamma = 1
        'dt': 1. / 125.,
        'T': 5.,
        'dt_eval': 1. / 50.,
        'T_eval': 5.,

        # Initial policy
        'do_init_pol': False,

        # Network:
        'n_network': 4,
        'activation': 'Tanh',
        'n_width': 96,
        'n_depth': 3,
        'n_output': 1,
        'g_hidden': 1.41,
        'g_output': 1.,
        'b_output': -0.1,

        # Samples
        'n_iter': 25,
        'eval_minibatch': 128 * 200,
        'test_minibatch': 128 * 20,
        'n_minibatch': 128,
        'n_batches': 200,

        # Network Optimization
        'max_epoch': 20,
        'lr_SGD': 1.0e-6,
        'weight_decay': 1.e-6,
        'exp': 1.,

        # Lambda Traces
        'trace_weight_n': 1.e-4,
        'trace_lambda': 0.85,

        # Exploration:
        'x_noise': 1.e-6,
        'u_noise': 1.e-6,

        # Single (=1) or sweep (=0)
        'single1sweep0': single1sweep0,

        # For sweep, initialize new sweep (=1) or load previous data (=0)
        'init1load0_sweep': init1load0_sweep,

    }

    # Select the admissible set of the adversary:
    hyper['xi_x_alpha'] = 0.025 if hyper["robust"] else 1.e-6
    hyper['xi_u_alpha'] = 0.100 if hyper["robust"] else 1.e-6
    hyper['xi_o_alpha'] = 0.025 if hyper["robust"] else 1.e-6
    hyper['xi_m_alpha'] = 0.150 if hyper["robust"] else 1.e-6

    print("Hyperparameters:")
    for key in hyper.keys():
        print(f"{key:30}: {hyper[key]}")

    # Run experiments
    run_experiment_multi(hyper)
