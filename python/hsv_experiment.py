import torch
import argparse
import numpy as np

from value_iteration.hsv import hsvintaugquad, hsvquad
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

    # Do integral augmentation (=1) or not (=0)
    dointaug = True

    model_path = None
    if rfvi1cfvi0 and not init1load0_single:
        model_path = 'data/rFVI_hsvintaug_quad.torch' if dointaug else 'data/rFVI_hsv_quad.torch'

    if not rfvi1cfvi0 and not init1load0_single:
        model_path = 'data/cFVI_hsvintaug_quad.torch' if dointaug else 'data/cFVI_hsv_quad.torch'


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
        'system_class': hsvintaugquad if dointaug else hsvquad,
        # Recall: x = [x_p^T z^T]^T, where z = [z_V, z_\gamma] \in R^2 is the integrator bank
        'state_cost': '2., 5., 0.05, 1e-6, 2., 2.5' if dointaug else '2., 5., 0.05, 1e-6',
        'action_cost': '2.5, 1.',

        'eps': 6.5e-1,  # eps = 1 => \gamma = 1
        'dt': 1. / 25.,
        'T': 20.,
        'dt_eval': 1. / 25.,
        'T_eval': 50.,

        # Initial policy
        'do_init_pol': True,
        'filename_init_pol': 'data/hsvintaug_quad/00_init_pol.torch',

        # Network:
        'n_network': 4,
        'activation': 'Tanh',
        'n_width': 96,
        'n_depth': 3,
        'n_output': 1,
        'g_hidden': 1.41,
        'g_output': 1.,
        'b_output': -0.1,

        # softplus \beta parameter in differential network.
        # See 'differential_quadratic_network.py', line 23
        'L_softplus_beta': 7.5,

        # Clip state, control
        'doclip_eval': True,

        # Samples
        'n_iter': 50,
        'eval_minibatch': 256 * 200,
        'test_minibatch': 256 * 20,
        'n_minibatch': 256,
        'n_batches': 200,

        # Network Optimization
        'max_epoch': 20,
        'lr_SGD': 1.0e-4,
        'weight_decay': 1.e-6,
        'exp': 1.,

        # Lambda Traces
        'trace_weight_n': 1.e-3,
        'trace_lambda': 0.95,

        # Exploration:
        'x_noise': 1.e-6,
        'u_noise': 1.e-6,

        # Save tabular data (=1) or not (=0)
        'savetbldata': True,

        # Single (=1) or sweep (=0)
        'single1sweep0': single1sweep0,

        # For sweep, initialize new sweep (=1) or load previous data (=0)
        'init1load0_sweep': init1load0_sweep,

    }

    # Select the admissible set of the adversary:
    hyper['xi_x_alpha'] = 0.001 if hyper["robust"] else 1.e-6
    hyper['xi_u_alpha'] = 0.05 if hyper["robust"] else 1.e-6
    hyper['xi_o_alpha'] = 0.01 if hyper["robust"] else 1.e-6
    hyper['xi_m_alpha'] = 0.125 if hyper["robust"] else 1.e-6

    print("Hyperparameters:")
    for key in hyper.keys():
        print(f"{key:30}: {hyper[key]}")

    # Run experiments
    run_experiment_multi(hyper)
