import sys
import pathlib
currpath = str(pathlib.Path(__file__).parent.resolve())
#sys.path.append(currpath + f"/../")

import time
import torch
import numpy as np

from value_iteration.run_experiment import  run_experiment

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


if __name__ == "__main__":

    # Args
    seed = 0
    iter = 100
    system_name = 'hsvintaug_quad'
    alg_name = 'cFVI'
    #alg_name = 'rFVI'
    n_eval = 1000

    model_file = f"../data/{system_name}/{alg_name}_seed_{seed:03d}_step_{iter:03d}.torch"

    # Load data
    data = torch.load(model_file, map_location=torch.device('cpu'))

    # Extract data
    hyper = data['hyper']
    state_dict = data['state_dict']

    hyper['doeval'] = True
    hyper['state_dict'] = state_dict
    hyper['iter_start'] = iter - 1
    hyper['meanreturn_vec'] = torch.zeros(1, iter)
    hyper['stdreturn_vec'] = torch.zeros(1, iter)
    hyper['runtime_vec'] = torch.zeros(1, iter)

    run_experiment(hyper)


