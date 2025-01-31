import time
import torch
import numpy as np

from value_iteration.run_experiment import run_experiment


# for saving python arrays to .mat files
from scipy.io import savemat

def run_experiment_multi(hyper):
    cuda = torch.cuda.is_available()
    alg_name = "rFVI" if hyper['robust'] else "cFVI"

    n_iter = hyper['n_iter']
    single1sweep0 = hyper['single1sweep0']
    system_class = hyper['system_class']
    init1load0_sweep = hyper['init1load0_sweep']

    Q = np.array([float(x) for x in hyper['state_cost'].split(',')])
    R = np.array([float(x) for x in hyper['action_cost'].split(',')])
    system = hyper['system_class'](Q, R, cuda=cuda, **hyper)

    # INITIALIZE SEEDS
    if not single1sweep0:
        seeds = torch.arange(0, 20, 1).int()
        numseeds = seeds.shape[0]
    else:
        seeds = torch.tensor([42]).int()
        numseeds = 1
        numseeds_prev = 0

    meanreturn = torch.zeros(numseeds, n_iter + 1)
    stdreturn = torch.zeros(numseeds, n_iter + 1)
    runtime = torch.zeros(numseeds, n_iter + 1)


    # SWEEP ONLY: Load previous training data
    if not single1sweep0:
        # File name
        filename = f"data/{alg_name}_{system_class.name}_sweep.torch"
        filename_mat = f"data/{alg_name}_{system_class.name}_sweep.mat"
        if not init1load0_sweep:
            # Load data
            data = torch.load(filename, map_location=torch.device('cpu'))
            # Unpack data
            state_dict_data = data['state_dict_data']
            meanreturn_prev = data['meanreturn']
            stdreturn_prev = data['stdreturn']
            runtime_prev = data['runtime']
            # Get number of seeds trained for
            numseeds_prev = len(state_dict_data)
            n_iter_prev = meanreturn_prev.shape[1] - 1
            # For each seed trained for, determine if additional training is necessary and what iteration to begin at
            zeros_prev = torch.eq(meanreturn_prev, torch.zeros_like(meanreturn_prev))
            train_all = n_iter_prev < n_iter
            train_vec = torch.zeros(numseeds, dtype=torch.bool)
            train_vec[numseeds_prev:] = True
            train_vec = torch.logical_or(train_vec, torch.tensor(train_all))
            n_iter_prev_vec = torch.zeros(numseeds, dtype=torch.int)
            for seedcnt in range(numseeds_prev):
                for itercnt in range(n_iter_prev + 1):
                    if zeros_prev[seedcnt, itercnt]:
                        train_vec[seedcnt] = True
                        n_iter_prev_vec[seedcnt] = itercnt
                        break
                if itercnt == n_iter_prev + 1:
                    n_iter_prev_vec[seedcnt] = n_iter_prev + 1
            # Fill in previous data
            meanreturn[0:numseeds_prev, 0:n_iter_prev + 1] = meanreturn_prev[0:numseeds_prev, :]
            stdreturn[0:numseeds_prev, 0:n_iter_prev + 1] = stdreturn_prev[0:numseeds_prev, :]
            runtime[0:numseeds_prev, 0:n_iter_prev + 1] = runtime_prev[0:numseeds_prev, :]
        else:
            # Create a new list
            state_dict_data = list()
            numseeds_prev = 0
            train_vec = torch.ones(numseeds, dtype=torch.bool)
            n_iter_prev_vec = torch.zeros(numseeds, dtype=torch.int)
    else:
        # Create a new list
        state_dict_data = list()
        numseeds_prev = 0
        train_vec = torch.ones(numseeds, dtype=torch.bool)
        n_iter_prev_vec = torch.zeros(numseeds, dtype=torch.int)
        if hyper['checkpoint'] is not None:

            # Load data
            data = torch.load(hyper['checkpoint'], map_location=torch.device('cpu'))
            hyper_prev = data['hyper']

            # Sweep settings
            numseeds_prev = 1
            n_iter_prev = data['epoch'] + 1
            n_iter_prev_vec = torch.tensor([n_iter_prev], dtype=torch.int)
            train_vec = torch.ones(numseeds_prev, dtype=torch.bool)

            # Update old hyperparams
            hyper_prev = data['hyper']
            hyper_prev['n_iter'] = n_iter
            hyper_prev['single1sweep0'] = hyper['single1sweep0']
            # hyper_prev['eval_J'] = True      # DEBUGGING

            # Set hyperparams as old
            hyper = hyper_prev

            # Add old (single) sweep data
            runtime[0, 0:-1] = data['runtime_vec'][0:-1]
            state_dict_data.append(data['state_dict'])



    # Run Experiment:
    for seedcnt in range(numseeds):
        if train_vec[seedcnt]:
            # Set current seed, starting iteration, RNG
            seedi = seeds[seedcnt]
            print("\n\n################################################")
            print("################################################")
            print("################################################\n\n")
            print(f"TRAINING FOR SEED NUMBER: {seedcnt+1}   OF:   {numseeds}")
            print(f"\nCURRENT SEED: {seedi}")
            print("\n\n################################################")
            print("################################################")
            print("################################################\n\n")
            iter_starti = n_iter_prev_vec[seedcnt].item()
            hyperi = hyper
            hyperi['seed'] = seedi
            hyperi['numseeds'] = numseeds
            hyperi['iter_start'] = iter_starti
            if iter_starti > 0:
                hyperi['state_dict'] = state_dict_data[seedcnt]
                hyperi['meanreturn_vec'] = meanreturn[seedcnt, :]
                hyperi['stdreturn_vec'] = stdreturn[seedcnt, :]
                hyperi['runtime_vec'] = runtime[seedcnt, :]
            np.random.seed(seedi)
            torch.manual_seed(seedi)
            torch.cuda.manual_seed_all(seedi)
            # Train
            outdata = run_experiment(hyperi)
            if not single1sweep0:
                # Unpack data
                if seedcnt < numseeds_prev:
                    state_dict_data[seedcnt] = outdata['state_dict']
                else:
                    state_dict_data.append(outdata['state_dict'])
                meanreturn[seedcnt, :] = outdata['meanreturn']
                stdreturn[seedcnt, :] = outdata['stdreturn']
                runtime[seedcnt, :] = outdata['runtime']
                # Save data to file
                data = {
                    'state_dict_data': state_dict_data,
                    'meanreturn': meanreturn,
                    'stdreturn': stdreturn,
                    'runtime': runtime,
                    'hyper': hyper
                }
                data_mat = {
                    'meanreturn': meanreturn.cpu().numpy(),
                    'stdreturn': stdreturn.cpu().numpy(),
                    'runtime': runtime.cpu().numpy(),
                }
                torch.save(data, filename)
                savemat(filename_mat, data_mat)

