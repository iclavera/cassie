import argparse
from distutils.util import strtobool
import json
import os
import pickle

import tensorflow as tf
import numpy as np
from softlearning.policies.utils import get_policy_from_variant
from softlearning.samplers import rollouts


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('checkpoint_path',
                        type=str,
                        help='Path to the checkpoint.')
    parser.add_argument('--max-path-length', '-l', type=int, default=1000)
    parser.add_argument('--num-rollouts', '-n', type=int, default=10)
    parser.add_argument('--render-mode', '-r',
                        type=str,
                        default='human',
                        choices=('human', 'rgb_array', None),
                        help="Mode to render the rollouts in.")
    parser.add_argument('--deterministic', '-d',
                        type=strtobool,
                        nargs='?',
                        const=True,
                        default=True,
                        help="Evaluate policy deterministically.")

    args = parser.parse_args()

    return args


def simulate_policy(args):
    session = tf.keras.backend.get_session()
    checkpoint_path = args.checkpoint_path.rstrip('/')
    experiment_path = os.path.dirname(checkpoint_path)

    variant_path = os.path.join(experiment_path, 'params.json')
    with open(variant_path, 'r') as f:
        variant = json.load(f)

    with session.as_default():
        pickle_path = os.path.join(checkpoint_path, 'checkpoint.pkl')
        with open(pickle_path, 'rb') as f:
            pickleable = pickle.load(f)

    env = pickleable['env']
    policy = (
        get_policy_from_variant(variant, env, Qs=[None]))
    policy.set_weights(pickleable['policy_weights'])

    if True: #hard coded
        import numpy as np
        import scipy.io as sio    
        ws = policy.get_weights()
        w0, b0, w1, b1, w2, b2 = ws[0], ws[1], ws[2], ws[3], ws[4], ws[5]
        savematpath = '/home/parsa/projects/cassie/cassie_ignasi3/policy_weights.mat' #hard coded
        sio.savemat(savematpath, {'w0':w0, 'b0':b0, 'w1':w1, 'b1':b1, 'w2':w2, 'b2':b2})

    # env.unwrapped.vis.start_recording()

    with policy.set_deterministic(args.deterministic):
        paths = rollouts(env,
                         policy,
                         path_length=args.max_path_length,
                         n_paths=args.num_rollouts,
                         render_mode=args.render_mode)

        import matplotlib.pyplot as plt

        real = [path['observations'][:, 0] for path in paths][0]
        filtered = [path['observations'][:, 1] for path in paths][0]

        fig, axarr = plt.subplots(2, 1)
        axarr[0].plot(range(len(real)), real)
        axarr[1].plot(range(len(filtered)), filtered)

        # velocities_pelvis_filtered  = [path['observations'][:, :3] for path in paths]
        # velocities_pelvis  = [path['observations'][:, -3:] for path in paths]

        # fig, axarr = plt.subplots(3, 2)
        # for i in range(3):
        #     for vel_path in velocities_pelvis:
        #         axarr[i, 0].plot(range(len(vel_path)), vel_path[:,i])
        #     for vel_path in velocities_pelvis_filtered:
        #         axarr[i, 1].plot(range(len(vel_path)), np.cumsum(vel_path[:,i]) * 10) 


        # plt.show()

    if args.render_mode != 'human':
        from pprint import pprint; import pdb; pdb.set_trace()
        pass

    # env.unwrapped.vis.stop_recording('./test_vid.mp4', speedup=1, frame_skip=20, timestep=env.unwrapped.dt)

    return paths


if __name__ == '__main__':
    args = parse_args()
    simulate_policy(args)
