from cassiegazebo_ctypes import *
from distutils.util import strtobool
import ctypes
import numpy as np
import time
import argparse
import tensorflow as tf
import json
import os
import pickle
from softlearning.policies.utils import get_policy_from_variant



class CassieUdp:
    motor_names = ['hipRollDrive', 'hipYawDrive', 'hipPitchDrive', 'kneeDrive', 'footDrive']
    joint_names = ['shinJoint', 'tarsusJoint', 'footJoint']

    def __init__(self, remote_addr='127.0.0.1', remote_port='25000',
                 local_addr='0.0.0.0', local_port='25001'):
        self.sock = udp_init_client(str.encode(remote_addr),
                                    str.encode(remote_port),
                                    str.encode(local_addr),
                                    str.encode(local_port))
        self.packet_header_info = packet_header_info_t()
        self.recvlen = 2 + 697
        self.sendlen = 2 + 58
        self.recvlen_pd = 2 + 493
        self.sendlen_pd = 2 + 476
        self.recvbuf = (ctypes.c_ubyte * max(self.recvlen, self.recvlen_pd))()
        self.sendbuf = (ctypes.c_ubyte * max(self.sendlen, self.sendlen_pd))()
        self.inbuf = ctypes.cast(ctypes.byref(self.recvbuf, 2),
                                 ctypes.POINTER(ctypes.c_ubyte))
        self.outbuf = ctypes.cast(ctypes.byref(self.sendbuf, 2),
                                  ctypes.POINTER(ctypes.c_ubyte))

    def send(self, u):
        pack_cassie_user_in_t(u, self.outbuf)
        send_packet(self.sock, self.sendbuf, self.sendlen, None, 0)

    def send_pd(self, u):
        pack_pd_in_t(u, self.outbuf)
        send_packet(self.sock, self.sendbuf, self.sendlen_pd, None, 0)

    def recv_wait(self):
        nbytes = -1
        while nbytes != self.recvlen:
            nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen,
                                       None, None)
        process_packet_header(self.packet_header_info,
                              self.recvbuf, self.sendbuf)
        cassie_out = cassie_out_t()
        unpack_cassie_out_t(self.inbuf, cassie_out)
        return cassie_out

    def recv_wait_pd(self):
        nbytes = -1
        while nbytes != self.recvlen_pd:
            nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen_pd,
                                       None, None)
        process_packet_header(self.packet_header_info,
                              self.recvbuf, self.sendbuf)
        state_out = state_out_t()
        unpack_state_out_t(self.inbuf, state_out)
        return state_out

    def recv_newest(self):
        nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen,
                                   None, None)
        if nbytes != self.recvlen:
            return None
        process_packet_header(self.packet_header_info,
                              self.recvbuf, self.sendbuf)
        cassie_out = cassie_out_t()
        unpack_cassie_out_t(self.inbuf, cassie_out)
        return cassie_out

    def recv_newest_pd(self):
        nbytes = get_newest_packet(self.sock, self.recvbuf, self.recvlen_pd,
                                   None, None)
        if nbytes != self.recvlen_pd:
            return None
        process_packet_header(self.packet_header_info,
                              self.recvbuf, self.sendbuf)
        state_out = state_out_t()
        unpack_state_out_t(self.inbuf, state_out)
        return state_out

    def delay(self):
        return ord(self.packet_header_info.delay)

    def seq_num_in_diff(self):
        return ord(self.packet_header_info.seq_num_in_diff)

    def __del__(self):
        udp_close(self.sock)

    @property
    def motor_pos(self):
        cassie_out = self.recv_wait()
        qpos_left = np.array([getattr(cassie_out.leftLeg, motor).position for motor in self.motor_names])
        qpos_right = np.array([getattr(cassie_out.rightLeg, motor).position for motor in self.motor_names])
        return np.concatenate([qpos_left, qpos_right])

    @property
    def motor_vel(self):
        cassie_out = self.recv_wait()
        qvel_left = np.array([getattr(cassie_out.leftLeg, motor).velocity for motor in self.motor_names])
        qvel_right = np.array([getattr(cassie_out.rightLeg, motor).velocity for motor in self.motor_names])
        return np.concatenate([qvel_left, qvel_right])

    @property
    def joint_pos(self):
        cassie_out = self.recv_wait()
        qpos_left = np.array([getattr(cassie_out.leftLeg, joint).position for joint in self.joint_names])
        qpos_right = np.array([getattr(cassie_out.rightLeg, joint).position for joint in self.joint_names])
        return np.concatenate([qpos_left, qpos_right])

    @property
    def joint_vel(self):
        cassie_out = self.recv_wait()
        qvel_left = np.array([getattr(cassie_out.leftLeg, joint).velocity for joint in self.joint_names])
        qvel_right = np.array([getattr(cassie_out.rightLeg, joint).velocity for joint in self.joint_names])
        return np.concatenate([qvel_left, qvel_right])

    @property
    def pelvis_measurements(self):
        cassie_out = self.recv_wait()
        nav = cassie_out.pelvis.vectorNav

        orien = np.array(nav.orientation)
        angvel = np.array(nav.angularVelocity)
        linacc = np.array(nav.linearAcceleration)
        mag = np.array(nav.magneticField)

        return orien, angvel, linacc, mag

    def get_obs(self):
        motor_pos, motor_vel = self.motor_pos, self.motor_vel
        joint_pos, joint_vel = self.joint_pos, self.joint_vel
        # orien, angvel, linacc, mag = self.pelvis_measurements
        orien, angvel, linvel, pos = self.pelvis_measurements  # This is a hack in the cpp code. Not sure how it works
        obs = np.concatenate([pos[1:], orien, motor_pos, joint_pos, linvel, angvel, motor_vel, joint_vel])
        # obs = np.concatenate([orien, angvel, linacc, motor_pos, joint_pos, motor_vel, joint_vel]) # TODO: probably need to integrate to get linvel of the torso
        return obs

    def get_pos(self):
        motor_pos, motor_vel = self.motor_pos, self.motor_vel
        joint_pos, joint_vel = self.joint_pos, self.joint_vel
        # orien, angvel, linacc, mag = self.pelvis_measurements
        orien, angvel, linvel, pos = self.pelvis_measurements  # This is a hack in the cpp code. Not sure how it works
        obs = np.concatenate([pos, orien, motor_pos, joint_pos])
        # obs = np.concatenate([orien, angvel, linacc, motor_pos, joint_pos, motor_vel, joint_vel]) # TODO: probably need to integrate to get linvel of the torso
        return obs

    def apply_torques(self, u):
        u *= np.array([4.5 * 25, 4.5 * 25, 12.2 * 16, 12.2 * 16, 0.9 * 50,
                       4.5 * 25, 4.5 * 25, 12.2 * 16, 12.2 * 16, 0.9 * 50])
        # u = u * np.array([25, 25, 16, 16, 50, 25, 25, 16, 16, 50]),
        u_ptr = np.array(u, dtype=np.float64).ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        user_in = cassie_user_in_t()
        ctypes.memmove(user_in.torque, u_ptr, 10 * 1 * ctypes.sizeof(ctypes.c_double))
        self.send(user_in)


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

    cassie = CassieUdp(remote_addr='127.0.0.1')
    received_data = False

    # Listen/respond loop
    env = pickleable['env']
    policy = (
        get_policy_from_variant(variant, env, Qs=[None]))
    policy.set_weights(pickleable['policy_weights'])
    first = True
    posses = []

    with policy.set_deterministic(args.deterministic):
        while True:
            if not received_data:
                # Send packets until the simulator responds
                print('Connecting...')
                y = None
                while y is None:
                    cassie.send(cassie_user_in_t())
                    time.sleep(0.005)
                    y = cassie.recv_newest()
                received_data = True
                print('Connected!\n')
                obs = cassie.get_obs()
                t0 = time.time()
            else:
                # Wait for new data
                y = cassie.recv_wait()
                time.sleep(0.009)


                obs = cassie.get_obs()
                pos = cassie.get_pos()
                posses.append(pos)
                if time.time() - t0 > 20:
                    pickle.dump(posses, open("/home/ignasi/posses.pkl", "wb"))
                    t0 = time.time()

            if False and first and obs[2] > .975:
                print("None")
                pass
            else:
                first = False
                action = policy.actions_np(obs[None])[0]
                print("action")

                # Run controller
                cassie.apply_torques(action)




if __name__ == '__main__':
    args = parse_args()
    simulate_policy(args)

