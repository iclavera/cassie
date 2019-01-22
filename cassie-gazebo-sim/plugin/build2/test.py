from cassiegazebo_ctypes import *
import ctypes
import numpy as np
import time


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
        orien, angvel, linvel, pos = self.pelvis_measurements
        obs = np.concatenate([orien, angvel, linacc, motor_pos, joint_pos, motor_vel, joint_vel]) # TODO: probably need to integrate to get linvel of the torso
        print(obs.shape)
        return obs

    def apply_torques(self, u):
        u_ptr = np.array(u, dtype=np.float64).ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        user_in = cassie_user_in_t()
        ctypes.memmove(user_in.torque, u_ptr, 10 * 1 * ctypes.sizeof(ctypes.c_double))
        self.send(user_in)


if __name__ == "__main__":
    # Set up UDP connection
    cassie = CassieUdp(remote_addr='127.0.0.1')
    received_data = False

    # Listen/respond loop
    while True:
        if not received_data:
            # Send packets until the simulator responds
            print('Connecting...')
            y = None
            while y is None:
                cassie.send(cassie_user_in_t())
                time.sleep(0.01)
                y = cassie.recv_newest()
            received_data = True
            print('Connected!\n')
        else:
            # Wait for new data
            y = cassie.recv_wait()
            time.sleep(0.01)

        # Print connection stats
        # print('\033[F\033[Jdelay: {}, diff: {}'.format(cassie.delay(),
        #                                                cassie.seq_num_in_diff()))


        u = np.random.uniform(-1000, 1000, size=10)
        cassie.apply_torques(u)

        orien, angvel, linvel, pos = cassie.pelvis_measurements
        print("orien  :", orien)
        print("angvel  :", angvel)
        print("linvel  :", linvel)
        print("pos  :", pos)


        #
        # cassie.send(user_in)