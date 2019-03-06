from softlearning.environments.cassie.assets.cassiemujoco import CassieSim, CassieVis, cassie_user_in_t
import numpy as np
from gym import utils
from gym import spaces
import gym


# CASSIE_TORQUE_LIMITS = np.array([4.5*25, 4.5*25, 12.2*16, 12.2*16, 0.9*50]) # ctrl_limit * gear_ratio
# CASSIE_MOTOR_VEL_LIMIT = np.array([2900, 2900, 1300, 1300, 5500]) / 60 / (2*np.pi) # max_rpm / 60 / 2*pi
# P_GAIN_RANGE = [10, 10000]
# D_GAIN_RANGE = [1, 100]
# MODEL_TIMESTEP = 0.001
#
# DEFAULT_P_GAIN = 200
# DEFAULT_D_GAIN = 20
#
# NUM_QPOS = 34
# NUM_QVEL = 32
#
# CTRL_COST_COEF = 0.001
# STABILISTY_COST_COEF = 0.01


class CassieEnv(gym.Env, utils.EzPickle):
    _JOINT_NAMES = ['hipRollDrive', 'hipYawDrive', 'hipPitchDrive', 'kneeDrive', 'shinJoint', 'tarsusJoint', 'footDrive']

    def __init__(self, render=False, fix_pelvis=False, frame_skip=20,
                 stability_cost_coef=1e-2, ctrl_cost_coef=1e-3, alive_bonus=0.2, impact_cost_coef=1e-5,
                 rotation_cost_coef=1e-2, policytask='running', ctrl_type='T', apply_forces=False):

        print('fr_skip:', frame_skip, 'task', policytask)

        assert ctrl_type in ['T', 'P', 'V', 'TP', 'TV', 'PV', 'TPV']
        assert ctrl_type == 'T'
        # T: Torque ctrl        # TP: Torque + Position ctrl    # None or all: Torque + Position + Velocity
        # P: Positon ctrl       # TV: Torque + Velocity ctrl
        # V: Velocity ctrl      # PV: Position + Velocity ctr

        self.sim = CassieSim()
        if render:
            self.vis = CassieVis()
        else:
            self.vis = None

        self.fix_pelvis = fix_pelvis
        self.model_timestep = 0.01
        self.act_dim = 10
        self.frame_skip = frame_skip
        self.task = policytask
        self.ctrl_type = ctrl_type
        self.apply_forces = apply_forces

        # Reward function coeffs
        self.stability_cost_coef = stability_cost_coef
        self.ctrl_cost_coef = ctrl_cost_coef
        self.impact_cost_coef = impact_cost_coef
        self.alive_bonus = alive_bonus
        self.rotation_cost_coef = rotation_cost_coef

        self._time_step = 0
        self._torque_limits = np.array([4.5, 4.5, 12.2, 12.2, 0.9] * 2)

        if fix_pelvis:
            self.sim.hold()

        utils.EzPickle.__init__(self, locals())

    def _get_obs(self):
        internal_state = self.sim.recv_wait()
        foot_state = self.sim.recv_wait_state()
        _, pelvis_ang_vel, _, pelvis_magnetic_field = self.sim.pelvis_measurements

        pelvis_pos_rel_to_r_foot = -np.array(foot_state.rightFoot.position)
        pelvis_vel_rel_to_r_foot = -np.array(foot_state.rightFoot.footTranslationalVelocity)

        qpos_left = [np.array(getattr(internal_state.leftLeg, joint).position) for joint in self._JOINT_NAMES]
        qpos_right = [np.array(getattr(internal_state.rightLeg, joint).position) for joint in self._JOINT_NAMES]

        qvel_left = [np.array(getattr(internal_state.leftLeg, joint).velocity) for joint in self._JOINT_NAMES]
        qvel_right = [np.array(getattr(internal_state.rightLeg, joint).velocity) for joint in self._JOINT_NAMES]

        obs = np.concatenate([pelvis_pos_rel_to_r_foot,
                              pelvis_magnetic_field,
                              qpos_left,
                              qpos_right,
                              pelvis_vel_rel_to_r_foot,
                              pelvis_ang_vel,
                              qvel_left,
                              qvel_right])

        return obs

    def step(self, action):
        assert action.ndim == 1 and action.shape == (self.act_dim,)
        if self.apply_forces and self._time_step % 10 == 0:
            self.apply_random_force()
        _ = self.do_simulation(action, self.frame_skip)
        obs = self._get_obs()

        reward, forward_vel = self.reward()

        done = self.done()
        info = {'forward_vel': forward_vel}

        return obs, reward, done, info

    def reset(self):
        self.sim = CassieSim()
        if self.fix_pelvis: self.sim.hold()
        internal_state = self.do_simulation(np.zeros(self.act_dim), 1)
        self._time_step = 0
        return self._get_obs()

    def do_simulation(self, u, n_frames):
        assert n_frames >= 1
        u = self._action_to_user_in_t(u)
        for _ in range(n_frames):
            internal_state = self.sim.step(u)
        return internal_state

    def done(self):
        state = self.sim.get_state()
        pelvis_pos = np.array(state.qpos())
        return pelvis_pos[2] < 0.65

    def reward(self):
        internal_state = self.sim.recv_wait_state()
        state = self.sim.get_state()
        # reward fct
        qvel = np.array(state.qvel())
        pelvis_rot_vel = qvel[3:6]
        pelvis_transl_vel = qvel[:3]

        foot_forces = self.get_foot_forces(internal_state)
        motor_torques = _to_np(internal_state.motor.torque)
        forward_vel = pelvis_transl_vel[0]

        ctrl_cost = self.ctrl_cost_coef * 0.5 * np.mean(np.square(motor_torques/self._torque_limits))
        stability_cost = self.stability_cost_coef * 0.5 * np.mean(np.square(pelvis_transl_vel[1:]))  #  quadratic velocity of pelvis in y and z direction ->
        rotation_cost = self.rotation_cost_coef * 0.5 * np.mean(np.square(pelvis_rot_vel))#  enforces to hold the pelvis in same position while walking
        impact_cost = self.impact_cost_coef * 0.5 * np.sum(np.square(np.clip(foot_forces, -1, 1)))

        if self.task == 'balancing':
            vel_cost = self.stability_cost_coef * forward_vel ** 2
            reward = - vel_cost - ctrl_cost - stability_cost - impact_cost + self.alive_bonus
        elif self.task == 'fixed-vel':
            vel_reward = np.exp(- (2.3 - forward_vel) ** 2)
            reward = vel_reward - ctrl_cost - stability_cost - rotation_cost - impact_cost + self.alive_bonus
        else:
            reward = forward_vel - ctrl_cost - stability_cost - rotation_cost - impact_cost + self.alive_bonus
        return reward, forward_vel

    def render(self):
        if self.vis is None:
            print('Setting up cassie visualizer')
            self.setup_cassie_vis()
        self.vis.draw(self.sim)

    def get_foot_forces(self, internal_state):
        left_toe = _to_np(internal_state.leftFoot.toeForce)
        left_heel = _to_np(internal_state.leftFoot.heelForce)
        right_toe = _to_np(internal_state.rightFoot.toeForce)
        right_heel = _to_np(internal_state.rightFoot.heelForce)
        return np.concatenate([left_toe, left_heel, right_toe, right_heel])

    def apply_random_force(self):
        force = np.zeros((6,))
        y_force = np.random.choice([0, 10, 25, 50]) * np.random.choice([-1, 1])
        force[1] = y_force
        self.sim.apply_force(force)

    def _action_to_user_in_t(self, action):
        u = cassie_user_in_t()
        for i in range(self.act_dim):
            u.torque[i] = action[i]
        return u

    @property
    def torque_limits(self):
        return np.concatenate([self.parameters['cassie_torque_limits']] * 2)

    @property
    def dt(self):
        return self.model_timestep

    @property
    def action_space(self):
        return spaces.Box(low=-self._torque_limits, high=self._torque_limits, dtype=np.float32)

    @property
    def observation_space(self):
        obs_limit = np.inf * np.ones(self.obs_dim)
        return spaces.Box(-obs_limit, obs_limit, dtype=np.float32)

    def setup_cassie_vis(self):
        self.vis = CassieVis()

    def log_diagnostics(self, paths):
        pass
        # forward_vel = [np.mean(path['env_infos']['forward_vel']) for path in paths]
        # ctrl_cost = [np.mean(path['env_infos']['ctrl_cost']) for path in paths]
        # stability_cost = [np.mean(path['env_infos']['stability_cost']) for path in paths]
        # path_length = [path["observations"].shape[0] for path in paths]
        #
        # logger.record_tabular('AvgForwardVel', np.mean(forward_vel))
        # logger.record_tabular('StdForwardVel', np.std(forward_vel))
        # logger.record_tabular('AvgCtrlCost', np.mean(ctrl_cost))
        # logger.record_tabular('AvgStabilityCost', np.mean(stability_cost))
        # logger.record_tabular('AvgPathLength', np.mean(path_length))


def _to_np(o, dtype=np.float32):
    return np.array([o[i] for i in range(len(o))], dtype=dtype)


if __name__ == '__main__':
    render = True
    env = CassieEnv(render=render, fix_pelvis=False, frame_skip=200)
    import time

    for i in range(5):
        obs = env.reset()
        for j in range(50000):
            cum_forward_vel = 0
            act = env.action_space.sample()
            env.apply_random_force()
            obs, reward, done, info = env.step(act)
            if render:
                env.render()
            time.sleep(1)
            # if done:
            #     break
