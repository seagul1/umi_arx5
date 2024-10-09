import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import scipy.interpolate as si
import scipy.spatial.transform as st

from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from diffusion_policy.common.precise_sleep import precise_wait
import sys

sys.path.append("/home/zyj/arx5-sdk/python")
import arx5_interface as arx5

from communication.zmq_client import Arx5Client, CTRL_DT
import time
import numpy as np
import numpy.typing as npt

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

class Arx5Interface:
    def __init__(self, model='X5', can_interface='can0', urdf_path='/home/zyj/arx5-sdk/models/arx5.urdf'):
        self.cartesian_controller = arx5.Arx5CartesianController(model, can_interface, urdf_path)

    def get_ee_pose(self):
        eef_state = self.cartesian_controller.get_eef_state()
        return eef_state.pose_6d()

    def get_eef_pose(self):
        eef_state = self.cartesian_controller.get_eef_state()
        return eef_state.gripper_pos

    def set_eef_state(self, eef_state):
        self.cartesian_controller.set_eef_state(eef_state)

    def get_joint_positions(self):
        joint_state = self.cartesian_controller.get_joint_state()
        return joint_state.pos()

    def get_joint_velocities(self):
        joint_state = self.cartesian_controller.get_joint_state()
        return joint_state.vel()

    def move_to_joint_positions(self, positions: np.ndarray, time_to_go: float):
        client = Arx5Client(zmq_ip="127.0.0.1", zmq_port=8765)
        client.reset_to_home()

        home_pose = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        duration = time_to_go

        def move_to_pose(
                start_pose: npt.NDArray[np.float64],
                stop_pose: npt.NDArray[np.float64],
                duration: float,
        ):
            step_num = int(duration / CTRL_DT)
            start_time = time.monotonic()
            for i in range(step_num):
                interp_pose = stop_pose * (i + 1) / step_num + start_pose * (
                        1 - (i + 1) / step_num
                )
                communication_start_time = time.monotonic()
                client.set_ee_pose(interp_pose, 0)
                print(
                    f"Step {i + 1}/{step_num}. Time: {time.monotonic() - communication_start_time:.4f} s."
                )
                while time.monotonic() - start_time < (i + 1) * CTRL_DT:
                    pass

        move_to_pose(home_pose, positions, duration)

    def start_cartesian_impedance(self):
        gain = arx5.Gain(np.array([100.0, 100.0, 100.0, 30.0, 30, 5.0]), np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), 5.0, 0.2)
        gain.gripper_kp = 5.0
        gain.gripper_kd = 0.2

        gain.kp()[:] = np.array([100.0, 100.0, 100.0, 30.0, 30, 5.0])
        gain.kd()[:] = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.cartesian_controller.set_gain(gain)

    def update_desired_ee_pose(self, pose: np.ndarray):
        eef_state = arx5.EEFState(pose, 0.0)
        self.cartesian_controller.set_eef_cmd(eef_state)

    def terminate_current_policy(self):
        self.cartesian_controller.set_to_damping()

    def close(self):
        # modified on 2024/10/08 by: zyj
        # self.cartesian_controller.set_to_damping()
        del self.cartesian_controller
        print("disconnect to arx5 robot!")


class ARX5GripperController:
    def __init__(self, controller):
        self.controller = controller

    def start(self, wait=True):
        pass

    def stop(self, wait=True):
        pass

    def start_wait(self):
        self.start(wait=True)

    def stop_wait(self):
        self.stop(wait=True)

    @property
    def is_ready(self):
        return self.controller.is_ready

    def get_state(self):
        return {
            'gripper_position': 1.0,
            'gripper_timestamp': time.time(),
        }

    def get_all_state(self):
        return {
            'gripper_position': np.array([1.0]),
            'gripper_timestamp': np.array([time.time()]),
        }

    def schedule_waypoint(self, pos: float, target_time: float):
        pass


class ARX5InterpolationController(mp.Process):
    def __init__(self,
         shm_manager: SharedMemoryManager,
         model='X5',
         can_interface='can0',
         frequency=1000,
         launch_timeout=3,
         joints_init=None,
         joints_init_duration=None,
         soft_real_time=False,
         verbose=False,
         get_max_k=None,
         receive_latency=0.0
         ):

        if joints_init is not None:
            joints_init = np.array(joints_init)
            # modified on 2024/10/08 by: zyj
            # assert joints_init.shape == (7,)
            assert joints_init.shape == (6,)

        super().__init__(name="ARX5PositionalController")
        self.model = model
        self.can_interface = can_interface
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.joints_init = joints_init
        self.joints_init_duration = joints_init_duration
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose

        # self.robot = Arx5Interface()

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        # build input queue
        example = {
            'cmd': Command.SERVOL.value,
            'target_pose': np.zeros((6,), dtype=np.float64),
            'duration': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=256
        )

        # build ring buffer
        receive_keys = [
            ('ActualTCPPose', 'get_ee_pose'),
            ('ActualQ', 'get_joint_positions'),
            ('ActualQd', 'get_joint_velocities'),
        ]
        example = dict()
        for key, func_name in receive_keys:
            if 'joint' in func_name:
                example[key] = np.zeros(6)
            elif 'ee_pose' in func_name:
                example[key] = np.zeros(6)

        example['robot_receive_timestamp'] = time.time()
        example['robot_timestamp'] = time.time()
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = ring_buffer
        self.receive_keys = receive_keys

    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[ARX5PositionalController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.STOP.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()

    def stop_wait(self):
        self.join()

    @property
    def is_ready(self):
        return self.ready_event.is_set()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= command methods ============
    def servoL(self, pose, duration=0.1):
        """
        duration: desired time to reach pose
        """
        assert self.is_alive()
        assert (duration >= (1 / self.frequency))
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SERVOL.value,
            'target_pose': pose,
            'duration': duration
        }
        self.input_queue.put(message)

    def schedule_waypoint(self, pose, target_time):
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': pose,
            'target_time': target_time
        }
        self.input_queue.put(message)

    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    # def get_ee_pose(self):
    #     return self.robot.get_ee_pose()
    #
    # def get_eef_pose(self):
    #     return self.robot.get_eef_pose()
    #
    # def set_eef_pose(self, state):
    #     self.robot.set_eef_state(state)

        # ========= main loop in process ============
    def run(self):
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))

        # start polymetis interface
        robot = Arx5Interface()

        try:
            if self.verbose:
                print(f"[ARX5PositionalController] Connect to robot")

            # init pose
            if self.joints_init is not None:
                robot.move_to_joint_positions(
                    positions=np.asarray(self.joints_init),
                    time_to_go=self.joints_init_duration
                )

            # main loop
            dt = 1. / self.frequency
            curr_pose = robot.get_ee_pose()

            # use monotonic time to make sure the control loop never go backward
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[curr_pose]
            )

            # start franka cartesian impedance policy
            robot.start_cartesian_impedance(
            )

            t_start = time.monotonic()
            iter_idx = 0
            keep_running = True
            while keep_running:
                # send command to robot
                t_now = time.monotonic()
                # diff = t_now - pose_interp.times[-1]
                # if diff > 0:
                #     print('extrapolate', diff)
                tip_pose = pose_interp(t_now)
                flange_pose = tip_pose

                # send command to robot
                robot.update_desired_ee_pose(flange_pose)

                # update robot state
                state = dict()
                for key, func_name in self.receive_keys:
                    state[key] = getattr(robot, func_name)()

                t_recv = time.time()
                state['robot_receive_timestamp'] = t_recv
                state['robot_timestamp'] = t_recv - self.receive_latency
                self.ring_buffer.put(state)

                # fetch command from queue
                try:
                    # commands = self.input_queue.get_all()
                    # n_cmd = len(commands['cmd'])
                    # process at most 1 command per cycle to maintain frequency
                    commands = self.input_queue.get_k(1)
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']

                    if cmd == Command.STOP.value:
                        keep_running = False
                        # stop immediately, ignore later commands
                        break
                    elif cmd == Command.SERVOL.value:
                        # since curr_pose always lag behind curr_target_pose
                        # if we start the next interpolation with curr_pose
                        # the command robot receive will have discontinouity
                        # and cause jittery robot behavior.
                        target_pose = command['target_pose']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        pose_interp = pose_interp.drive_to_waypoint(
                            pose=target_pose,
                            time=t_insert,
                            curr_time=curr_time,
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print("[ARX5PositionalController] New pose target:{} duration:{}s".format(
                                target_pose, duration))
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']
                        target_time = float(command['target_time'])
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                # regulate frequency
                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[ARX5PositionalController] Actual frequency {1 / (time.monotonic() - t_now)}")

        finally:
            # manditory cleanup
            # terminate
            print('\n\n\n\nterminate_current_policy\n\n\n\n\n')
            robot.terminate_current_policy()
            del robot
            self.ready_event.set()

            if self.verbose:
                print(f"[Arx5PositionalController] Disconnected from robot")