#!/usr/bin/env python

"""
Interface for manipulators that can be connected via TCP/IP sockets.
"""

import socket
from rtde.rtde import RTDE
import numpy as np
from utils import *


class Manipulator:
    def __init__(self, ip, port_read, port_write):
        assert isinstance(ip, str)
        assert isinstance(port_read, int) or isinstance(port_read, str)
        assert isinstance(port_write, int) or isinstance(port_write, str)
        if type(port_read) is str:
            port_read = int(port_read)

        if type(port_write) is str:
            port_write = int(port_write)

        self.ip = ip
        self.port_write = port_write
        self.port_read = port_read
        self.coordinates_mapping = None

        try:
            self.socket_read = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_write = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_read.connect((self.ip, self.port_read))
            self.socket_write.connect((self.ip, self.port_write))
            self.socket_write.settimeout(5)
            self.socket_read.settimeout(5)
            print("[Socket -- IP: {0}. Write port: {1}, read port: {2}]\n".format(ip, self.port_write, self.port_read))
        except exc:
            print("[Socket cannot be created. Exception occured:\n{0}]\n".format(exc))

        # set up RTDE connection with robot
        # for more fields visit
        # https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
        self.rtde = RTDE(ip, 30004)
        self.rtde.connect()
        self.rtde.get_controller_version()
        state_names = ['actual_q', 'actual_TCP_pose']
        state_types = ['VECTOR6D', 'VECTOR6D']
        self.rtde.send_output_setup(state_names, state_types)
        self.rtde.send_start()

    def __del__(self):
        self.socket_read.close()
        self.socket_write.close()
        self.rtde.send_pause()
        self.rtde.disconnect()
        print("\n[Sockets succesfully closed.]\n".format(self.ip))

    def set_mapping(self, matrix):
        assert isinstance(matrix, np.ndarray)
        assert matrix.shape == (4, 4)
        self.coordinates_mapping = matrix
        print(
            "Coordinates mapping will be applied:\n{0}\n From now on specify robot's pose in your coordinate system.".format(
                matrix))

    def reset_mapping(self):
        self.coordinates_mapping = None

    def get_mapping(self):
        return self.coordinates_mapping

    def get_pose(self):
        state = self.rtde.receive()
        return state.actual_TCP_pose

    def get_joints(self):
        state = self.rtde.receive()
        return state.actual_q

    @robot_command
    def move(self, trajectory, is_movej=True, is_pose=True, a=1, v=1, use_mapping=False):
        assert isinstance(trajectory, list)
        assert len(trajectory) > 0
        assert isinstance(is_movej, bool)
        assert isinstance(is_pose, bool)
        assert isinstance(trajectory[0], np.ndarray)
        assert trajectory[0].size == 6

        # if user provides coordinates expressed not in robot system transform them to proper system
        if use_mapping:
            assert self.coordinates_mapping is not None
            assert self.coordinates_mapping.shape == (4, 4)

        print("Trajectory has {0} target points".format(len(trajectory)))
        for i, point in enumerate(trajectory):

            command = ""

            if is_movej:
                command += "movej("
            else:
                command += "movel("

            if is_pose:
                command += "p"

            if use_mapping and is_pose:
                point = mat2pose(np.linalg.inv(self.coordinates_mapping).dot(pose2mat(point)))

            print(point)
            command += "[{}, {}, {}, {}, {}, {}], ".format(*point)
            command += "a={0},v={1}".format(a, v)
            command += ")\n"

            self.socket_write.send(command)
            self.wait_for_move_end(point, is_pose)
            print("Achieved {0} target points".format(i))

    def wait_for_move_end(self, target_position, is_pose):
        start_time = time.time()
        while True:
            is_in_position = True
            if is_pose:
                current_position = self.get_pose()
            else:
                current_position = self.get_joints()

            for i, el in enumerate(target_position):
                if (target_position[i] < current_position[i] - 0.07) or (
                        target_position[i] > current_position[i] + 0.07):
                    is_in_position = False
            elapsed_time = time.time() - start_time
            if is_in_position or elapsed_time > 5:
                break

    @robot_command
    def grip(self, range_open):
        assert isinstance(range_open, float) or isinstance(range_open, int)
        command = rg6_cmd(range_open)
        self.socket_write.send(command)
