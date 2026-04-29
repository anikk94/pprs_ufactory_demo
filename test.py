import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI
from xarm.version import __version__





class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._ignore_exit_state = False
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._ignore_exit_state:
                return True
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def print_tcp(self):
        pos = self.arm.get_position()[1]
        x  = pos[0]
        y  = pos[1]
        z  = pos[2]
        rx = pos[3]
        ry = pos[4]
        rz = pos[5]
        # print(f'arm position: {pos}')
        # print(f'ret type: {type(pos)}')
        # print(f'x: {x}\ny: {y}\nz: {z}\nrx: {rx}\nry: {ry}\nrz: {rz}\n')
        print(f'(x: {x}, y: {y}, z: {z}, rx: {rx}, ry: {ry}, rz: {rz})')
        return pos

    # def move_x(self):
        

    # Robot Main Run
    def run(self):
        try:
            ##### Python code #####
            # python code 
            # count = 0                 
            # while count < 10:                                        
            while True:                                        
                c_pos = self.print_tcp()
                c_pos[0] += 30
                c_pos[1] += 30
                self.arm.set_position(c_pos[0],c_pos[1],c_pos[2],c_pos[3],c_pos[4],c_pos[5], wait=True)
                c_pos = self.print_tcp()
                # arm.set_pause_time(0.5)
                c_pos[0] -= 30
                c_pos[1] -= 30
                self.arm.set_position(c_pos[0],c_pos[1],c_pos[2],c_pos[3],c_pos[4],c_pos[5], wait=True)
                # arm.set_pause_time(0.5)
                
                # count += 1
                
            # c_pos = self.print_tcp()
            # c_pos[0] += 10
            # c_pos[1] += 10
            # self.arm.set_position(c_pos[0],c_pos[1],c_pos[2],c_pos[3],c_pos[4],c_pos[5])
            # c_pos = self.print_tcp()
            # c_pos[0] -= 50
            # self.arm.set_position(c_pos[0],c_pos[1],c_pos[2],c_pos[3],c_pos[4],c_pos[5])
            # c_pos = self.print_tcp()

            #######################
            # while self.is_alive:
            #     t1 = time.monotonic()
            #     print("[CI #0] {}".format(self._arm.get_cgpio_digital(0)[1]))
            #     time.sleep(0.01)
            #     interval = time.monotonic() - t1
            #     if interval < 0.01:
            #         time.sleep(0.01 - interval)
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        finally:
            self.alive = False
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)









if __name__ == '__main__':

# test 1
    # print('xArmPythonSDK Version: {}'.format(__version__))
    # arm = XArmAPI('192.168.1.221')
    # print('xArm Version: {}'.format(arm.version))
    # arm.disconnect()

# test 2
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.221', baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()
