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

from pickit_scan import pickit_search

from datetime import datetime



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
        
    def transform_pick_pose(self, pose):
        x, y, z, rx, ry, rz = pose
        new_x=-(y*1000)-73.68
        new_y=x*1000+740
        new_z=z*1000+60

        return [new_x, new_y, new_z, rx, ry, rz]


    # Robot Main Run
    def run(self):
        try:
            ##### Python code #####
            # python code 

            pHome = [469.189117, -3.040936, 411.463287, 178.781682, -0.014782, 0.115107]
            pScan = [442, 265, 249, 180, 0, 0]
            pPrePick = pScan
            pPreDrop = [511, -199, 347, 180, 0, 0]
            p1 = [547, 247.7, 250.6, 178.8, 0, 0.1]
            speed = 100
            pick_count = 0
            fail_count = 0


            # clear gripper
            arm.set_vacuum_gripper(False)
            # move to home position
            self.arm.set_position(*pHome, wait=True,speed=speed)
            # move arm to scan position
            self.arm.set_position(*pScan, wait=True,speed=speed)
            
            start_time = datetime.now()

            results=pickit_search()


            while True:     
                pick_pose = results[0]                                   
                # c_pos = self.print_tcp()
                # c_pos[0] += 30
                # c_pos[1] += 30
                # self.arm.set_position(c_pos[0],c_pos[1],c_pos[2],c_pos[3],c_pos[4],c_pos[5], wait=True)
                # print(pick_pose['position'])
                pose = self.transform_pick_pose(pick_pose['position']+[180, 0, 0])
                # print(pose)

                arm.set_vacuum_gripper(True)
                # pre-pick approach
                self.arm.set_position(*pose, wait=True,speed=speed)
                # approach
                self.arm.set_position(z=pose[2]-35, wait=True)
                # retreat
                self.arm.set_position(z=pose[2]+35, wait=True)
                # # post-pick retreat
                self.arm.set_position(*pScan, wait=True)

                griper_code, grip_success = arm.get_vacuum_gripper()
                if grip_success:
                    pick_count += 1
                    self.arm.set_position(*pPreDrop, wait=True)
                    arm.set_vacuum_gripper(False)
                    arm.set_pause_time(1, wait=True)

                # print('\nnext\n')
                time_delta = datetime.now()-start_time

                print(f'[{datetime.now().strftime("%H:%M:%S")}] THROUGHPUT: {(pick_count/time_delta.total_seconds()): < .3g} parts/second')

                self.arm.set_position(*pScan, wait=True,speed=speed)
                results = pickit_search()
                
                # break
                # c_pos = self.print_tcp()
                # # arm.set_pause_time(0.5)
                # c_pos[0] -= 30
                # c_pos[1] -= 30
                # self.arm.set_position(c_pos[0],c_pos[1],c_pos[2],c_pos[3],c_pos[4],c_pos[5], wait=True)



                # arm.set_pause_time(0.5)
                

                
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
