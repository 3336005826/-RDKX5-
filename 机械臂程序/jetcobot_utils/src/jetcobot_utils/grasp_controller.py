#!/usr/bin/env python3
# encoding: utf-8

import logging
from time import sleep

import jetcobot_utils.logger_config as logger_config
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot


RIGHT_90_JOINT1 = 180
WATCH_JOINTS = [RIGHT_90_JOINT1, 0, 0, -85, -7, -3]


class GraspController:
    def __init__(self):
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        logger_config.setup_logger()
        self.func_start = False
        self.garbage_num = [0, 0, 0, 0]

    def init_pose(self):
        self.mc.send_angles([RIGHT_90_JOINT1, 0, 0, 0, 0, -45], 50)
        sleep(2)

    def init_pose2(self):
        self.mc.send_angles([RIGHT_90_JOINT1, 0, -90, 95, 0, -45], 50)
        sleep(2)

    def init_watch_pose(self):
        self.go_watch_position(2)
        logging.info('go_watch_position')
        self.open_gripper(2)

    def open_gripper(self, delay=0):
        self.mc.set_gripper_value(100, 50)
        if delay > 0:
            sleep(delay)

    def close_gripper(self, delay=0):
        self.mc.set_gripper_value(20, 50)
        if delay > 0:
            sleep(delay)

    def rotate_gripper(self, yaw, delay=0):
        self.mc.send_angle(Angle.J6.value, yaw, 50)
        sleep(2)

    def go_watch_position(self, delay=0):
        self.mc.send_angles(WATCH_JOINTS, 50)
        if delay > 0:
            sleep(delay)

    def go_calibration_angles(self, angles):
        self.mc.send_angles(angles, 70)

    def go_angles(self, angles, delay=0):
        self.mc.send_angles(angles, 40)
        if delay > 0:
            sleep(delay)

    def go_radians(self, radians, delay=2):
        self.mc.send_radians(radians, 40)
        if delay > 0:
            sleep(delay)

    def go_coords(self, coords, delay=0):
        self.mc.send_coords(coords, 40, 0)
        if delay > 0:
            sleep(delay)

    def grasp_state(self):
        return self.func_start

    def grasp_get_offset_xy(self, task, kind, origin_x, origin_y):
        offset_x = -0.012
        offset_y = 0.0005
        if kind == 'garbage':
            offset_x = -0.012
            offset_y = 0.0002
        elif kind == 'apriltag':
            offset_x = -0.012
            offset_y = 0.0005
        elif kind == 'color':
            offset_x = -0.012
            offset_y = 0.0005
        return offset_x, offset_y

    def grasp_run(self, task, kind, msg, joint1456=None, height_profile=None):
        if len(msg) == 0:
            logging.info('未识别到物块')
            return

        move_num = 1
        self.func_start = True
        self.garbage_num = [0, 0, 0, 0]

        for name, pos in msg.items():
            try:
                yaw = 0
                if kind == 'color' and len(pos) > 2:
                    yaw = pos[2]

                logging.info(f'name={name}, pos0={pos[0]}, pos1={pos[1]}, yaw={yaw}')
                if pos[1] > 0.27652:
                    logging.info('目标位置超出当前抓取范围')
                    continue

                offset_x, offset_y = self.grasp_get_offset_xy(task, kind, pos[1], -pos[0])
                x = (pos[1] + offset_x) * 1000
                y = (-pos[0] + offset_y) * 1000

                approach_z = 170
                grasp_z = 115
                if isinstance(height_profile, dict):
                    approach_z = int(height_profile.get('approach_z', approach_z))
                    grasp_z = int(height_profile.get('grasp_z', grasp_z))

                self.grasp(
                    task,
                    kind,
                    str(move_num),
                    str(name),
                    yaw,
                    x,
                    y,
                    z=approach_z,
                    grasp_z=grasp_z,
                )
                move_num += 1
            except Exception as exc:
                logging.info(f'grasp_run error={exc}')

        if joint1456 is not None:
            joints_angles = [joint1456[0], 0, 0, joint1456[1], joint1456[2], joint1456[3]]
            self.go_angles(joints_angles, 2)
        logging.info('回到抓取完成后的观察位')
        self.func_start = False

    def grasp(self, task, kind, move_num, name, yaw, x, y, z=170, grasp_z=115, rx=-175, ry=0, rz=-45, speed=40):
        logging.info(str(name))
        coords_init = [x, y, z, rx, ry, rz]
        sleep(1)
        logging.info('1 到达预抓取上方')
        self.go_coords(coords_init, 3)

        logging.info('2 下降到抓取高度')
        self.mc.send_coord(Coord.Z.value, grasp_z, speed)
        sleep(1)

        logging.info('3 合拢夹爪')
        self.close_gripper(1.5)

        logging.info('4 抬升')
        self.mc.send_coord(Coord.Z.value, z, speed)
        sleep(2)

        logging.info('5 去过渡位')
        self.goOverPose(task, kind)

        logging.info('6 去投放位')
        self.goTargetPose(task, kind, move_num, name)

        logging.info('7 打开夹爪')
        self.open_gripper(1)

        logging.info('8 抬起夹爪')
        self.lift_gripper(task, kind, move_num, name)

        logging.info('9 返回过渡位')
        self.goOverPose(task, kind)
        return True

    def goOverPose(self, task, kind):
        if task == 'sorting':
            if kind == 'garbage':
                self.goGarbageOverPose()
            elif kind in ('color', 'apriltag'):
                self.goColorOverPose()
        elif task == 'stacking':
            self.goStackingOverPose()

    def goStackingOverPose(self):
        self.go_radians([-0.726684, -0.000574, -0.290444, -0.690522, 0.0, -0.77226], 2)

    def goStackingUpperPose(self):
        self.go_radians([-0.835744, 0.145222, -1.27141, -0.363342, 0.0, -0.07747], 2)

    def goColorOverPose(self):
        self.go_radians([0.82, -0.2, -0.35, -0.69, 0, -0.77], 2)

    def goGarbageOverPose(self):
        self.go_radians([-0.82, 0, -0.29, -0.69, 0, -0.77], 2)

    def lift_gripper(self, task, kind, move_num, name):
        if task == 'sorting':
            self.mc.send_coord(Coord.Z.value, 180, 40)
            sleep(1.5)
        elif task == 'stacking':
            self.ctrl_gripper_height(150 + int(move_num) * 30, 1.5)

    def ctrl_gripper_height(self, value_z, delay=0):
        self.mc.send_coord(Coord.Z.value, value_z, 40)
        if delay > 0:
            sleep(delay)

    def rise_gripper(self, delay=0):
        self.mc.send_coord(Coord.Z.value, 180, 40)
        if delay > 0:
            sleep(delay)

    def drop_gripper(self, delay=0):
        self.mc.send_coord(Coord.Z.value, 110, 40)
        if delay > 0:
            sleep(delay)

    def ctrl_nod(self):
        for _ in range(2):
            self.mc.send_angle(Angle.J4.value, -30, 50)
            sleep(1)
            self.mc.send_angle(Angle.J4.value, 10, 50)
            sleep(1)
        self.mc.send_angle(Angle.J4.value, 0, 50)
        sleep(1)

    def goTargetPose(self, task, kind, move_num, name):
        if task == 'sorting':
            if kind == 'garbage':
                self.goGarbageSortingPose(name)
            elif kind == 'color':
                self.goColorSortingPose(name)
            elif kind == 'apriltag':
                self.goApriltagSortingPose(name)
        elif task == 'stacking':
            if kind == 'garbage':
                self.goGarbageStackingPose(name, self.garbage_num)
            else:
                self.goStackingPose(move_num)
                sleep(1)

    def goStackingPose(self, move_num):
        if move_num == '1':
            self.goStackingNum1Pose()
        elif move_num == '2':
            self.goStackingNum2Pose()
        elif move_num == '3':
            self.goStackingNum3Pose()
        elif move_num == '4':
            self.goStackingNum4Pose()

    def goColorSortingPose(self, name):
        if name == 'red':
            self.goRedPose()
        if name == 'blue':
            self.goBluePose()
        if name == 'green':
            self.goGreenPose()
        if name == 'yellow':
            self.goYellowPose()

    def goApriltagSortingPose(self, name):
        if name == '1':
            self.goApriltag1fixedPose()
        elif name == '2':
            self.goApriltag2fixedPose()
        elif name == '3':
            self.goApriltag3fixedPose()
        elif name == '4':
            self.goApriltag4fixedPose()
        else:
            self.goApriltag1fixedPose()
        sleep(1)

    def goGarbageStackingPose(self, name, num):
        if name in ('Zip_top_can', 'Newspaper', 'Old_school_bag', 'Book'):
            num[0] = int(num[0]) + 1
            self.goRecyclablePose(num[0])
        elif name in ('Syringe', 'Used_batteries', 'Expired_cosmetics', 'Expired_tablets'):
            num[1] = int(num[1]) + 1
            self.goHazardousWastePose(num[1])
        elif name in ('Fish_bone', 'Watermelon_rind', 'Apple_core', 'Egg_shell'):
            num[2] = int(num[2]) + 1
            self.goFoodWastePose(num[2])
        elif name in ('Cigarette_butts', 'Toilet_paper', 'Peach_pit', 'Disposable_chopsticks'):
            num[3] = int(num[3]) + 1
            self.goResidualWastePose(num[3])

    def goGarbageSortingPose(self, name):
        if name in ('Zip_top_can', 'Newspaper', 'Old_school_bag', 'Book'):
            self.goRecyclablePose()
        elif name in ('Syringe', 'Used_batteries', 'Expired_cosmetics', 'Expired_tablets'):
            self.goHazardousWastePose()
        elif name in ('Fish_bone', 'Watermelon_rind', 'Apple_core', 'Egg_shell'):
            self.goFoodWastePose()
        elif name in ('Cigarette_butts', 'Toilet_paper', 'Peach_pit', 'Disposable_chopsticks'):
            self.goResidualWastePose()

    def limit_garbage_layer(self, layer):
        if int(layer) < 1 or int(layer) > 2:
            return 2
        return layer
#其他垃圾放置位置
    def goResidualWastePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        self.go_coords([130, -225, 120 + int(layer - 1) * 40, -180, -10, 135], 3)
#厨余垃圾放置位置
    def goFoodWastePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        self.go_coords([65, -225, 110 + int(layer - 1) * 40, -180, -10, 135], 3)
#有害垃圾放置位置
    def goHazardousWastePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        self.go_coords([5, -225, 110 + int(layer - 1) * 40, -180, -10, 135], 3)
#可回收垃圾放置位置
    def goRecyclablePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        self.go_coords([-70, -225, 110 + int(layer - 1) * 40, -180, -10, 135], 3)

    def goStackingNum1Pose(self):
        self.go_coords([135, -150, 115, -180, -10, 135], 3)

    def goStackingNum2Pose(self):
        self.go_coords([132, -150, 145, -180, -10, 135], 3)

    def goStackingNum3Pose(self):
        self.go_coords([132, -150, 175, -180, -10, 135], 3)

    def goStackingNum4Pose(self):
        self.go_coords([131, -150, 205, -180, -10, 135], 3)

    def goBoxCenterlayer1Pose(self, grasp=0):
        if grasp == 0:
            coords = [210, -5, 120, -175, 0, 135]
        else:
            coords = [200, -5, 120, -175, 0, 135]
        self.go_coords(coords, 3)

    def goYellowPose(self):
        self.go_coords([140, 215, 115, -175, 0, 135], 3)

    def goRedPose(self):
        self.go_coords([75, 215, 115, -175, 0, 135], 3)

    def goGreenPose(self):
        self.go_coords([10, 215, 115, -175, 0, 135], 3)

    def goBluePose(self):
        self.go_coords([-70, 215, 115, -175, 0, 135], 3)

    def goYellowfixedPose(self):
        self.go_coords([135, 230, 160, -170, 3, 135], 3)

    def goRedfixedPose(self):
        self.go_coords([70, 230, 160, -170, 3, 135], 3)

    def goGreenfixedPose(self):
        self.go_coords([0, 230, 160, -170, 3, 135], 3)

    def goBluefixedPose(self):
        self.go_coords([-70, 230, 160, -170, 3, 135], 3)

    def goApriltag4fixedPose(self, layer=1):
        self.go_coords([142, 140, 110 + 50 * (layer - 1), -170, 3, 135], 2)

    def goApriltag3fixedPose(self, layer=1):
        self.go_coords([77, 140, 110 + 50 * (layer - 1), -170, 3, 135], 2)

    def goApriltag2fixedPose(self, layer=1):
        self.go_coords([2, 140, 110 + 50 * (layer - 1), -170, 3, 135], 2)

    def goApriltag1fixedPose(self, layer=1):
        self.go_coords([-68, 140, 110 + 50 * (layer - 1), -170, 3, 135  ], 2)
