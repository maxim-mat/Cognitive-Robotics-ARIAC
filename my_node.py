#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc

from geometry_msgs.msg import TransformStamped
from nist_gear.msg import Order, Model, LogicalCameraImage, VacuumGripperState

from std_srvs.srv import Trigger
from nist_gear.srv import AGVControl, AGVToAssemblyStation, GetMaterialLocations, VacuumGripperControl, AssemblyStationSubmitShipment

import sys
import copy
import yaml
import re
from math import pi, sqrt
from check import solve_problems


def start_competition():
    rospy.wait_for_service('/ariac/start_competition')
    rospy.ServiceProxy('/ariac/start_competition', Trigger)()


def end_competition():
    rospy.wait_for_service('/ariac/end_competition')
    rospy.ServiceProxy('/ariac/end_competition', Trigger)()


def submit_kitting_shipment(agv, assembly_station, shipment_type):
    rospy.wait_for_service('/ariac/' + agv + '/submit_shipment')
    rospy.ServiceProxy('/ariac/' + agv + '/submit_shipment', AGVToAssemblyStation)(assembly_station, shipment_type)


def submit_assembly_shipment(assembly_station, shipment_type):
    rospy.wait_for_service('/ariac/' + assembly_station + '/submit_shipment')
    rospy.ServiceProxy('/ariac/' + assembly_station + '/submit_shipment', AssemblyStationSubmitShipment)(shipment_type)


def get_order():
    order = rospy.wait_for_message('/ariac/orders', Order)
    return order


def get_part_type_location(part):
    print("wait")
    rospy.wait_for_service('/ariac/material_locations')
    print("response")
    response = rospy.ServiceProxy('/ariac/material_locations',
                                  GetMaterialLocations)(part.type)
    reachable_location = None
    for loc in response.storage_units:
        if 'bin' in loc.unit_id:
            reachable_location = loc.unit_id
            break
    return reachable_location


def get_parts_from_cameras():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # wait for all cameras to be broadcasting
    all_topics = rospy.get_published_topics()
    camera_topics = [t for t, _ in all_topics if '/ariac/logical_camera' in t]
    for topic in camera_topics:
        rospy.wait_for_message(topic, LogicalCameraImage)

    camera_frame_format = r"logical_camera_(.*?)_frame"
    all_frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()).keys()
    part_frames = [f for f in all_frames if re.match(camera_frame_format, f)]

    objects = []
    for frame in part_frames:
        try:
            world_tf = tf_buffer.lookup_transform(
                'world',
                frame,
                rospy.Time(),
                rospy.Duration(0.1)
            )
            ee_tf = tf_buffer.lookup_transform(
                frame,
                'ee_link',
                rospy.Time(),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            continue

        tf_time = rospy.Time(
            world_tf.header.stamp.secs,
            world_tf.header.stamp.nsecs
        )
        if rospy.Time.now() - tf_time > rospy.Duration(1.0):
            continue

        model = Model()
        model.type = re.match(camera_frame_format, frame).group(1)
        model.pose.position = world_tf.transform.translation
        model.pose.orientation = ee_tf.transform.rotation
        objects.append(model)
    return objects


def get_target_world_pose(target, agv):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    h_frame = ''

    if agv == 'agv1':
        h_frame = 'kit_tray_1'
    elif agv == 'agv2':
        h_frame = 'kit_tray_2'
    elif agv == 'agv3':
        h_frame = 'kit_tray_3'
    elif agv == 'agv4':
        h_frame = 'kit_tray_4'

    tf_msg = TransformStamped()
    if h_frame:
        tf_msg.header.frame_id = h_frame
    else:
        assert(h_frame), "No AGV provided"

    tf_msg.header.stamp = rospy.Time()
    tf_msg.child_frame_id = 'target_frame'
    tf_msg.transform.translation = target.pose.position
    tf_msg.transform.rotation = target.pose.orientation

    for _ in range(5):
        tf_broadcaster.sendTransform(tf_msg)

    # tf lookup fails occasionally, this automatically retries the lookup
    MAX_ATTEMPTS = 10
    attempts = 0
    #print("attempt loop")
    while attempts < MAX_ATTEMPTS:
        try:
            #print("world target lookup")
            world_target_tf = tf_buffer.lookup_transform(
                'world',
                'target_frame',
                rospy.Time(),
                rospy.Duration(0.1)
            )
            #print("ee target lookup")
            ee_target_tf = tf_buffer.lookup_transform(
                'target_frame',
                'ee_link',
                rospy.Time(),
                rospy.Duration(0.1)
            )
            break
        except:
            continue

    world_target = copy.deepcopy(target)
    world_target.pose.position = world_target_tf.transform.translation
    world_target.pose.orientation = ee_target_tf.transform.rotation
    return world_target


class MoveitRunner():

    def __init__(self, group_names, node_name='my_node', ns='', robot_description='robot_description'):
        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)

        self.robot = mc.RobotCommander(ns+'/'+robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)
        self.groups = {}
        for group_name in group_names:
            group = mc.MoveGroupCommander(
                group_name,
                robot_description=ns+'/'+robot_description,
                ns=ns
            )
            group.set_goal_tolerance(0.05)
            self.groups[group_name] = group

        name = 'kitting_robot' if len(group_names) < 3 else 'gantry_robot'

        self.set_preset_location()
        self.goto_preset_location('start', name)

    def set_preset_location(self):

        locations = {}

        name = 'start'
        kitting_arm = [0, 3.141594222190707, -1.128743290405139, 1.5106304587276407, 4.25, -1.5079761953269788, 0]
        gantry_torso = [0, 0, 0]
        gantry_arm = [0.0, -1.13, 1.88, -0.72, 1.55, 0.83]
        locations[name] = (kitting_arm, gantry_torso, gantry_arm)

        name = 'bin8'
        kitting_arm = [1.3458887656258813, -0.5601138939850792, -0.2804510290896989, 0, -0.8072468824120538, 1.5385783777411373, 0.8298981409931709]
        gantry_torso = [-2.48400006879773, -1.6336322021423504, 0, 3.4200004668605506]
        gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (kitting_arm, gantry_torso, gantry_arm)

        name = 'standby'
        kitting_arm = [2.70, 3.141594222190707, -1.01, 1.88, 3.77, -1.55, 0]
        gantry_torso = [0, 0, 0]
        gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (kitting_arm, gantry_torso, gantry_arm)

        name = 'agv4'
        kitting_arm = [1.50, 3.141594222190707, -1.01, 1.88, 3.77, -1.55, 0]
        gantry_torso = [0, 0, 0]
        gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (kitting_arm, gantry_torso, gantry_arm)

        self.locations = locations

    def goto_preset_location(self, location_name, robot_type):

        group = None
        if robot_type == 'kitting_robot':
            group = self.groups['kitting_arm']
        elif robot_type == 'gantry_robot':
            group = self.groups['gantry_full']

        kitting_arm, gantry_torso, gantry_arm = self.locations[location_name]
        location_pose = group.get_current_joint_values()

        if robot_type == "kitting_robot":
            location_pose[:] = kitting_arm

        print("Location Pose:", location_pose)

        MAX_ATTEMPTS = 5
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            # assert(attempts < MAX_ATTEMPTS)

    def goto_location(self, control_group, location):
        print(self.groups.keys())
        group = self.groups[control_group]
        while not group.go(location, wait=True):
            pass

    def move_part(self, part, target, part_location, agv, robot_type):

        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        near_pick_pose = copy.deepcopy(part.pose)
        pick_pose = copy.deepcopy(part.pose)
        place_pose = copy.deepcopy(target.pose)

        near_pick_pose.position.z += 0.1
        pick_pose.position.z += 0.050
        place_pose.position.z += 0.1

        print("part_pose: ", part)
        print("near_pick_pose: ", near_pick_pose.position.z)
        self.goto_preset_location(part_location)
        gm.activate_gripper()

        path = [near_pick_pose, pick_pose]
        self.cartesian_move(group, path)

        num_attempts = 0
        MAX_ATTEMPTS = 20
        while not gm.is_object_attached() and num_attempts < MAX_ATTEMPTS:
            num_attempts += 1
            rospy.sleep(0.1)

        if not gm.is_object_attached():
            self.goto_preset_location(part_location)
            self.goto_preset_location('standby')
            self.goto_preset_location('start')
            return False

        self.goto_preset_location('standby')
        self.goto_preset_location(agv)

        path = [place_pose]
        self.cartesian_move(group, path)

        gm.deactivate_gripper()

        self.goto_preset_location('standby')
        self.goto_preset_location('start')
        return True

    def cartesian_move(self, group, waypoints):
        print("CARTESIAN MOVE")
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        group.execute(plan, wait=True)


class GripperManager:
    def __init__(self, ns):
        self.ns = ns

    def activate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(True)

    def deactivate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(False)

    def is_object_attached(self):
        status = rospy.wait_for_message(self.ns + 'state', VacuumGripperState)
        return status.attached


if __name__ == '__main__':

    kitting_group_names = ['kitting_arm']
    gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']

    moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')
    moveit_runner_gantry = MoveitRunner(gantry_group_names, ns='/ariac/gantry')

    start_competition()
    order = get_order()
    # print(order)
    agv_states = {'agv1': [], 'agv2': [], 'agv3': [], 'agv4': []}

    all_known_parts = get_parts_from_cameras()
    part_locs = ((part.pose.position.x, part.pose.position.y, part.pose.position.z) for part in all_known_parts)

    print(rospy.get_published_topics())

    for shipment in order.kitting_shipments:

        # print(shipment)

        if shipment.agv_id == 'any':
            active_agv = 'agv1'
        else:
            active_agv = shipment.agv_id

        agv_state = agv_states[active_agv]

        while True:
            valid_products = []
            for product in shipment.products:
                if product not in agv_state:
                    valid_products.append(product)

            candidate_moves = []
            for part in all_known_parts:
                for product in valid_products:
                    if product.type in part.type:
                        candidate_moves.append((part, product))

            # these should be the actual position of object in the simulation however we couldn't retrieve most of them
            kit_rob_loc = (0, 0)
            ass_rob_loc = (0, 0)
            ass_stations_loc = ((2, 2), (0, 2))
            kit_stations_loc = ((-2, 0), (2, 0))
            parts_required_ass = ((1, 2, 3), (1, 3))
            parts_required_kit = ((2, 4, 4), (1, 3, 5))
            avilable_parts = part_locs
            # for this test one 3 is missing, 3 are in different locations, 6 isn't required, 5 is not available
            # goal: parts required is empty or no more required parts are available.
            problems = [(kit_rob_loc, ass_rob_loc, ass_stations_loc, kit_stations_loc, parts_required_ass,
                         parts_required_kit, avilable_parts)]
            strategy = solve_problems(problems)
            print(strategy)
            # this function should execute the strategy given by the planner,
            # however we had trouble controlling the robots and couldn't implement it
            execute_strategy(strategy)

            #print(part_locs[0])
            #moveit_runner_gantry.goto_location("gantry_torso", part_locs[0])

            #print(candidate_moves)

            if candidate_moves:
                part, target = candidate_moves[0]

                print("getting world target")
                world_target = get_target_world_pose(target, active_agv)
                print("getting part type location")
                part_location = get_part_type_location(part)

                #print("world_target: ", world_target)
                print("part_location: ", part_location)

                move_successful = moveit_runner.move_part(
                    part,
                    world_target,
                    part_location,
                    active_agv
                )
                if move_successful:
                    all_known_parts.remove(part)
                    agv_state.append(target)
            else:
                break

        submit_kitting_shipment(active_agv, shipment.station_id, shipment.shipment_type)
        agv_states[active_agv] = []

    end_competition()
    print('Done')
