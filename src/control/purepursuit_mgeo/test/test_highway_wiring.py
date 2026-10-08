"""ROS adapter smoke tests without a ROS installation; launch include contracts."""
import importlib.util
import math
import sys
import time
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]


class Message:
    def __init__(self, data=None):
        self.data = data
        self.header = types.SimpleNamespace(frame_id='map', stamp=Stamp())
        self.pose = types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),
                                          orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        self.poses=[]
        self.point=types.SimpleNamespace(x=0.,y=0.,z=0.)
        self.scale=types.SimpleNamespace(z=0.)
        self.color=types.SimpleNamespace(r=0.,g=0.,b=0.,a=0.)
        self.longlCmdType=1
        self.accel=self.brake=self.steering=self.velocity=self.acceleration=0.
    TEXT_VIEW_FACING=9
    ADD=0


class Stamp:
    def __init__(self,value=None):self.value=time.time() if value is None else value
    def to_sec(self):return self.value
    def __sub__(self,other):return Stamp(self.value-other.value)
    @staticmethod
    def now():return Stamp()


class Publisher:
    def __init__(self,*args,**kwargs):self.messages=[]
    def publish(self,message):self.messages.append(message)


def ros_modules():
    ros=types.ModuleType('rospy')
    ros.Time=Stamp
    ros.Duration=lambda x:x
    ros.get_param=lambda name,default=None:default
    ros.Publisher=Publisher
    for name in ('init_node','Subscriber','Timer','logwarn','logwarn_throttle','loginfo_throttle'):
        setattr(ros,name,lambda *a,**k:None)
    modules={'rospy':ros}
    for package,names in {
        'geometry_msgs':['PoseStamped','PointStamped'],
        'nav_msgs':['Odometry','Path'],
        'std_msgs':['Bool','Float64','String'],
        'visualization_msgs':['Marker'],
        'lidar_perception':['LidarObstacleArray'],
        'morai_msgs':['CtrlCmd'],
    }.items():
        mod=types.ModuleType(package+'.msg')
        for name in names:setattr(mod,name,Message)
        modules[package+'.msg']=mod
    return modules


def load_script(name):
    spec=importlib.util.spec_from_file_location(name,PACKAGE/'scripts'/str(name+'.py'))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class AdapterTests(unittest.TestCase):
    def test_final_controller_follows_new_path_and_keeps_pedestrian_stop(self):
        from purepursuit_mgeo.path import PathPoint
        modules=ros_modules()
        with tempfile.TemporaryDirectory() as folder:
            route=Path(folder)/'route.txt'
            route.write_text('0 0 0\n100 0 0\n')
            params={'~path_file':str(route),'~enable_control':True,
                    '~target_speed_mps':80/3.6}
            modules['rospy'].get_param=lambda name,default=None:params.get(name,default)
            with patch.dict(sys.modules,modules):
                module=load_script('purepursuit_mgeo_node')
                node=module.PurePursuitNode()
                node.highway_active=True
                node.controller.points=[PathPoint(float(x),0.,0.) for x in range(100)]
                odom=Message()
                pose=odom.pose
                pose.position.y=-.3
                odom.pose=types.SimpleNamespace(pose=pose)
                odom.twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=80/3.6,y=0.)))
                node.odom_callback(odom)
                node.control_callback(None)
                self.assertGreater(node.command_pub.messages[-1].steering,0.)
                self.assertEqual(node.command_pub.messages[-1].brake,0.)
                node.pedestrian_stop_required=True
                node.control_callback(None)
                self.assertEqual(node.command_pub.messages[-1].brake,1.)
                self.assertEqual(node.command_pub.messages[-1].accel,0.)
                # The environment detector can lag behind the moving car.
                # Physical steering authority is still bounded at road speed.
                node.pedestrian_stop_required=False
                node.highway_active=False
                node.steering_rate_active=False
                node.controller.points=[PathPoint(float(x),8.,0.) for x in range(100)]
                odom.twist.twist.linear.x=19.
                node.control_callback(None)
                limit=math.atan(2.5*node.wheelbase_m/(19.*19.))
                self.assertLessEqual(abs(node.command_pub.messages[-1].steering),limit+1e-9)

    def test_adapter_publishes_rolling_path_and_final_lane_status(self):
        from purepursuit_mgeo.highway import Ego, Lane, Road
        with patch.dict(sys.modules,ros_modules()):
            module=load_script('highway_lane_strategy_node')
            node=module.HighwayNode()
            node.ego=Ego(0.,0.,0.,80/3.6)
            node.enabled=True
            start=time.time()
            for i in range(4):
                now=start+i*.05
                node.odom_at=node.lidar_at=now
                node.lidar_stamp=now
                node.planner.observe(Lane(Road(0.,0.,0.),3.5,'white_solid','white_dashed',now),now,node.ego)
                with patch.object(module.time,'time',return_value=now):
                    node.tick(None)
            self.assertFalse(node.stop_pub.messages[-1].data)
            self.assertTrue(node.active_pub.messages[-1].data)
            self.assertGreater(len(node.path_pub.messages[-1].poses),80)
            self.assertIn('LOCKED',node.state_pub.messages[-1].data)

    def test_entry_uses_global_path_until_gap_change_then_lane_centre(self):
        from purepursuit_mgeo.highway import Ego, Lane, Road
        modules=ros_modules()
        with tempfile.TemporaryDirectory() as folder:
            route=Path(folder)/'route.txt'
            route.write_text(''.join('%d 0.1 0\n' % x for x in range(-5,110)))
            modules['rospy'].get_param=lambda name,default=None: (
                str(route) if name == '~global_path_file' else default)
            with patch.dict(sys.modules,modules):
                module=load_script('highway_lane_strategy_node')
                node=module.HighwayNode()
                node.ego=Ego(0.,0.,0.,80/3.6)
                node.enabled=True
                now=time.time()
                node.odom_at=node.lidar_at=node.lidar_stamp=now
                node.planner.observe(Lane(Road(0.,0.,0.),3.5,'white_dashed',
                                          'white_dashed',now),now,node.ego)
                with patch.object(module.time,'time',return_value=now):
                    node.tick(None)
                self.assertIn('"path_source": "global"',node.state_pub.messages[-1].data)
                self.assertAlmostEqual(node.path_pub.messages[-1].poses[10].pose.position.y,.1)
                node.planner.changes=1
                node.planner.locked=True
                next_time=now+.05
                node.odom_at=node.lidar_at=node.lidar_stamp=next_time
                node.planner.observe(Lane(Road(0.,0.,0.),3.5,'white_solid',
                                          'white_dashed',next_time),next_time,node.ego)
                with patch.object(module.time,'time',return_value=next_time):
                    node.tick(None)
                self.assertIn('"path_source": "lane_center"',node.state_pub.messages[-1].data)
                self.assertAlmostEqual(node.path_pub.messages[-1].poses[10].pose.position.y,0.)

    def test_global_diagonal_waits_inside_lane_when_adjacent_car_blocks_gap(self):
        from purepursuit_mgeo.highway import Ego, Lane, Obstacle, Road
        modules=ros_modules()
        with tempfile.TemporaryDirectory() as folder:
            route=Path(folder)/'route.txt'
            route.write_text(''.join('%d %.3f 0\n' % (x,max(0.,min(3.5,(x-15)*.1)))
                                     for x in range(-5,110)))
            modules['rospy'].get_param=lambda name,default=None: (
                str(route) if name == '~global_path_file' else default)
            with patch.dict(sys.modules,modules):
                module=load_script('highway_lane_strategy_node')
                node=module.HighwayNode()
                node.ego=Ego(0.,0.,0.,80/3.6)
                node.enabled=True
                node.obstacles=[Obstacle(1,0.,3.5,0.,4.6,1.9,80/3.6,0.)]
                start=time.time()
                for i in range(45):
                    now=start+i*.05
                    node.odom_at=node.lidar_at=node.lidar_stamp=now
                    node.planner.observe(Lane(Road(0.,0.,0.),3.5,'white_dashed',
                                              'white_dashed',now),now,node.ego)
                    with patch.object(module.time,'time',return_value=now):
                        node.tick(None)
                self.assertEqual(node.planner.state,'WAIT_GAP')
                self.assertIn('"path_source": "global_guarded"',
                              node.state_pub.messages[-1].data)
                self.assertLessEqual(max(p.pose.position.y for p in
                                         node.path_pub.messages[-1].poses),.2+1e-6)

    def test_braking_keeps_highway_steering_but_not_throttle(self):
        with patch.dict(sys.modules,ros_modules()):
            module=load_script('purepursuit_mgeo_node')
            node=module.PurePursuitNode.__new__(module.PurePursuitNode)
            node.highway_active=True
            command=node.make_command(.01,True,.3,.2)
            self.assertEqual(command.steering,.01)
            self.assertEqual(command.brake,1.)
            self.assertEqual(command.accel,0.)
            node.highway_active=False
            self.assertEqual(node.make_command(.01,True).steering,0.)

    def test_clock_alignment_uses_measurement_age_not_arrival(self):
        with patch.dict(sys.modules,ros_modules()):
            module=load_script('highway_lane_strategy_node')
            node=module.HighwayNode()
            now=time.time()
            header=types.SimpleNamespace(stamp=Stamp(now-.4))
            self.assertAlmostEqual(now-node.measurement_time(header),.4,places=2)
            header.stamp=Stamp(now+3.)
            self.assertEqual(node.measurement_time(header),-float('inf'))


class LaunchTests(unittest.TestCase):
    def test_80kph_defaults_and_highway_sensor_crop_chain(self):
        top=ET.parse(PACKAGE/'launch/highway.launch').getroot()
        defaults={x.attrib['name']:x.attrib.get('default') for x in top.findall('arg')}
        self.assertAlmostEqual(float(defaults['target_speed_mps']),80/3.6,places=5)
        self.assertEqual(defaults['lidar_rear_min_m'],'-80.0')
        files=[PACKAGE/'launch/morai_avoidance_highway_roundabout_final.launch']
        files += [ROOT/'src/common/morai_bringup/launch'/name for name in (
            'morai_udp_ekf_purepursuit_lidar_camera.launch',
            'morai_udp_ekf_purepursuit_lidar_tracking.launch')]
        for file in files:
            tree=ET.parse(file).getroot()
            declared={x.attrib['name'] for x in tree.findall('arg')}
            passed={x.attrib['name'] for x in tree.findall('include/arg')}
            self.assertTrue({'lidar_rear_min_m','lidar_lateral_range_m'} <= declared & passed,str(file))
        pipeline=ROOT/'src/detection/lidar_perception/launch/lidar_algorithm_pipeline.launch'
        tree=ET.parse(pipeline).getroot()
        passed={x.attrib['name']:x.attrib.get('value') for x in tree.findall('include/arg')}
        self.assertEqual(passed['x_min_m'],'$(arg x_min_m)')
        self.assertEqual(passed['y_abs_m'],'$(arg y_abs_m)')


if __name__=='__main__':unittest.main()
