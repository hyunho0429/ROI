"""Requirement-level regression tests, including closed-loop bicycle runs.

These are synthetic sensor/vehicle tests, not MORAI or on-road validation.
"""
import math
import random
import unittest
from dataclasses import replace

from purepursuit_mgeo.highway import (
    Change, Config, Ego, Highway, Lane, Obstacle, Road,
    clamp, lane_from_json, track_path, wrap,
)
from purepursuit_mgeo.motion import SteeringRateLimiter
from path_planning.longitudinal_controller import PedalSpeedController


SPEED = 80./3.6


def lane(t, y=0., left='white_dashed', x=0., curvature=0.):
    return Lane(Road(x,y,0.,curvature),3.5,left,'white_dashed',t)


def object_at(x, y=0., speed=SPEED, ident=1):
    return Obstacle(ident,x,y,0.,4.6,1.9,speed,0.)


def camera_message(ego, t, center=0., noise=0., left='white_dashed'):
    # Generate ground-truth straight physical boundaries in the moving camera
    # frame, rather than giving the planner an ideal centreline.
    c,s = math.cos(ego.yaw), math.sin(ego.yaw)
    def points(y):
        result=[]
        for x in range(1,31):
            # World horizontal line -> ego camera coordinates at fixed local x.
            local_y = (y-ego.y-s*x)/c + noise
            result.append([float(x),local_y])
        return result
    return dict(timestamp=t, lane_valid=True, output_status='FRESH',
                left_lane=dict(detected=True,type=left,age=10),
                right_lane=dict(detected=True,type='white_dashed',age=10),
                left_boundary_points=points(center+1.75),
                right_boundary_points=points(center-1.75))


class GeometryTests(unittest.TestCase):
    def test_composite_entrance_does_not_unlock_final_solid_lane(self):
        m=camera_message(Ego(0.,0.,0.,SPEED),0.,left='white_solid')
        m['right_lane']['type']='white_solid'
        m['left_outer_lane']=dict(detected=True,type='white_dashed',coef=[0.,0.,1.8],x_range_m=[1.,30.])
        entry=lane_from_json(m,(0.,0.,0.))
        self.assertTrue(entry.left_composite)
        self.assertTrue(entry.may_change)
        m['right_lane']['type']='white_dashed'
        final=lane_from_json(m,(0.,0.,0.))
        self.assertTrue(final.final)
        self.assertFalse(final.may_change)
        m['left_outer_lane']['coef']=[0.,0.,5.25]
        self.assertFalse(lane_from_json(m,(0.,0.,0.)).left_composite)

    def test_arc_projection_and_parallel_shift(self):
        for k in (0., .001, -.003):
            road=Road(100.,-80.,.3,k)
            for s in (-20.,0.,80.):
                for d in (-3.,0.,3.5):
                    actual=road.project(*road.point(s,d))
                    self.assertAlmostEqual(actual[0],s,places=6)
                    self.assertAlmostEqual(actual[1],d,places=6)

    def test_camera_midpoint_compensates_turning_yaw_and_pose(self):
        ego=Ego(42.,3.2,.09,SPEED)
        obs=lane_from_json(camera_message(ego,1.,3.5), (ego.x,ego.y,ego.yaw))
        self.assertAlmostEqual(obs.road.yaw,0.,places=6)
        self.assertAlmostEqual(obs.road.project(42.,3.5)[1],0.,places=6)
        self.assertAlmostEqual(obs.width,3.5,places=6)

    def test_held_coasted_straddling_and_two_lane_pair_rejected(self):
        ego=Ego(0.,0.,0.,SPEED)
        for modify in (
            lambda m:m.update(output_status='HELD'),
            lambda m:m['left_lane'].update(coasted=True),
            lambda m:m.update(straddling_lane={'detected':True}),
            lambda m:m.update(left_boundary_points=[[x,y+3.5] for x,y in m['left_boundary_points']]),
        ):
            m=camera_message(ego,0.)
            modify(m)
            with self.assertRaises(ValueError):lane_from_json(m,(0.,0.,0.))

    def test_quintic_one_lane_terminal_heading_and_acceleration(self):
        for yaw in (-.01,0.,.01):
            change=Change.create(Road(0.,0.,0.),Ego(0.,.1,yaw,SPEED),3.5,Config())
            self.assertAlmostEqual(change.lateral(0),.1)
            self.assertEqual(change.lateral(change.length+100),3.5)
            co=change.coefficients
            self.assertAlmostEqual(sum(i*v for i,v in enumerate(co)),0.,places=10)
            self.assertAlmostEqual(sum(i*(i-1)*v for i,v in enumerate(co)),0.,places=10)
            self.assertGreater(change.length,80.)


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.p=Highway()
        self.e=Ego(0.,0.,0.,SPEED)

    def run_ready(self, objects=(), lidar_stamp=True, left='white_dashed'):
        r=None
        for i in range(45):
            t=i*.05
            self.p.observe(lane(t,left=left),t,self.e)
            r=self.p.step(t,self.e,list(objects),True,lidar_stamp=i if lidar_stamp else 1)
        return r

    def test_no_activation_without_environment(self):
        self.p.observe(lane(0.),0.,self.e)
        self.assertEqual(self.p.step(0.,self.e,[],False).state,'OFF')

    def test_pre_activation_turn_does_not_pin_old_road_or_lock_highway(self):
        for i in range(5):
            self.p.observe(lane(i*.05,left='white_solid'),i*.05,self.e)
        new=Lane(Road(100.,50.,math.pi/2),3.5,'white_dashed','white_dashed',1.)
        self.assertTrue(self.p.observe(new,1.,Ego(100.,50.,math.pi/2,SPEED)))
        self.assertFalse(self.p.locked)
        self.assertAlmostEqual(self.p.road.yaw,math.pi/2)

    def test_empty_target_commits_after_distinct_lidar_frames(self):
        self.assertEqual(self.run_ready().state,'CHANGE')
        self.assertAlmostEqual(self.p.change.target,3.5)

    def test_duplicate_lidar_does_not_authorize_change(self):
        self.assertEqual(self.run_ready(lidar_stamp=False).state,'WAIT_GAP')

    def test_near_rear_or_side_blocks_entry_without_braking_cruise(self):
        for x in (-15.,0.):
            self.setUp()
            r=self.run_ready([object_at(x,3.5,SPEED+3.)])
            self.assertEqual(r.state,'WAIT_GAP')
            self.assertFalse(r.stop)
            self.assertAlmostEqual(r.target_speed,SPEED)

    def test_far_rear_closure_during_change_blocks_entry(self):
        r=self.run_ready([object_at(-55.,3.5,SPEED+12.)])
        self.assertEqual(r.reason,'target_rear_gap')

    def test_rear_gap_accounts_for_slowing_to_target_lane_lead(self):
        r=self.run_ready([object_at(80.,3.5,14.,1),object_at(-45.,3.5,SPEED,2)])
        self.assertEqual(r.state,'WAIT_GAP')
        self.assertEqual(r.reason,'target_rear_gap')

    def test_overtaken_source_lead_does_not_brake_during_safe_change(self):
        e=Ego(0.,0.,0.,19.)
        c=Change.create(Road(0.,0.,0.),e,3.5,self.p.cfg)
        source=object_at(35.,0.,17.,1)
        rear=object_at(-45.,3.5,21.,2)
        self.assertTrue(self.p._gap(c,e,[source,rear])[0])
        self.p.change=c
        speed,stop,follow=self.p._follow(c.road,e,[source,rear])
        self.assertEqual(follow,{})
        self.assertFalse(stop)
        self.assertAlmostEqual(speed,SPEED)
        close=object_at(12.,0.,10.,3)
        self.assertTrue(self.p._follow(c.road,e,[close])[1])

    def test_solid_left_keeps_driving_and_never_changes(self):
        r=self.run_ready(left='white_solid')
        self.assertEqual(r.state,'LOCKED')
        self.assertFalse(r.stop)
        self.assertAlmostEqual(r.target_speed,SPEED)
        # Flickering dashed classification cannot unlock the final lane.
        self.p.observe(lane(2.3),2.3,self.e)
        self.assertEqual(self.p.step(2.3,self.e,[],True,lidar_stamp=46).state,'LOCKED')

    def test_single_solid_is_immediate_veto(self):
        for i in range(20):
            t=i*.05; self.p.observe(lane(t),t,self.e)
            self.p.step(t,self.e,[],True,lidar_stamp=i)
        self.p.observe(lane(1.,left='white_solid'),1.,self.e)
        self.assertNotEqual(self.p.step(1.,self.e,[],True,lidar_stamp=20).state,'CHANGE')

    def test_wrong_camera_lane_cannot_move_fixed_destination(self):
        self.run_ready()
        change=self.p.change
        self.assertFalse(self.p.observe(lane(2.3,7.),2.3,self.e))
        self.assertIs(self.p.change,change)
        self.assertAlmostEqual(change.target,3.5)

    def test_lateral_yaw_must_settle_before_first_change(self):
        self.e=replace(self.e,yaw=.05)
        self.assertEqual(self.run_ready().state,'HOLD')

    def test_yaw_and_rear_vehicle_do_not_create_false_lead(self):
        road=Road(0.,0.,0.)
        e=replace(self.e,yaw=.15)
        target,stop,lead=self.p._follow(road,e,[object_at(-3.),object_at(20.,3.5,10.)])
        self.assertEqual(lead,{})
        self.assertFalse(stop)
        self.assertEqual(target,SPEED)

    def test_front_follow_uses_bumper_gap_and_absolute_road_speed(self):
        target,stop,lead=self.p._follow(Road(0.,0.,0.),self.e,[object_at(45.,0.,14.)])
        self.assertAlmostEqual(lead['gap'],45.-1.5-2.3-4.635/2,places=2)
        self.assertEqual(lead['lead_speed'],14.)
        self.assertLess(target,SPEED)
        self.assertFalse(stop)

    def test_imminent_front_collision_retains_emergency_brake(self):
        self.p.observe(lane(0.),0.,self.e)
        result=self.p.step(0.,self.e,[object_at(12.,0.,0.)],True,lidar_stamp=1)
        self.assertTrue(result.stop)
        self.assertEqual(result.reason,'front_collision_emergency')

    def test_sensor_staleness_is_explicit_not_mission_brake(self):
        self.p.observe(lane(0.),0.,self.e)
        result=self.p.step(2.,self.e,[],True,lidar_age=2.)
        self.assertTrue(result.stop)
        self.assertEqual(result.reason,'lidar_stale')
        result=self.p.step(2.05,self.e,[],True,lidar_stamp=2)
        self.assertEqual(result.reason,'lane_geometry_stale')
        self.assertLess(result.target_speed,SPEED)


class ClosedLoopTests(unittest.TestCase):
    def simulate(self, seed=0, noise=0., delay=0., actuator_tau=0., curvature=0., divider_dropout=False):
        rng=random.Random(seed)
        p=Highway(); e=Ego(0.,0.,0.,SPEED)
        road=Road(0.,0.,0.,curvature)
        p.observe(Lane(road,3.5,'white_dashed','white_dashed',-delay),0.,e)
        queue=[]; changes=[]; previous='OFF'; angle=0.; peak=0.; holds=[]
        limiter=SteeringRateLimiter(.2,.05)
        for i in range(680):
            t=i*.05
            s,d=road.project(e.x,e.y)
            n=int(clamp(math.floor((d+1.75)/3.5),0,2))
            in_crossing=False
            # The frozen change path is parameterized in its own road frame.
            if divider_dropout and p.change is not None:
                station,_=p.change.road.project(e.x,e.y)
                in_crossing=.15 < station/p.change.length < .90
            if i%2==0 and not in_crossing:
                # Simulate original capture time + delayed delivery, including
                # occasional false adjacent-lane association and missed frames.
                if noise and i%47==0:
                    observed=lane(t,3.5*(n+1),x=e.x)
                elif curvature:
                    reference=road.shifted(s,3.5*n)
                    observed=Lane(reference,3.5,'white_solid' if n==2 else 'white_dashed','white_dashed',t)
                else:
                    measured=replace(e,y=e.y)  # capture pose, before future motion
                    m=camera_message(measured,t,3.5*n,rng.uniform(-noise,noise),
                                     'white_solid' if n==2 else 'white_dashed')
                    if noise:
                        heading_noise=rng.uniform(-.003,.003)
                        curvature_noise=rng.uniform(-.00012,.00012)
                        for key in ('left_boundary_points','right_boundary_points'):
                            m[key]=[[x,y+heading_noise*x+.5*curvature_noise*x*x] for x,y in m[key]]
                    try:
                        observed=lane_from_json(m,(measured.x,measured.y,measured.yaw))
                    except ValueError as exc:
                        if str(exc) != 'boundaries_do_not_bracket_vehicle':
                            raise
                        observed=None
                if observed is not None and (not noise or i%43):
                    queue.append((t+delay,observed))
            while queue and queue[0][0]<=t:
                _,measurement=queue.pop(0)
                p.observe(measurement,t,e)
            r=p.step(t,e,[],True,lidar_stamp=i)
            if divider_dropout and p.change is not None and r.diagnostics.get('lane_age',0)>1.:
                self.assertGreater(r.target_speed,SPEED-.1,(t,r.reason,r.diagnostics))
            if r.state=='CHANGE' and previous!='CHANGE':
                changes.append((t,d,e.yaw-road.heading(s)))
            if r.reason=='change_complete_parallel':holds.append(t)
            previous=r.state
            self.assertFalse(r.stop,(t,r.reason,r.diagnostics))
            command,_,_=track_path(r.path,e)
            command=limiter.update(command,t,True)
            angle+=clamp(.05/max(actuator_tau,.05),0.,1.)*(command-angle)
            e=Ego(e.x+e.speed*math.cos(e.yaw)*.05,e.y+e.speed*math.sin(e.yaw)*.05,
                  e.yaw+e.speed/3.*math.tan(angle)*.05,e.speed)
            _,lateral=road.project(e.x,e.y)
            peak=max(peak,lateral)
        self.assertEqual(p.changes,2,(p.state,r.reason,r.diagnostics))
        self.assertEqual(p.state,'LOCKED')
        self.assertEqual(len(changes),2)
        self.assertGreaterEqual(changes[1][0]-holds[0],5.)
        self.assertLess(abs(changes[1][1]-3.5),.22)
        self.assertLess(abs(changes[1][2]),math.radians(1.5))
        self.assertLess(abs(road.project(e.x,e.y)[1]-7.),.15)
        self.assertLess(peak,7.25)  # vehicle remains well inside final solid boundary

    def test_80kph_two_changes_final_lock(self):
        self.simulate()

    def test_80kph_noisy_delayed_camera_actuator_and_wrong_lane(self):
        for seed in range(3):
            with self.subTest(seed=seed):
                self.simulate(seed,noise=.06,delay=.25,actuator_tau=.15)

    def test_80kph_gentle_curved_road(self):
        self.simulate(curvature=.001)

    def test_80kph_divider_temporarily_hides_camera(self):
        self.simulate(divider_dropout=True)

    def test_front_follow_converges_without_stop_or_collision(self):
        for initial_gap in (35.,50.,80.):
            p=Highway(); e=Ego(0.,0.,0.,SPEED)
            pid=PedalSpeedController(kp=.18,ki=.02,kd=0.)
            front=initial_gap+1.5+4.635/2+2.3
            smallest=initial_gap
            for i in range(900):
                t=i*.05
                p.observe(lane(t,left='white_solid',x=e.x),t,e)
                r=p.step(t,e,[object_at(front,0.,14.)],True,lidar_stamp=i)
                self.assertFalse(r.stop)
                a,b=pid.compute(r.target_speed,e.speed,t)
                e=Ego(e.x+e.speed*.05,0.,0.,max(0.,e.speed+(3*a-8*b)*.05))
                front+=14*.05
                smallest=min(smallest,front-e.x-1.5-4.635/2-2.3)
            self.assertGreater(smallest,10.)
            self.assertAlmostEqual(e.speed,14.,delta=.08)

    def test_video_pattern_source_lead_and_fast_rear_during_camera_gap(self):
        """A safe pass must not slow into the approaching destination car."""
        p=Highway(); ego=Ego(0.,0.,0.,19.)
        p.road=Road(0.,0.,0.)
        p.change=Change.create(p.road,ego,3.5,p.cfg)
        p.change_started=0.
        p.state='CHANGE'
        p.last_good=0.
        p.speed_command=19.
        source_x,rear_x=35.,-45.
        pid=PedalSpeedController(kp=.18,ki=.02,kd=0.)
        smallest_rear=math.inf
        for i in range(111):
            t=i*.05
            objects=[object_at(source_x,0.,17.,1),object_at(rear_x,3.5,21.,2)]
            r=p.step(t,ego,objects,True,lidar_stamp=i)
            self.assertFalse(r.stop,(t,r.reason))
            self.assertGreaterEqual(r.target_speed,18.9,(t,r.reason,r.diagnostics))
            accel,brake=pid.compute(r.target_speed,ego.speed,t)
            wheel,_,_=track_path(r.path,ego)
            ego=Ego(ego.x+ego.speed*math.cos(ego.yaw)*.05,
                    ego.y+ego.speed*math.sin(ego.yaw)*.05,
                    ego.yaw+ego.speed/3*math.tan(wheel)*.05,
                    max(0.,ego.speed+(3*accel-8*brake)*.05))
            source_x+=17*.05
            rear_x+=21*.05
            smallest_rear=min(smallest_rear,ego.x-rear_x-6.)
        self.assertGreater(smallest_rear,20.)
        self.assertLess(abs(ego.y-3.5),.2)


if __name__=='__main__':
    unittest.main()
