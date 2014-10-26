#!/usr/bin/env python
import argparse, herbpy, prpy.rave, openravepy, numpy, os, signal, subprocess, time

#export OPENRAVE_DATA=/opt/pr/pr_ordata/ordata:/opt/pr/librarian_ordata/ordata:${OPENRAVE_DATA}

TABLE_PATH = 'objects/table.kinbody.xml'
TABLE_POSE = numpy.array([
    [  0.,  0.,  1., 0.668 ],
    [  1.,  0.,  0., 0.000 ],
    [  0.,  1.,  0., 0.000 ],
    [  0.,  0.,  0., 1.000 ],
])
OBJECT_PATH = 'objects/household/fuze_bottle.kinbody.xml'
OBJECT_POSE = numpy.array([
    [ 1.,  0.,  0.,  0.79049718 ],
    [ 0.,  1.,  0., -0.34460402 ],
    [ 0.,  0.,  1.,  0.73686087 ],
    [ 0.,  0.,  0.,  1.         ],
])
GRASP_POSE = numpy.array([
    [ 0.,  0.,  1.,  0.53820671 ],
    [ 1.,  0.,  0., -0.34998364 ],
    [ 0.,  1.,  0.,  0.85162171 ],
    [ 0.,  0.,  0.,  1.         ],
])
GHOST_COLOR_GOOD = numpy.array([ 0x00, 0xFF, 0x00, 0x77 ], dtype=float) / 0xFF
GHOST_COLOR_BAD = numpy.array([ 0xFF, 0x00, 0x00, 0x77 ], dtype=float) / 0xFF
GHOST_COLOR = numpy.array([ 196, 196, 0xBB, 0x77 ], dtype=float) / 0xFF
GHOST_HIGHLIGHT = numpy.array([ 0xFF, 0x99, 0x33, 0x77 ], dtype=float) / 0xFF

parser = argparse.ArgumentParser()
parser.add_argument('--iterations', type=int, default=5)
parser.add_argument('--timeout', type=float, default=20.0)
parser.add_argument('--window-id', type=str, default=None)
parser.add_argument('--output-dir', type=str, default='')
parser.add_argument('planner_name', type=str, default='BITstar')
args = parser.parse_args()

env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')

robot.SetDOFVelocityLimits(2.0 * numpy.ones(robot.GetDOF()))
robot.SetDOFAccelerationLimits(50.0 * numpy.ones(robot.GetDOF()))

planner = openravepy.RaveCreatePlanner(env, 'OMPL')
params = openravepy.Planner.PlannerParameters()
params.SetExtraParameters(
    '<planner_type>{planner:s}</planner_type>'\
    '<time_limit>{timeout:f}</time_limit>'\
    '<stop_on_each_solution_improvement>1</stop_on_each_solution_improvement>'.format(
        planner=args.planner_name, timeout=args.timeout)
)

simplifier = openravepy.RaveCreatePlanner(env, 'OMPLSimplifier')
simplifier_params = openravepy.Planner.PlannerParameters()
simplifier_params.SetExtraParameters('<time_limit>10.0</time_limit>')

with env:
    dof_indices, dof_values = robot.configurations.get_configuration('relaxed_home')
    robot.SetDOFValues(dof_values, dof_indices)
    full_dof_values = robot.GetDOFValues()

    manipulator = robot.right_arm
    manipulator.SetActive()
    home_dof_values = robot.GetActiveDOFValues()

    prpy.rave.add_object(env, 'table', TABLE_PATH, TABLE_POSE)
    prpy.rave.add_object(env, 'object', OBJECT_PATH, OBJECT_POSE)

def plan_to_configuration(config):
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(config)

    simplifier_params.SetRobotActiveJoints(robot)

    traj = openravepy.RaveCreateTrajectory(env, '')
    
    with env:
        with robot:
            planner.InitPlan(robot, params)
            result = planner.PlanPath(traj)

    if result == openravepy.PlannerStatus.HasSolution:
        return traj
    else:
        return None

def plan_to_ik(manipulator, pose, try_all=False):
    ik_options = openravepy.IkFilterOptions.CheckEnvCollisions
    ik_solutions = manipulator.FindIKSolutions(pose, ik_options)

    for i, ik_solution in enumerate(ik_solutions):
        traj = plan_to_configuration(ik_solution)

        if traj is not None:
            print('Solution {:d} succeeded!'.format(i))
            return traj
        else:
            print('Solution {:d} failed'.format(i))

        break # FIXME

    return None

def execute_trajectory(robot, traj):
    result = openravepy.planningutils.RetimeTrajectory(traj)
    assert result == openravepy.PlannerStatus.HasSolution
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

class GhostKinBody(object):
    def __init__(self, body, visual=False, color=None, scale=1.0):
        self.env = body.GetEnv()
        self.body = body
        self.handles = dict() 
        self.color = None

        if color is None:
            color = numpy.array([ 0.0, 1.0, 0.0, 0.5 ])

        with self.env:
            self.parameters = self._Create(body, scale=scale, visual=visual)
            self.update(self.body, color)

    def set_visible(self, flag):
        for handle in self.handles.itervalues():
            if handle is not None:
                handle.SetShow(flag)

    def update(self, body, color=None):
        with body.GetEnv():
            # Redraw the mesh to change its color.
            if self.color is None or (color is not None and any(color != self.color)):
                for link_name, base_parameters in self.parameters.iteritems():
                    parameters = dict()
                    parameters.update(base_parameters)
                    parameters['colors'] = color
                    self.handles[link_name] = self.env.drawtrimesh(**parameters)

                self.color = color

            # Efficiently update the transforms.
            for link_name, handle in self.handles.iteritems():
                if handle is not None:
                    pose = body.GetLink(link_name).GetTransform()
                    handle.SetTransform(pose)

    def _Create(self, body, visual, scale):
        parameters = dict()

        for link in body.GetLinks():
            if link.IsVisible() or True:
                handle = self._CreateGhostLink(link, visual, scale)
                if handle is not None:
                    parameters[link.GetName()] = handle

        return parameters

    def _CreateGhostLink(self, link, visual, scale):
        points = []
        indices = []

        for geom in link.GetGeometries():
            if visual:
                mesh_path = geom.GetRenderFilename()
                if mesh_path is None:
                    continue

                mesh = env.ReadTrimeshURI(mesh_path)
                if mesh is None:
                    continue
            else:
                mesh = geom.GetCollisionMesh()

            # Transform the points into the geometry frame.
            num_points = mesh.vertices.shape[0]
            geom_points_H = numpy.column_stack((mesh.vertices, numpy.ones(num_points)))
            geom_points_H[:, 0:3] *= scale
            geom_points = numpy.dot(geom.GetTransform(), geom_points_H.T).T[:, 0:3]

            # Offset the triangle indices to match the global array.
            geom_indices = mesh.indices + len(points)

            points.extend(geom_points)
            indices.extend(geom_indices)

        if len(points) > 0 and len(indices) > 0:
            return {
                'points': numpy.copy(points),
                'indices': numpy.copy(indices),
            }
        else:
            return None

def visualize_trajectories(robot, traj, ghost_trajs, color=None, colors=None):
    all_trajs = [ traj ] + list(ghost_trajs)
    duration = max(traj.GetDuration() for traj in all_trajs)

    assert color is None or colors is None
    assert colors is None or len(colors) == len(ghost_trajs)

    if color is not None:
        colors = [ color ] * len(ghost_trajs)

    with env:
        ghost_robots = [ GhostKinBody(robot, visual=True, color=color) for color in colors ]

        # Infer which DOF(s) are in each ghost trajectory:
        for ghost_traj, ghost_robot in zip(ghost_trajs, ghost_robots):
            cspec = ghost_traj.GetConfigurationSpecification()
            used_indices, _ = cspec.ExtractUsedIndices(robot)

            # Hide links that are not active.
            for link in robot.GetLinks():
                is_visible = [ robot.DoesDOFAffectLink(dof_index, link.GetIndex()) \
                               for dof_index in used_indices ]

                # TODO: Switch to a nicer API for this.
                if not any(is_visible):
                    ghost_robot.handles[link.GetName()] = None

    t = 0.0
    time_start = time.time()
    while t <= duration:
        t = time.time() - time_start

        with env:
            # Update the ghost robots.
            # TODO: This should not only change ActiveDOFs.
            for ghost_traj, ghost_robot in zip(ghost_trajs, ghost_robots):
                waypoint = ghost_traj.Sample(t)
                cspec = ghost_traj.GetConfigurationSpecification()
                dof_values = cspec.ExtractJointValues(waypoint, robot, robot.GetActiveDOFIndices(), 0)

                robot.SetActiveDOFValues(dof_values)
                ghost_robot.update(robot)

            # Update the actual robot model.
            waypoint = traj.Sample(t)
            cspec = traj.GetConfigurationSpecification()
            dof_values = cspec.ExtractJointValues(waypoint, robot, robot.GetActiveDOFIndices(), 0)
            robot.SetActiveDOFValues(dof_values)

            # Update the viewer.
            env.UpdatePublishedBodies()
            env.GetViewer().EnvironmentSync()

    return ghost_robots

def start_recording(window_id, output_path):
    args = [ 'recordmydesktop', '--full-shots', '--no-sound', '--fps', '30',
             '--windowid', window_id, '-o', output_path ]
    return subprocess.Popen(args)

def stop_recording(proc):
    proc.send_signal(signal.SIGINT)
    proc.wait()


ik_options = openravepy.IkFilterOptions.CheckEnvCollisions
config = manipulator.FindIKSolutions(GRASP_POSE, ik_options)[0, :]

params.SetRobotActiveJoints(robot)
params.SetGoalConfig(config)

trajectories = []
arm_links = [
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger0_0'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger0_1'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger0_2'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger1_0'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger1_1'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger1_2'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger2_1'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/finger2_2'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/hand_base'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam1'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam2'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam3'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam4'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam5'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam6'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam7'),
    openravepy.RaveGetEnvironment(1).GetKinBody('herb').GetLink('/right/wam_base'),
]

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx

jet = cm = plt.get_cmap('hot') 
cNorm  = colors.Normalize(vmin=-0.2, vmax=1.0)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
colormap_colors = []

with env:
    with robot:
        planner.InitPlan(robot, params)

        for num_iteration in xrange(args.iterations):
            traj = openravepy.RaveCreateTrajectory(env, '')

            print('Planning trajectory {:d} of {:d} using {:s}'.format(
                num_iteration + 1, args.iterations, args.planner_name))
            result = planner.PlanPath(traj)
            assert result == openravepy.PlannerStatus.HasSolution

            # Color trajectory by iteration number.
            if args.iterations > 1:
                r = num_iteration / (args.iterations - 1.0)
            else:
                r = 1.0

            assert 0.0 <= r <= 1.0
            color = numpy.array(scalarMap.to_rgba(r))
            color[3] = 0.5

            # Render the trajectory.
            openravepy.planningutils.RetimeTrajectory(traj)
            trajectories.append(traj)
            colormap_colors.append(color)

raw_input('Press <ENTER> to record.')

if args.window_id is not None:
    proc = start_recording(args.window_id, 'or_ompl_bitstar')

for link in arm_links:
    link.SetVisible(False)

handles = []

for itraj in xrange(len(trajectories)):
    print 'Executing Trajectory[{:d}]'.format(itraj + 1)

    time.sleep(1.0)
    colors = [ GHOST_COLOR for _ in xrange(itraj) ] + [ GHOST_HIGHLIGHT ]
    handles = visualize_trajectories(robot, trajectories[0], trajectories[0:itraj + 1], colors=colors)

    time.sleep(1.0)
    del handles

    with env:
        robot.SetDOFValues(dof_values, dof_indices)

#
print 'Rendering all trajectories at once.'

time.sleep(1.0)
robot.SetDOFValues(dof_values, dof_indices)
handles = visualize_trajectories(robot, trajectories[-1], trajectories[0:-1], colors=colormap_colors[0:-1])

time.sleep(1.0)
del handles

if args.window_id is not None:
    time.sleep(1.0)
    proc = stop_recording(proc)
