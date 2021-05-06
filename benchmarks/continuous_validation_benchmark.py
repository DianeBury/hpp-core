# {{{2 Imports and argument parsing
from __future__ import print_function
from CORBA import Any, TC_long, TC_float
from hpp.corbaserver.manipulation import Robot, loadServerPlugin, createContext, newProblem, ProblemSolver, ConstraintGraph, Rule, Constraints, CorbaClient
from hpp.gepetto.manipulation import ViewerFactory
from hpp_idl.hpp import Error as HppError
import sys, argparse, numpy as np, time, rospy
try:
    import tqdm
    def progressbar_iterable(iterable, *args, **kwargs):
        return tqdm.tqdm(iterable, *args, **kwargs)
    def progressbar_object(*args, **kwargs):
        return tqdm.tqdm(*args, **kwargs)
except ImportError:
    def progressbar_iterable(iterable, *args, **kwargs):
        return iterable
    def progressbar_object(*args, **kwargs):
        class faketqdm:
            def set_description(self, *args, **kwargs):
                pass
            def update(self, *args, **kwargs):
                pass
            def write(self, s):
                print(s)
            def close(self):
                pass
        return faketqdm()

# parse arguments
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of Pyrene manipulating a box')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
p.add_argument ('--n-random-handles', type=int, default=None,
                help="Generate a random model with N handles.")
args = p.parse_args ()
if args.context != defaultContext:
    createContext (args.context)
isSimulation = args.context == "simulation"
# 2}}}

# {{{2 Robot and problem definition

#Robot.urdfFilename = "package://tiago_data/robots/tiago_steel_without_wheels.urdf"
#Robot.srdfFilename = "package://tiago_data/srdf/tiago.srdf"
try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro("package://tiago_description/robots/tiago.urdf.xacro", "robot:=steel", "end_effector:=pal-hey5", "ft_sensor:=schunk-ft")
Robot.srdfString = ""

if args.n_random_handles is None:
    srdf_cylinder = "package://agimus_demos/srdf/cylinder.srdf"
else:
    from generate_obstacle_model import generate_srdf
    bvh_file = "/home/jmirabel/devel/hpp/src/agimus-demos/meshes/cylinder.stl"
    srdf_cylinder = "/tmp/cylinder.srdf"
    with open(srdf_cylinder, 'w') as output:
        generate_srdf(bvh_file, args.n_random_handles, output)

class Cylinder:
    urdfFilename = "package://agimus_demos/urdf/cylinder.urdf"
    srdfFilename = srdf_cylinder
    rootJointType = "freeflyer"

class Driller:
    urdfFilename = "package://gerard_bauzil/urdf/driller_with_qr_drill.urdf"
    srdfFilename = "package://gerard_bauzil/srdf/driller.srdf"
    rootJointType = "freeflyer"

class Table:
    urdfFilename = "package://agimus_demos/urdf/P72-table.urdf"
    srdfFilename = "package://agimus_demos/srdf/table.srdf"
    rootJointType = "freeflyer"

## Reduce joint range for security
def shrinkJointRange (robot, ratio):
    for j in robot.jointNames:
        if j[:6] != "tiago/": continue
        tj = j[6:]
        if tj.startswith("torso") or tj.startswith("arm") or tj.startswith("head"):
            bounds = robot.getJointBounds (j)
            if len (bounds) == 2:
                width = bounds [1] - bounds [0]
                mean = .5 * (bounds [1] + bounds [0])
                m = mean - .5 * ratio * width
                M = mean + .5 * ratio * width
                robot.setJointBounds (j, [m, M])

print("context=" + args.context)
loadServerPlugin (args.context, "manipulation-corba.so")
newProblem()
client = CorbaClient(context=args.context)
client.basic._tools.deleteAllServants()
client.manipulation.problem.selectProblem (args.context)

robot = Robot("robot", "tiago", rootJointType="planar", client=client)
crobot = robot.hppcorba.problem.getProblem().robot()

from tiago_fov import TiagoFOV, TiagoFOVGuiCallback
from hpp import Transform

qneutral = crobot.neutralConfiguration()
qneutral[robot.rankInConfiguration['tiago/hand_thumb_abd_joint']] = 1.5707
qneutral[robot.rankInConfiguration['tiago/hand_index_abd_joint']]  = 0.35
qneutral[robot.rankInConfiguration['tiago/hand_middle_abd_joint']] = -0.1
qneutral[robot.rankInConfiguration['tiago/hand_ring_abd_joint']]   = -0.2
qneutral[robot.rankInConfiguration['tiago/hand_little_abd_joint']] = -0.35
removedJoints = [
            'tiago/caster_back_left_1_joint',
            'tiago/caster_back_left_2_joint',
            'tiago/caster_back_right_1_joint',
            'tiago/caster_back_right_2_joint',
            'tiago/caster_front_left_1_joint',
            'tiago/caster_front_left_2_joint',
            'tiago/caster_front_right_1_joint',
            'tiago/caster_front_right_2_joint',
            'tiago/suspension_left_joint',
            'tiago/wheel_left_joint',
            'tiago/suspension_right_joint',
            'tiago/wheel_right_joint',

            # Comment this 3 joints otherwise sot is not happy
            #'tiago/hand_index_joint',
            #'tiago/hand_mrl_joint',
            #'tiago/hand_thumb_joint',

            'tiago/hand_index_abd_joint',
            'tiago/hand_index_virtual_1_joint',
            'tiago/hand_index_flex_1_joint',
            'tiago/hand_index_virtual_2_joint',
            'tiago/hand_index_flex_2_joint',
            'tiago/hand_index_virtual_3_joint',
            'tiago/hand_index_flex_3_joint',
            'tiago/hand_little_abd_joint',
            'tiago/hand_little_virtual_1_joint',
            'tiago/hand_little_flex_1_joint',
            'tiago/hand_little_virtual_2_joint',
            'tiago/hand_little_flex_2_joint',
            'tiago/hand_little_virtual_3_joint',
            'tiago/hand_little_flex_3_joint',
            'tiago/hand_middle_abd_joint',
            'tiago/hand_middle_virtual_1_joint',
            'tiago/hand_middle_flex_1_joint',
            'tiago/hand_middle_virtual_2_joint',
            'tiago/hand_middle_flex_2_joint',
            'tiago/hand_middle_virtual_3_joint',
            'tiago/hand_middle_flex_3_joint',
            'tiago/hand_ring_abd_joint',
            'tiago/hand_ring_virtual_1_joint',
            'tiago/hand_ring_flex_1_joint',
            'tiago/hand_ring_virtual_2_joint',
            'tiago/hand_ring_flex_2_joint',
            'tiago/hand_ring_virtual_3_joint',
            'tiago/hand_ring_flex_3_joint',
            'tiago/hand_thumb_abd_joint',
            'tiago/hand_thumb_virtual_1_joint',
            'tiago/hand_thumb_flex_1_joint',
            'tiago/hand_thumb_virtual_2_joint',
            'tiago/hand_thumb_flex_2_joint',
            ]
crobot.removeJoints(removedJoints, qneutral)

del crobot
robot.insertRobotSRDFModel("tiago", "package://tiago_data/srdf/tiago.srdf")
robot.insertRobotSRDFModel("tiago", "package://tiago_data/srdf/pal_hey5_gripper.srdf")
robot.setJointBounds('tiago/root_joint', [-5, 5, -5, 5])
ps = ProblemSolver(robot)
ps.loadPlugin("manipulation-spline-gradient-based.so")
vf = ViewerFactory(ps)

vf.loadRobotModel (Table, "table")
robot.setJointBounds('table/root_joint', [-2, 2, -2, 2, -2, 2])

srdf_disable_collisions_fmt = """  <disable_collisions link1="{}" link2="{}" reason=""/>\n"""
srdf_disable_collisions = """<robot>"""

linka, linkb, enabled = robot.hppcorba.robot.autocollisionPairs()
for la, lb, en in zip(linka, linkb, enabled):
    if not en: continue
    disable = False
    # Disable collision between caster wheels and anything else
    if not disable:
        disable = ('caster' in la or 'caster' in lb)
    # Disable collision between tiago/hand (except hand_safety_box_0) and all other tiago links
    if not disable:
        hand_vs_other = False
        for l in [la, lb]:
            if l.startswith("tiago/hand_") and l != "tiago/hand_safety_box_0":
                disable = True
                break
    # Disable collision between driller/* and tiago/arm_[1234567]_link
    if not disable:
        for l0, l1 in [ (la, lb), (lb, la) ]:
            if l0.startswith("driller/") and l1.startswith("tiago/arm_") and l1.endswith("_link_0") and l1[10] in "1234567":
                disable = True
                break
    if disable:
        srdf_disable_collisions += srdf_disable_collisions_fmt.format(la[:la.rfind('_')], lb[:lb.rfind('_')])
# TODO
srdf_disable_collisions += "</robot>"
robot.client.manipulation.robot.insertRobotSRDFModelFromString("", srdf_disable_collisions)

try:
    v = vf.createViewer()
except:
    print("Did not find viewer")

joint_bounds = {}
def setRobotJointBounds(which):
    for jn, bound in joint_bounds[which]:
        robot.setJointBounds(jn, bound)

joint_bounds["default"] = [ (jn, robot.getJointBounds(jn)) for jn in robot.jointNames ]
shrinkJointRange(robot, 0.6)
joint_bounds["grasp-generation"] = [ (jn, robot.getJointBounds(jn)) for jn in robot.jointNames ]
setRobotJointBounds("default")
shrinkJointRange(robot, 0.95)
joint_bounds["planning"] = [ (jn, robot.getJointBounds(jn)) for jn in robot.jointNames ]

#ps.selectPathValidation("Graph-Discretized", 0.05)
ps.selectPathValidation("Graph-Dichotomy", 0)
#ps.selectPathValidation("Graph-Progressive", 0.02)
ps.selectPathProjector("Progressive", 0.2)
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
if isSimulation:
    ps.setMaxIterProjection (1)

ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)
ps.setParameter("SteeringMethod/Carlike/turningRadius", 0.05)

q0 = robot.getCurrentConfig()
q0[:4] = [0, -0.9, 0, 1]
#q0[robot.rankInConfiguration['tiago/torso_lift_joint']] = 0.15
q0[robot.rankInConfiguration['tiago/torso_lift_joint']] = 0.34
q0[robot.rankInConfiguration['tiago/arm_1_joint']] = 0.10
q0[robot.rankInConfiguration['tiago/arm_2_joint']] = -1.47
q0[robot.rankInConfiguration['tiago/arm_3_joint']] = -0.16
q0[robot.rankInConfiguration['tiago/arm_4_joint']] = 1.87
q0[robot.rankInConfiguration['tiago/arm_5_joint']] = -1.57
q0[robot.rankInConfiguration['tiago/arm_6_joint']] = 1.3
q0[robot.rankInConfiguration['tiago/arm_7_joint']] = 0.00

q0[robot.rankInConfiguration['table/root_joint']:] = [0,0.3,0,0,0,1,0]
# 2}}}

# {{{2 Constraint graph initialization

# {{{3 Constraint definition
def lockJoint(jname, q, cname=None, constantRhs=True):
    if cname is None:
        cname = jname
    s = robot.rankInConfiguration[jname]
    e = s+robot.getJointConfigSize(jname)
    ps.createLockedJoint(cname, jname, q[s:e])
    ps.setConstantRightHandSide(cname, constantRhs)
    return cname

# # {{{3 Constraint graph instanciation
from hpp.corbaserver.manipulation import ConstraintGraphFactory #TODO
graph = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(graph)
factory.setGrippers([])
factory.setObjects([], [], [])
factory.generate()

cproblem = ps.hppcorba.problem.getProblem()

cgraph = cproblem.getConstraintGraph()

graph.initialize()

# # {{{2 Benchmark tiago

from progress.bar import IncrementalBar, Bar
from pathlib import Path
import csv
import numpy as np
import scipy as sp
from scipy import stats
import matplotlib.pyplot as plt
import pandas as pd

def shootValidConfigArmNeutral(Nmax=200, q_ref=q0):
    for i in range(Nmax):
        q = robot.shootRandomConfig()
        q[robot.rankInConfiguration['tiago/torso_lift_joint']] = 0.34
        q[robot.rankInConfiguration['tiago/arm_1_joint']] = 0.10
        q[robot.rankInConfiguration['tiago/arm_2_joint']] = -1.47
        q[robot.rankInConfiguration['tiago/arm_3_joint']] = -0.16
        q[robot.rankInConfiguration['tiago/arm_4_joint']] = 1.87
        q[robot.rankInConfiguration['tiago/arm_5_joint']] = -1.57
        q[robot.rankInConfiguration['tiago/arm_6_joint']] = 1.3
        q[robot.rankInConfiguration['tiago/arm_7_joint']] = 0.00
        q[robot.rankInConfiguration['table/root_joint']:] = [0,0.3,0,0,0,1,0]
        q[robot.rankInConfiguration['tiago/hand_index_joint']] = q_ref[robot.rankInConfiguration['tiago/hand_index_joint']]
        q[robot.rankInConfiguration['tiago/hand_mrl_joint']] = q_ref[robot.rankInConfiguration['tiago/hand_mrl_joint']]
        q[robot.rankInConfiguration['tiago/hand_thumb_joint']] = q_ref[robot.rankInConfiguration['tiago/hand_thumb_joint']]
        q[robot.rankInConfiguration['tiago/head_1_joint']] = q_ref[robot.rankInConfiguration['tiago/head_1_joint']]
        q[robot.rankInConfiguration['tiago/head_2_joint']] = q_ref[robot.rankInConfiguration['tiago/head_2_joint']]
        if robot.configIsValid(q):
            return True, q
    return False, None

def shootValidConfigFixedBase(Nmax=200, q_ref=None):
    root_joint = robot.rankInConfiguration['tiago/root_joint']
    for i in range(Nmax):
        q = robot.shootRandomConfig()
        q[robot.rankInConfiguration['table/root_joint']:] = [0,0.3,0,0,0,1,0]
        if q_ref is not None:
            q[root_joint:root_joint+4] = q_ref[root_joint:root_joint+4]
        if robot.configIsValid(q):
            return True, q
    return False, None

def shootValidConfigFullBody(Nmax=200):
    for i in range(Nmax):
        q = robot.shootRandomConfig()
        q[robot.rankInConfiguration['table/root_joint']:] = [0,0.3,0,0,0,1,0]
        if robot.configIsValid(q):
            return True, q
    return False, None

def shootValidConfigs(Nmax=200, q_ref=q0, case='FullBody'):
    if case.lower() in 'fullbody':
        res1, q1 = shootValidConfigFullBody(Nmax)
        res2, q2 = shootValidConfigFullBody(Nmax)
        if res1 and res2:
            return True, q1, q2
        else:
            return False, None, None
    if case.lower() in 'fixedbase':
        res1, q1 = shootValidConfigFixedBase(Nmax)
        res2, q2 = shootValidConfigFixedBase(Nmax, q1)
        if res1 and res2:
            return True, q1, q2
        else:
            return False, None, None
    if case.lower() in 'armneutral':
        res1, q1 = shootValidConfigArmNeutral(Nmax, q_ref)
        res2, q2 = shootValidConfigArmNeutral(Nmax, q_ref)
        if res1 and res2:
            return True, q1, q2
        else:
            return False, None, None

def validatePath(q1,q2):
    # Start the timer
    start = time.time()
    # Test path
    res, pathid , report = ps.directPath(q1, q2, True)
    # End timer
    end = time.time()
    elapsed_time = end - start
    return res, elapsed_time

# Test the given paths for all the continuous validation methods
# (basic, with memory, with vmax, with sorting, complete)
# and print and return the computation time
def testPath(q1,q2):
    valid = {}
    times = {}
    # Test BASIC
    ps.selectPathValidation("DichotomyBasic", 0)
    graph.initialize()
    res, time = validatePath(q1,q2)
    times['BASIC'] = time
    valid['BASIC'] = res
    # Test MEMORY
    ps.selectPathValidation("DichotomyMemory", 0)
    graph.initialize()
    res, time = validatePath(q1,q2)
    times['MEMORY'] = time
    valid['MEMORY'] = res
    # Test VMAX
    ps.selectPathValidation("DichotomyVmax", 0)
    graph.initialize()
    res, time = validatePath(q1,q2)
    times['VMAX'] = time
    valid['VMAX'] = res
    # Test SORTING
    ps.selectPathValidation("DichotomySorting", 0)
    graph.initialize()
    res, time = validatePath(q1,q2)
    times['SORTING'] = time
    valid['SORTING'] = res
    # Test COMPLETE
    ps.selectPathValidation("Dichotomy", 0)
    graph.initialize()
    res, time = validatePath(q1,q2)
    times['COMPLETE'] = time
    valid['COMPLETE'] = res
    return valid, times

def all_valid(results):
    for key in results:
        if not results[key]:
            return False
    return True

def add_new_times(all_times, new_time):
    for key in all_times:
        all_times[key].append(new_time[key])

def launch_tests(Ntests = 1, case='FullBody'):
    bar = Bar('Processing', max=Ntests)
    print("Launching benchmark, case", case, "for", Ntests, "paths")
    all_times = {'BASIC':[], 'MEMORY':[], 'VMAX':[], 'SORTING':[], 'COMPLETE':[]}
    valid_paths = 0
    configs = []
    for i in range(Ntests):
        # Generate 2 random configurations
        res, q1, q2 = shootValidConfigs(200, case=case)
        if res:
            configs.append([q1,q2])
            # Generate and test the straight path between the 2 configs
            results, times = testPath(q1, q2)
            if all_valid(results):
                valid_paths += 1
            add_new_times(all_times, times)
            # Print elapsed time in logs
            # print("Test", i, "- elapsed time:", elapsed_time)
            bar.next()
    bar.finish()
    return configs, valid_paths, all_times

def get_stats(all_times):
    stats = {}
    for key in all_times:
        stats[key] = {}
        stats[key]['min'] =  min(all_times[key])
        stats[key]['max'] =  max(all_times[key])
        stats[key]['mean'] =  sum(all_times[key]) / len(all_times[key])
    return stats

def plot_hist(all_times, key):
    print(key)
    data = all_times[key]
    plt.hist(data, bins=100)
    plt.show()
    

def plot_box(all_times):
    cases = ['BASIC', 'SORTING', 'VMAX', 'MEMORY', 'COMPLETE']
    data = [all_times[key] for key in cases]
    fig, ax = plt.subplots()
    ax.set_title('Continuous validation benchmark')
    ax.boxplot(data)
    plt.yscale('log')
    plt.xticks(range(1,6), cases)
    plt.show()

def pretty_dict_string(dico, indent=0, output=None):
    pretty_string = ""
    for key, value in dico.items():
        if type(value) is dict:
            pretty_string += '    ' * indent + str(key) + '\n'
            pretty_string += pretty_dict_string(value, indent+1)
        else:
            pretty_string +=  '    ' * indent + str(key) + ' : ' + str(dico[key]) + '\n'
        if indent == 0:
            pretty_string += '\n'
    return pretty_string

def write_raw_data(filename, times):
    cases = ['BASIC', 'SORTING', 'VMAX', 'MEMORY', 'COMPLETE']
    with open(filename, 'w') as csvfile:
        csvwriter = csv.DictWriter(csvfile, fieldnames=cases)
        rows = [ {  'BASIC': times['BASIC'][i],
                    'VMAX': times['VMAX'][i],
                    'MEMORY': times['MEMORY'][i],
                    'SORTING': times['SORTING'][i],
                    'COMPLETE': times['COMPLETE'][i]
                } for i in range(len(times['BASIC']))]
        csvwriter.writeheader()
        csvwriter.writerows(rows)

def launch_all_tests(Ntests=1, write=True):
    results = {}
    # Arm Neutral test
    configs, valid_paths, times = launch_tests(Ntests, case='ArmNeutral')
    print(valid_paths, "valid paths out of", Ntests)
    results['ArmNeutral'] = get_stats(times)
    if write:
        filename0 = './data/benchmark_raw_data_armneutral.csv'
        print("Writing file:", filename0)
        write_raw_data(find_free_filename(filename0, '.csv'), times)
    print()
    # Fixed Base test
    configs, valid_paths, times = launch_tests(Ntests, case='FixedBase')
    print(valid_paths, "valid paths out of", Ntests)
    results['FixedBase'] = get_stats(times)
    if write:
        filename0 = './data/benchmark_raw_data_fixedbase.csv'
        print("Writing file:", filename0)
        write_raw_data(find_free_filename(filename0, '.csv'), times)
    print()
    # FullBody test
    configs, valid_paths, times = launch_tests(Ntests, case='FullBody')
    print(valid_paths, "valid paths out of", Ntests)
    results['FullBody'] = get_stats(times)
    if write:
        filename0 = './data/benchmark_raw_data_fullbody.csv'
        print("Writing file:", filename0)
        write_raw_data(find_free_filename(filename0, '.csv'), times)
    print()
    return results
    #pretty_print_dict(results)

def find_free_filename(filename0, extension):
    i = 0
    file_ok = False
    while not file_ok:
        if i == 0:
            filename = filename0
        else:
            name = filename0.split(extension)[0]
            filename = name + str(i) + extension
        myfile = Path(filename)
        if myfile.is_file():
            i += 1
        else:
            file_ok = True
    return filename

def read_data(filename):
    time_data = {'BASIC':[], 'MEMORY':[], 'VMAX':[], 'SORTING':[], 'COMPLETE':[]}
    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            row_float = {key:float(value) for (key,value) in row.items()}
            add_new_times(time_data, row_float)
    return time_data

filename_armneutral = './data/benchmark_raw_data_armneutral.csv'
filename_fixedbase = './data/benchmark_raw_data_fullbody.csv'
filename_fullbody = './data/benchmark_raw_data_fullbody.csv'

def launch_benchmark(Ntests=1000):
    results = launch_all_tests(Ntests)
    results_string = pretty_dict_string(results)
    filename0 = "./data/benchmark_results.txt"
    filename = find_free_filename(filename0, '.txt')
    print("Writing results in file :", filename)
    with open(filename, 'w+') as myfile:
        myfile.write("Tests on " + str(Ntests) + " paths\n")
        myfile.write(results_string)

# 2}}}

#vim: foldmethod=marker foldlevel=1
