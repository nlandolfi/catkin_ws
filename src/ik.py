import openravepy as rave
from openravepy.misc import InitOpenRAVELogging 
import numpy as np
InitOpenRAVELogging() 

env = rave.Environment()
env.SetViewer('qtcoin')
module = rave.RaveCreateModule(env, 'urdf')

name = module.SendCommand('load /home/ubuntu/catkin_ws/src/j2s7s300_standalone.urdf /home/ubuntu/catkin_ws/src/jaco7dof_standalonev1.srdf')
jaco = env.GetRobot(name)
# kb = env.GetKinBody(name)
#print(dir(jaco))

print(jaco.GetManipulators())
jaco.SetActiveManipulator("j2s7s300")
ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(
		jaco,iktype=rave.IkParameterization.Type.Transform6D
		)
if not ikmodel.load():
    ikmodel.autogenerate()

with env:
    Tgoal = np.array([
	[ 0,  -0.1, 0.0,-0.0],
	[-0.1, 0,0, 0.0, 0.0],
	[ 0.0, 0.0,-0.1, 0.0],
	[ 0.0, 0.0, 0.0, 1.0]
    ])
    sol = ikmodel.manip.FindIKSolution(Tgoal, rave.IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    with jaco: # save robot state
        jaco.SetDOFValues(sol,ikmodel.manip.GetArmIndices()) # set the current solution
        Tee = ikmodel.manip.GetEndEffectorTransform()
        env.UpdatePublishedBodies() # allow viewer to update new robot
        raw_input('press any key')
    print('Tee is: '+repr(Tee))


"""

solver = rave.ikfast.IKFastSolver(kinbody=kinbody)

chaintree = solver.generateIkSolver(
	baselink=0,
	eelink=7,
	freeindices=[2],
	solvefn=rave.ikfast.IKFastSolver.solveFullIK_6D
)

code = solver.writeIkSolver(chaintree)
open('jacoik.cpp','w').write(code)


"""
