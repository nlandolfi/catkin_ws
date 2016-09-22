
import openravepy

env = openravepy.Environment()
env.SetViewer('qtcoin')
#env.SetViewer('RViz')
#env.SetViewer('InteractiveMarker')

module = openravepy.RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/ada_description/robots/mico.urdf /home/viki/catkin_ws/src/ada_description/robots/mico.srdf')
mico = env.GetRobot(name)


#get current robot dofs
q=mico.GetActiveDOFValues()

#example changing robot DOFs
q[3]=q[3]+.4
mico.SetActiveDOFValues(q)

#example getting the transform of the end effector
#this is a matrix containing orientation and position
eff = mico.GetManipulators()[0]
eff_trans =  eff.GetEndEffectorTransform()

#get forward kinematics at some other link
fk = mico.CalculateActiveJacobian(3,mico.GetLinks()[3].GetTransform()[0:3,3])

# documentation at http://openrave.org/docs/latest_stable/coreapihtml/


