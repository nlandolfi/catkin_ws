# import required libraries
import openravepy
import numpy as np

try:
	env = openravepy.Environment()
	env.SetViewer('qtcoin')
	#env.SetViewer('RViz')
	#env.SetViewer('InteractiveMarker')

	# Importing the Robot
	module = openravepy.RaveCreateModule(env, 'urdf')
	print(module)
	name = module.SendCommand('load /home/ubuntu/catkin_ws/src/j2s7s300_standalone.urdf /home/ubuntu/catkin_ws/src/jaco7dof_standalonev1.srdf')
	#name = module.SendCommand('load /home/ubuntu/catkin_ws/src/ada_description/robots/mico.urdf /home/ubuntu/catkin_ws/src/ada_description/robots/mico.srdf')
	jaco = env.GetRobot(name)


	# Importing objects
	table = env.ReadKinBodyXMLFile('objects/table.kinbody.xml')

	env.Add(table)

	bottle = env.ReadKinBodyXMLFile('objects/fuze_bottle.kinbody.xml')
	env.Add(bottle)
	mug = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
	env.Add(mug)


	# Arm and objects Placement
	Tz =openravepy.matrixFromAxisAngle([-np.pi/2,0,0])
	pos = np.array([[1,0,0,0],[0,1,0,0.70],[0,0,1,0.37],[0,0,0,1]])
	pos = np.dot(pos,Tz)
	jaco.SetTransform(pos)

	bottle_pos = np.array([[1,0,0,0.2],[0,1,0,0.73],[0,0,1,-0.1],[0,0,0,1]])
	bottle_pos = np.dot(bottle_pos,Tz)
	bottle.SetTransform(bottle_pos)

	mug_pos = np.array([[1,0,0,0.4],[0,1,0,0.73],[0,0,1,0.1],[0,0,0,1]])
	mug.SetTransform(mug_pos)

	#manip = jaco.GetManipulators()[0]



	jaco.SetDOFValues(np.array([ -4.13872047e-01 , -5.01264461e-01 ,  4.02553793e-01 , -3.22636312e-01, -6.96080833e-01   ,8.54815566e-01  , 9.35005132e-02 ,  1.14352972e-14,
	   1.99840144e-15 ,  3.33066907e-16]
	))

	raw_input('press any key to finish ... ')

	#example getting the transform of the end effector
	#this is a matrix containing orientation and position
	#eff = jaco.GetManipulators()[0]
	#eff_trans =  eff.GetEndEffectorTransform()

	#get forward kinematics at some other link
	#fk = jaco.CalculateActiveJacobian(3,jaco.GetLinks()[3].GetTransform()[0:3,3])

	#get current robot dofs
	#q=jaco.GetActiveDOFValues()

	#example changing robot DOFs
	#q[3]=q[3]+.4
	#jaco.SetActiveDOFValues(q)

	# documentation at http://openrave.org/docs/latest_stable/coreapihtml/

except Exception, e:
    print e


