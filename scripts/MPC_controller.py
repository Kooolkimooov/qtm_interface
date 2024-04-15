#!/usr/bin/python3
#=================================== Import =============================================
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
from scipy.integrate import odeint
from threading import Thread
from math import isnan, sin, cos
from copy import deepcopy
from scipy.optimize import Bounds, LinearConstraint, minimize, NonlinearConstraint

#Topic d'Acquisition
topic_name = "qtm/turtle/6dof_pose"


def turtle(
		x: np.ndarray, u: np.array
		) -> np.ndarray:

	xdot = np.zeros( x.shape )
	_, _, theta, _, _, _ = x
	v, w = u
	xdot[ 0 ] = v * cos( theta ) / 160
	xdot[ 1 ] = v * sin( theta ) / 160
	xdot[ 2 ] = w / 0.89

	return xdot

def turtle_oblective(
		x: np.ndarray, u: np.array
		) -> np.ndarray:
	temp = x[:2]
	return 10 * np.linalg.norm(temp)


# cost function for one horizon pass with given model and actuations
def cost(
		actuations: np.ndarray,
		current_actuation: np.ndarray,
		command_shape: tuple,
		model: callable,
		model_args: dict,
		target: np.ndarray,
		horizon: int,
		state: np.ndarray,
		time_step: float,
		objective: callable,
		final_cost_weight: float,
		activate_euclidean_cost: bool,
		activate_final_cost: bool,
		state_history: list,
		actuation_history: list,
		verb: bool
		) -> float:

	local_actuation = deepcopy( current_actuation )
	actuations = actuations.reshape( command_shape )

	if not activate_euclidean_cost and not activate_final_cost and objective is None:
		raise ValueError( "Cannot compute cost" )

	cost = 0.

	if actuation_history is not None:
		to_append = actuations
		actuation_history.append( to_append )

	states = np.zeros( (horizon, len( state )) )

	for i in range( horizon ):
		states[ i ] = state
		local_actuation += actuations[ i ]
		state = state + model(
				state, local_actuation, **model_args
				) * time_step
		if activate_euclidean_cost:
			cost += np.linalg.norm( state[ : 3 ] - target ) ** 2
		if objective is not None:
			cost += objective( state, local_actuation, **model_args )

	cost /= horizon

	if activate_final_cost:
		cost += final_cost_weight * np.linalg.norm( state[ : 3 ] - target ) ** 2
		if objective is not None:
			cost += objective( state, local_actuation, **model_args )

	if state_history is not None:
		state_history.append( states )

	if verb:
		print( f"cost: {cost}; u: {actuations}" )
	return cost


# model predictive control returns the optimal actuation
def model_predictive_control(
		cost: callable,
		model: callable,
		state: np.ndarray,
		target: np.ndarray,
		last_result: np.ndarray,
		current_actuation: np.ndarray,
		time_step: float,
		horizon: int,
		tolerance: float = 1e-6,
		max_iter: int = 100,
		model_args: dict = None,
		bounds: Bounds = None,
		constraints = None,
		objective: callable = None,
		activate_euclidean_cost: bool = True,
		activate_final_cost: bool = True,
		final_cost_weight: float = 1.0,
		state_history: list = None,
		actuation_history: list = None,
		verb: bool = False
		) -> np.ndarray:

	assert final_cost_weight > 0

	command_shape = last_result.shape
	initial_guess = np.concatenate(
			(last_result[ 1: ], [ last_result[ -1 ] ])
			).flatten()

	result = minimize(
			fun = cost,
			x0 = initial_guess,
			args = (current_actuation, command_shape, model, model_args, target, horizon,
							state, time_step, objective, final_cost_weight, activate_euclidean_cost,
							activate_final_cost, state_history, actuation_history, verb),
			tol = tolerance,
			bounds = bounds,
			constraints = constraints,
			options = { 'maxiter': max_iter }
			)

	if verb:
		print( result )
	return result.x.reshape( command_shape )



#================================= Déclarations =========================================

############################################ Peut être modifié ######################################################
#, [0.8, 0.8], [0.01, 0.01], [-0.5, -0.2], [0.8, -0.8], [-0.4, -0.4]]

class Controller:

    def __init__(self) -> None:
        self.waypoints = np.array([[0, 0, 0], [1, 1, 0], [-1, -0.7, 0], [1, -0.7, 0], [0, 0, 0], [-2, 0, 0]])

        self.horizon = 5
        self.mpc_result = np.zeros((self.horizon, 2))
		
        self.max_command_ramp = 16
        self.max_command = 50

        self.pose = Pose()
        self.pose.position.x = 0.
        self.pose.position.y = 0.
        self.pose.position.z = 0.
        self.pose.orientation.x = 0.
        self.pose.orientation.y = 0.
        self.pose.orientation.z = 0.
        self.command = Twist()
        self.command.linear.x = 0.
        self.command.linear.y = 0.
        self.command.linear.z = 0.
        self.command.angular.x = 0.
        self.command.angular.y = 0.
        self.command.angular.z = 0.
		
        self.target = np.array([0, 0, 0])
		
        rospy.init_node('MPC')
        rospy.Subscriber(topic_name, Pose, self.acquistion)
        self.command_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=2)
        self.rate = rospy.Rate(100)  # 100 Hz

        rospy.loginfo("MPC: init finished")

    def acquistion(self,pose):
        if (isnan((pose.position.x) or isnan(pose.position.y) or isnan(pose.orientation.z))):
            rospy.logwarn('Robot not in working space')
            self.command.linear.x = 0.
            self.command.linear.y = 0.
            self.command.linear.z = 0.
            self.command.angular.x = 0.
            self.command.angular.y = 0.
            self.command.angular.z = 0.
            self.command_publisher.publish(self.command)
        
        else:
            self.pose.position.x = pose.position.x / 1000
            self.pose.position.y = pose.position.y / 1000
            self.pose.orientation.z = pose.orientation.z * np.pi / 180
            self.compute_command()
            self.command_publisher.publish(self.command)

    def compute_command(self):
        state = np.array([self.pose.position.x, self.pose.position.y, self.pose.orientation.z, 0, 0, 0])
        command = np.array([self.command.linear.x, self.command.angular.z])
        self.mpc_result = model_predictive_control(
            cost=cost,
			objective=turtle_oblective,            
			model=turtle,
            state=state,
            target=self.target,
            last_result=self.mpc_result,
            current_actuation=command,
            time_step=0.01,
            horizon=self.horizon,
			bounds=Bounds(-self.max_command_ramp, self.max_command_ramp),
			constraints=NonlinearConstraint(
				lambda u: (command + np.cumsum(u.reshape(self.horizon, 2), axis=0)).flatten(),-self.max_command, self.max_command
				),
            tolerance=0.002,
			model_args={},
			final_cost_weight=10,
			#verb=True
        )
         
        self.command.linear.x += self.mpc_result[0][0] / 160
        self.command.angular.z += self.mpc_result[0][1] / 0.89
        self.command.linear.x = min(self.max_command / 160, self.command.linear.x) 
        self.command.angular.z = min(self.max_command / 160, self.command.angular.z) 

def main():
    controller = Controller()
    while not rospy.is_shutdown():
            controller.rate.sleep()


if __name__=="__main__":
    main()