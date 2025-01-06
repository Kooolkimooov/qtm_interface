from numpy import array, concatenate, cos, cross, diag, exp, eye, ndarray, sin, tan, zeros, pi
from numpy.linalg import inv
from geometry_msgs.msg import Pose, Wrench
from mavros_msgs.msg import OverrideRCIn
from scipy.spatial.transform import Rotation as R
from fake_qtm_interface_simulator.simulator import Simulator

G = 9.80665
rho_eau = 997.

class Bluerov(Simulator):
	"""
	Bluerov model, based on the BlueROV model from Blue Robotics
	parameters of the model are based on the BlueROV2 Heavy configuration
	and are stored in the class as class variables
	"""

	state_size = 12
	pose_size = 6
	actuation_size = 6
	linear_actuation_size = 3

	def __init__( self, water_surface_depth: float = 0., water_current: ndarray = None ):

		self.mass = 11.5
		self.center_of_mass = array( [ 0.0, 0.0, 0.0 ] )
		self.weight = array( [ 0., 0., self.mass * G ] )

		self.volume = 0.0134
		self.center_of_volume = array( [ 0.0, 0.0, -0.01 ] )
		self.buoyancy = -array( [ 0., 0., rho_eau * G * self.volume ] )

		self.water_surface_depth = water_surface_depth

		# water speed should be on [3:6]
		water_current = zeros( (6,) ) if water_current is None else water_current
		if water_current.shape == (3,):
			water_current = concatenate( (water_current, array( [ 0., 0., 0. ] )) )

		assert water_current.shape == (6,), 'water current should be a (6,) ndarray at this point'

		self.water_current = water_current

		self.inertial_coefficients = [ .26, .23, .37, 0., 0., 0. ]
		self.hydrodynamic_coefficients = [ 13.7, 0., 33.0, 0., 0.8, 0. ]
		self.added_mass_coefficients = [ 6.36, 7.12, 18.68, .189, .135, .222 ]

		self.inertial_matrix = self.build_inertial_matrix(
				self.mass, self.center_of_mass, self.inertial_coefficients
				) + diag( self.added_mass_coefficients )

		self.inverse_inertial_matrix = inv( self.inertial_matrix )

		self.hydrodynamic_matrix = diag( self.hydrodynamic_coefficients )

	def __call__( self, state: ndarray, actuation: ndarray, perturbation: ndarray = None ) -> ndarray:
		"""
		evaluates the dynamics of the Bluerov model
		:param state: current state of the system
		:param actuation: current actuation of the system
		:return: state derivative of the system
		"""

		transform_matrix = self.build_transformation_matrix( *state[ 3:6 ] )

		if perturbation is None:
			perturbation = zeros( (6,) )

		self.buoyancy[ 2 ] = rho_eau * G * self.volume * (-.5 - .5 / (1 + exp(
				10. * (self.water_surface_depth - state[ 2 ]) + 1.
				)))

		hydrostatic_forces = zeros( 6 )
		hydrostatic_forces[ :3 ] = transform_matrix[ :3, :3 ].T @ (self.weight + self.buoyancy)
		hydrostatic_forces[ 3: ] = cross(
				self.center_of_mass, transform_matrix[ :3, :3 ].T @ self.weight
				) + cross(
				self.center_of_volume, transform_matrix[ :3, :3 ].T @ self.buoyancy
				)

		xdot = zeros( state.shape )
		xdot[ :6 ] = transform_matrix @ state[ 6: ]
		xdot[ 6: ] = self.inverse_inertial_matrix @ (
				-self.hydrodynamic_matrix @ (state[ 6: ] - self.water_current) + hydrostatic_forces + actuation + perturbation)

		return xdot
	
	@staticmethod
	def initial_state() -> ndarray:
		state = zeros( (12,) )
		state[3] = pi
		return state 
	
	@staticmethod
	def rospose_from_state(state: ndarray) -> Pose:
		pose = Pose()
		pose.position.x = state[0]
		pose.position.y = state[1]
		pose.position.z = state[2]

		quaternion = R.from_euler('xyz', state[3:6]).as_quat()
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]

		return pose

	@staticmethod
	def actuation_from_ros(actuation) -> ndarray:
		actuation = zeros((Bluerov.actuation_size,))

		actuation[0] = actuation.channels[4]
		actuation[0] = actuation.channels[5]
		actuation[0] = actuation.channels[2]
		actuation[0] = actuation.channels[1]
		actuation[0] = actuation.channels[0]
		actuation[0] = actuation.channels[3]

		return actuation

	@staticmethod
	def build_transformation_matrix( phi: float, theta: float, psi: float ) -> ndarray:
		cPhi, sPhi = cos( phi ), sin( phi )
		cTheta, sTheta, tTheta = cos( theta ), sin( theta ), tan( theta )
		cPsi, sPsi = cos( psi ), sin( psi )

		matrix = eye( 6 )
		matrix[ 0, 0 ] = cPsi * cTheta
		matrix[ 0, 1 ] = -sPsi * cPhi + cPsi * sTheta * sPhi
		matrix[ 0, 2 ] = sPsi * sPhi + cPsi * sTheta * cPhi
		matrix[ 1, 0 ] = sPsi * cTheta
		matrix[ 1, 1 ] = cPsi * cPhi + sPsi * sTheta * sPhi
		matrix[ 1, 2 ] = -cPsi * sPhi + sPsi * sTheta * cPhi
		matrix[ 2, 0 ] = -sTheta
		matrix[ 2, 1 ] = cTheta * sPhi
		matrix[ 2, 2 ] = cTheta * cPhi
		matrix[ 3, 4 ] = sPhi * tTheta
		matrix[ 3, 5 ] = cPhi * tTheta
		matrix[ 4, 3 ] = 0
		matrix[ 4, 4 ] = cPhi
		matrix[ 4, 5 ] = -sPhi
		matrix[ 5, 3 ] = 0
		matrix[ 5, 4 ] = sPhi / cTheta
		matrix[ 5, 5 ] = cPhi / cTheta
		return matrix

	@staticmethod
	def build_inertial_matrix(
			mass: float, center_of_mass: ndarray, inertial_coefficients: list
			) -> ndarray:
		inertial_matrix = eye( 6 )
		for i in range( 3 ):
			inertial_matrix[ i, i ] = mass
			inertial_matrix[ i + 3, i + 3 ] = inertial_coefficients[ i ]
		inertial_matrix[ 0, 4 ] = mass * center_of_mass[ 2 ]
		inertial_matrix[ 0, 5 ] = - mass * center_of_mass[ 1 ]
		inertial_matrix[ 1, 3 ] = - mass * center_of_mass[ 2 ]
		inertial_matrix[ 1, 5 ] = mass * center_of_mass[ 0 ]
		inertial_matrix[ 2, 3 ] = mass * center_of_mass[ 1 ]
		inertial_matrix[ 2, 4 ] = - mass * center_of_mass[ 0 ]
		inertial_matrix[ 4, 0 ] = mass * center_of_mass[ 2 ]
		inertial_matrix[ 5, 0 ] = - mass * center_of_mass[ 1 ]
		inertial_matrix[ 3, 1 ] = - mass * center_of_mass[ 2 ]
		inertial_matrix[ 5, 1 ] = mass * center_of_mass[ 0 ]
		inertial_matrix[ 3, 2 ] = mass * center_of_mass[ 1 ]
		inertial_matrix[ 4, 2 ] = - mass * center_of_mass[ 0 ]
		inertial_matrix[ 3, 4 ] = - inertial_coefficients[ 3 ]
		inertial_matrix[ 3, 5 ] = - inertial_coefficients[ 4 ]
		inertial_matrix[ 4, 5 ] = - inertial_coefficients[ 5 ]
		inertial_matrix[ 4, 3 ] = - inertial_coefficients[ 3 ]
		inertial_matrix[ 5, 3 ] = - inertial_coefficients[ 4 ]
		inertial_matrix[ 5, 4 ] = - inertial_coefficients[ 5 ]

		return inertial_matrix


class BluerovXYZ( Bluerov ):

	actuation_size = 3
	linear_actuation_size = 3

	def __init__( self, water_surface_depth: float = 0., water_current: ndarray = None ):
		super().__init__( water_surface_depth, water_current )

	def __call__( self, state: ndarray, actuation: ndarray, perturbation = None ) -> ndarray:
		six_dof_actuation = zeros( (6,) )
		six_dof_actuation[ :3 ] = actuation

		return Bluerov.__call__( self, state, six_dof_actuation, perturbation )
	
	@staticmethod
	def build_transformation_matrix( phi: float, theta: float, psi: float ) -> ndarray:
		return eye( 6 )

class BluerovXYZPsi( Bluerov ):

	actuation_size = 4
	linear_actuation_size = 3

	def __init__( self, water_surface_depth: float = 0., water_current: ndarray = None ):
		super().__init__( water_surface_depth, water_current )

	def __call__( self, state: ndarray, actuation: ndarray, perturbation = None ) -> ndarray:
		six_dof_actuation = zeros( (6,) )
		six_dof_actuation[ :3 ] = actuation[ :3 ]
		six_dof_actuation[ 5 ] = actuation[ 3 ]

		return Bluerov.__call__( self, state, six_dof_actuation, perturbation )

	@staticmethod
	def build_transformation_matrix( phi: float, theta: float, psi: float ) -> ndarray:
		cPsi, sPsi = cos( psi ), sin( psi )

		matrix = eye( 6 )
		matrix[ 0, 0 ] = cPsi
		matrix[ 0, 1 ] = -sPsi
		matrix[ 1, 0 ] = sPsi
		matrix[ 1, 1 ] = cPsi

		return matrix

class BluerovXZPsi( Bluerov ):

	actuation_size = 3
	linear_actuation_size = 2

	def __init__( self, water_surface_depth: float = 0., water_current: ndarray = None ):
		super().__init__( water_surface_depth, water_current )

	def __call__( self, state: ndarray, actuation: ndarray, perturbation = None ) -> ndarray:
		six_dof_actuation = zeros( (6,) )
		six_dof_actuation[ :3:2 ] = actuation[ :2 ]
		six_dof_actuation[ 5 ] = actuation[ 2 ]

		return Bluerov.__call__( self, state, six_dof_actuation, perturbation )

	@staticmethod
	def build_transformation_matrix( phi: float, theta: float, psi: float ) -> ndarray:
		cPsi, sPsi = cos( psi ), sin( psi )

		matrix = eye( 6 )
		matrix[ 0, 0 ] = cPsi
		matrix[ 0, 1 ] = -sPsi
		matrix[ 1, 0 ] = sPsi
		matrix[ 1, 1 ] = cPsi

		return matrix

class USV( Bluerov ):

	actuation_size = 2
	linear_actuation_size = 1

	def __init__( self, water_surface_depth: float = 0. ):
		super().__init__( water_surface_depth )

	def __call__( self, state, actuation, perturbation = None ):
		six_dof_actuation = zeros( (6,) )
		# linear actuation on x
		six_dof_actuation[ 0 ] = actuation[ 0 ]
		# angular actuation around z
		six_dof_actuation[ 5 ] = actuation[ 1 ]

		return Bluerov.__call__( self, state, six_dof_actuation, perturbation )
	
	@staticmethod
	def build_transformation_matrix( phi: float, theta: float, psi: float ) -> ndarray:
		cPsi, sPsi = cos( psi ), sin( psi )

		matrix = eye( 6 )
		matrix[ 0, 0 ] = cPsi
		matrix[ 0, 1 ] = -sPsi
		matrix[ 1, 0 ] = sPsi
		matrix[ 1, 1 ] = cPsi

		return matrix

