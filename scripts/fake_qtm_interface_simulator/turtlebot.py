from fake_qtm_interface_simulator.simulator import Simulator
from geometry_msgs.msg import Pose, Twist
from numpy import array, cos, ndarray, sin, zeros
from scipy.spatial.transform import Rotation as R


class Turtlebot( Simulator ):
  """
  Turtlebot model
  """

  state_size = 3
  pose_size = 2
  actuation_size = 2
  linear_actuation_size = 1

  def __init__( self ):
    pass

  def __call__( self, state: ndarray, actuation: ndarray, perturbation: ndarray = None ) -> ndarray:
    """
    evaluates the dynamics of the Turtlebot model
    :param state: current state of the system
    :param actuation: current actuation of the system
    :return: state derivative of the system
    """

    xdot = zeros( (3,) )
    xdot[ 0 ] = cos( state[ 2 ] ) * actuation[ 0 ]
    xdot[ 1 ] = sin( state[ 2 ] ) * actuation[ 0 ]
    xdot[ 2 ] = actuation[ 1 ]

    return xdot

  @staticmethod
  def initial_state() -> ndarray:
    return zeros( (3,) )

  @staticmethod
  def rospose_from_state( state: ndarray ) -> Pose:
    pose = Pose()
    pose.position.x = state[ 0 ]
    pose.position.y = state[ 1 ]

    quaternion = R.from_euler( 'xyz', array( [ 0, 0, state[ 2 ] ] ) ).as_quat()
    pose.orientation.x = quaternion[ 0 ]
    pose.orientation.y = quaternion[ 1 ]
    pose.orientation.z = quaternion[ 2 ]
    pose.orientation.w = quaternion[ 3 ]

    return pose

  @staticmethod
  def actuation_from_ros( actuation ) -> ndarray:
    actuation = zeros( (Turtlebot.actuation_size,) )

    actuation[ 0 ] = actuation.linear.x
    actuation[ 1 ] = actuation.angular.z

    return actuation

  @staticmethod
  def get_command_type():
    return Twist
