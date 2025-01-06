from numpy import ndarray
from geometry_msgs.msg import Pose

class Simulator:
    def __call__(self, *args, **kwargs):
        raise NotImplementedError
    
    @staticmethod
    def initial_state() -> ndarray:
        raise NotImplementedError
    
    @staticmethod
    def rospose_from_state(state: ndarray) -> Pose:
        raise NotImplementedError
    
    @staticmethod
    def actuation_from_ros(actuation) -> any:
        raise NotImplementedError
    
    
