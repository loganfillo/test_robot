import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point, Wrench, Vector3

class GazeboController(Node):


    def __init__(self):
        super().__init__('controller')
        self.point_sub = self.create_subscription(
            Point,
            'input',
            self.point_callback,
            10
        )
        self.has_reached_point = False

        self.gazebo_force_pub = self.create_publisher(Wrench, 'output',10)
        self.ref_point = np.array([50,0,0]) #cm
        self.ref_error = np.array([3,3,3])
        self.prev_err = None
        self.Kp = np.array([0.7,0.7,0.7])
        self.Kd = np.array([40,40,40])


    def point_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        curr_point = np.array([x,y,z]) #cm
        p_err = (curr_point - self.ref_point)*(1/np.power(10, 2)) #m
        if self.prev_err is None:
            self.prev_err = p_err
        d_err = (p_err - self.prev_err) #m
        if (np.greater(p_err, (self.ref_point - self.ref_error)).all() and
            np.less(p_err, (self.ref_point + self.ref_error)).all() and 
            not has_reached_point
        ):
            self.ref_point = curr_point
            self.has_reached_point = True
        else:
            force = self.Kp*p_err + self.Kd*d_err
            force_vector = Vector3()
            force_vector.x = force[0]
            force_vector.y = force[1]
            force_vector.z = force[2]
            wrench_msg = Wrench()
            wrench_msg.force = force_vector
            self.gazebo_force_pub.publish(wrench_msg)
        self.prev_err = p_err

    
def main(args=None):
    rclpy.init(args=args)
    gazebo_controller = GazeboController()
    rclpy.spin(gazebo_controller)
    gazebo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
