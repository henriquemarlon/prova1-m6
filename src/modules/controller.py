from math import atan2
from modules import Pose, Stack
from rclpy.node import Node
from geometry_msgs.msg import Twist

MAX_DIFF = 0.1

class Controller(Node):
        
    def __init__(self, queue, stack=Stack(), control_period=0.02):
        super().__init__('turtle_controller')
        
        self.pose = Pose(x=0.0)
        self.setpoint = Pose(x=0.0)
        self.stack = stack
        self.queue = queue
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )
        
        self.control_timer = self.create_timer(
                timer_period_sec=control_period,
                callback=self.control_callback
        )

    def control_callback(self):
        
        if self.pose.x == 0.0:
            self.get_logger().info("Aguardando a primeira posição...")
            return
        msg = Twist()
        x_diff = self.setpoint.x - self.pose.x
        y_diff = self.setpoint.y - self.pose.y
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.update_setpoint()
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 0.5 if y_diff > 0 else -0.5
        else:
            msg.linear.y = 0.0
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.5 if x_diff > 0 else -0.5
        else:
            msg.linear.x = 0.0
        self.publisher.publish(msg)
        
    def update_setpoint(self):
        try:
            dequeue = self.queue.dequeue()
            self.setpoint = self.pose + dequeue
            self.stack.stackup(dequeue)
            self.get_logger().info(f"A sua tartaruga chegou em {self.pose}")
            
        except IndexError:
            try:
                self.setpoint = self.pose - self.stack.unstack()
            except IndexError:
                self.get_logger().info(f"Mbpappé chegou em {self.pose}, \
                                   andando para {self.setpoint}")
                exit()
            

    def pose_callback(self, msg):

        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == 0.0:
            self.update_setpoint()