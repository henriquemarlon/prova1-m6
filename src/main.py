import rclpy
from modules import Controller, Queue


def main(args=None):
    rclpy.init(args=args)
    qe = Queue()
    tc = Controller(qe)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()