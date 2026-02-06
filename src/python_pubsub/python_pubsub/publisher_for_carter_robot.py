import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'isaac_pub_topic', 10)
        
        
        self.subscription = self.create_subscription(
            Int32,
            'control_topic',
            self.control_callback,
            10)
        
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_value = 0
        
        

    def control_callback(self, msg):
        if msg.data in [0, 1]:

            self.current_value = msg.data
            


    def timer_callback(self):
        msg = String()
        msg.data = str(self.current_value)
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()