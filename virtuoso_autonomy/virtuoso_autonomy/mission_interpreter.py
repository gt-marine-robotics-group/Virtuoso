import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import Task

class MissionInterpreter(Node):

    def __init__(self):
        super().__init__('mission_interpreter')
        
        self.task_info_sub = self.create_subscription(Task, '/vrx/task/info', self.task_info_callback, 10)

        self.task = None
    
    def task_info_callback(self, msg:Task):

        # Task 1
        if (msg.name == 'station_keeping'): return self.station_keeping(msg)

        # Task 2
        if (msg.name == 'wayfinding'): return self.wayfinding(msg)

        #Task 3
        if (msg.name == 'perception'): return self.perception(msg)

        # Task 4
        if (msg.name == 'wildlife'): return self.wildlife(msg)

        # Task 5
        if (msg.name == 'gymkhana'): return self.gymkhana(msg)

        # Task 6
        if (msg.name == 'scan_dock_deliver'): return self.scan_dock_deliver(msg)
    
    def station_keeping(self, msg:Task):
        self.get_logger().info('STATION KEEPING')

    def wayfinding(self, msg:Task):
        self.get_logger().info('WAYFINDING')
    
    def perception(self, msg:Task):
        self.get_logger().info('PERCEPTION')

    def wildlife(self, msg:Task):
        self.get_logger().info('WILDLIFE')
    
    def gymkhana(self, msg:Task):
        self.get_logger().info('GYMKHANA')
    
    def scan_dock_deliver(self, msg:Task):
        self.get_logger().info('SCAN_DOCK_DELIVER')


def main(args=None):
    
    rclpy.init(args=args)

    node = MissionInterpreter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()