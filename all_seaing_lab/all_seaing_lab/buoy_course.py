import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Point

class BuoyCourse(Node):
    def __init__(self):
        super().__init__('buoy_course')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('red_buoys_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('red_buoys_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('green_buoys_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('green_buoys_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ]
        )

        self.global_frame_id = "map"
        self.timer = self.create_timer(1, self.timer_callback)
        self.red_publisher = self.create_publisher(Marker, 'red_buoys', 10)
        self.green_publisher = self.create_publisher(Marker, 'green_buoys', 10)

    def timer_callback(self):
        red_buoys_x = self.get_parameter('red_buoys_x').value
        red_buoys_y = self.get_parameter('red_buoys_y').value
        green_buoys_x = self.get_parameter('green_buoys_x').value
        green_buoys_y = self.get_parameter('green_buoys_y').value

        red_marker = self._buoys_to_marker(red_buoys_x, red_buoys_y, "red")
        green_marker = self._buoys_to_marker(green_buoys_x, green_buoys_y, "green")

        self.red_publisher.publish(red_marker)
        self.green_publisher.publish(green_marker)



    def _buoys_to_marker(self, buoys_x, buoys_y, color):
        """
        Helper function to convert buoys to marker of type sphere list.
        @param buoys_x x coordinates of buoys
        @param buoys_y y coordinates of buoys
        @param color the color of the marker
        """
        marker = Marker()
        marker.header = Header(frame_id=self.global_frame_id)
        marker.type = Marker.SPHERE_LIST
        marker.scale = Vector3(x=0.3, y=0.3, z=0.2)
        marker.color.a = 1.0
        if color == "red":
            marker.color.r = 1.0
        elif color == "green":
            marker.color.g = 1.0
        else:
            marker.color.b = 1.0
        for buoy_x, buoy_y in zip(buoys_x, buoys_y):
            marker.points.append(Point(x=buoy_x, y=buoy_y))
        return marker


        

def main():
    rclpy.init()
    node = BuoyCourse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()