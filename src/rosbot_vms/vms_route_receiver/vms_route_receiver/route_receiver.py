import paho.mqtt.client as mqtt
import rclpy
from rclpy.action import ActionClient
from vms_msgs.action import TranslateRouteToPath  # Ensure this is the correct path for your custom message
from vms_msgs.msg import Route, RoutePoint
from rclpy.node import Node
import json

class TranslateRouteClient(Node):
    def __init__(self):
        super().__init__('translate_route_action_client')
        self._action_client = ActionClient(self, TranslateRouteToPath, '/translate_route_to_path')
        self.get_logger().info("TranslateRouteClient initialized.")

    def send_goal(self, route_message):
        goal_msg = TranslateRouteToPath.Goal()

        initial_latitude = 53.745620171847804
        initial_longitude = -2.8941631520855164
        scale_factor = 10000
        
        # Parse the route from the received MQTT message
        goal_msg.route.routepoints = [
            RoutePoint(
                latitude=(point['latitude'] - initial_latitude) * scale_factor * -1,
                longitude=(point['longitude'] - initial_longitude) * scale_factor,
                altitude=point['altitude'],
                satisfies_requirement_id=''
            ) for point in route_message['routepoint']
        ]
        goal_msg.translator_id = "GridBased"
        
        self.get_logger().info("Sending goal to action server.")
        self.get_logger().info(f"Relative routepoint: {goal_msg}")
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

# MQTT message callback
def on_message(client, userdata, msg, logger):
    try:
        route_message = json.loads(msg.payload)
        translate_route_client = TranslateRouteClient()
        logger.info(f"Received MQTT message: {route_message}" )
        
        # Check for the nested keys correctly
        if 'routepoint' in route_message:
            translate_route_client.send_goal(route_message)
        else:
            logger.info("Invalid message format. 'routepoint' key is missing.")
    except json.JSONDecodeError:
        logger.info("Error decoding JSON message.")

# MQTT on_connect callback
def on_connect(client, userdata, flags, rc, logger):
    if rc == 0:
        logger.info("Connected successfully to MQTT broker")
        client.subscribe("route")  # Change topic as needed
    else:
        logger.info(f"Connection failed with code {rc}")

# Main function to connect to MQTT and start listening
def main():
    rclpy.init(args=None)
    logger = rclpy.logging.get_logger('mqtt_client')

    # Initialize MQTT client
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = lambda client, userdata, flags, rc: on_connect(client, userdata, flags, rc, logger)
    mqtt_client.on_message = lambda client, userdata, msg: on_message(client, userdata, msg, logger)

    # Connect to the MQTT broker
    mqtt_client.connect("192.168.0.100", 1883, 600)  # Change broker address if necessary
    mqtt_client.loop_forever()

if __name__ == "__main__":
    main()
