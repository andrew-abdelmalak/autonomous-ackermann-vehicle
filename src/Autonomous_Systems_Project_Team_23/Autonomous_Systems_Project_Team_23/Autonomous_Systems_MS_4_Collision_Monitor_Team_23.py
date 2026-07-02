"""Gazebo contact monitor for the Team 23 simulation vehicle."""

from ros_gz_interfaces.msg import Contacts
import rclpy
from rclpy.node import Node


class AutonomousSystemsMS4CollisionMonitorTeam23(Node):
    """Report wall / obstacle touches while ignoring wheel-ground contact."""

    def __init__(self):
        super().__init__('autonomous_systems_ms_4_collision_monitor_team_23')

        default_topics = [
            '/vehicle/chassis_contacts',
            '/vehicle/front_left_wheel_contacts',
            '/vehicle/front_right_wheel_contacts',
            '/vehicle/rear_left_wheel_contacts',
            '/vehicle/rear_right_wheel_contacts',
        ]
        self.declare_parameter('contact_topics', default_topics)
        self.declare_parameter('ignore_name_tokens', ['ground_plane', 'vehicle::'])
        self.declare_parameter('latch_first_collision', True)

        self.contact_topics = list(self.get_parameter('contact_topics').value)
        self.ignore_name_tokens = [
            token.lower() for token in self.get_parameter('ignore_name_tokens').value
        ]
        self.latch_first_collision = bool(
            self.get_parameter('latch_first_collision').value
        )
        self.first_collision_reported = False

        for topic in self.contact_topics:
            self.create_subscription(
                Contacts,
                topic,
                lambda msg, monitored_topic=topic: self.contacts_callback(
                    monitored_topic,
                    msg,
                ),
                10,
            )

        self.get_logger().info(
            'MS4 collision monitor ready for %d contact topics.'
            % len(self.contact_topics)
        )

    def contacts_callback(self, topic_name, message):
        """Log any non-ground contact involving the simulated vehicle."""
        for contact in message.contacts:
            name_1 = str(contact.collision1.name)
            name_2 = str(contact.collision2.name)
            lower_1 = name_1.lower()
            lower_2 = name_2.lower()

            if any(token in lower_1 and token in lower_2 for token in self.ignore_name_tokens):
                continue
            if 'ground_plane' in lower_1 or 'ground_plane' in lower_2:
                continue
            if name_1 == name_2:
                continue

            if self.latch_first_collision and self.first_collision_reported:
                return

            self.first_collision_reported = True
            self.get_logger().error(
                'COLLISION DETECTED on %s between %s and %s'
                % (topic_name, name_1, name_2)
            )
            return


def main(args=None):
    """Entry point for the Gazebo collision monitor."""
    rclpy.init(args=args)
    node = AutonomousSystemsMS4CollisionMonitorTeam23()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
