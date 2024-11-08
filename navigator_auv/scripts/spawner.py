import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math

class MarkerSpawner:
    def __init__(self):
        rospy.init_node('marker_spawner', anonymous=True)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.last_pose = None
        self.distance_threshold = 0.75 # Distance threshold to spawn a new marker

        # Wait for the spawn model service to become available
        rospy.wait_for_service('/gazebo/spawn_sdf_model')

        # Subscribe to the robot's pose topic
        self.pose_sub = rospy.Subscriber('/rexrov2/pose_gt', Odometry, self.pose_callback)

    def pose_callback(self, pose_msg):
        # Extract robot's pose
        pose = pose_msg.pose.pose
        if self.last_pose:
            dist = self.calculate_distance(pose, self.last_pose)
            if dist >= self.distance_threshold:
                self.spawn_marker(pose)
                self.last_pose = pose
        else:
            self.spawn_marker(pose)
            self.last_pose = pose

    @staticmethod
    def calculate_distance(pose1, pose2):
        return math.sqrt((pose1.position.x - pose2.position.x) ** 2 +
                         (pose1.position.y - pose2.position.y) ** 2 +
                         (pose1.position.z - pose2.position.z) ** 2)

    def spawn_marker(self, pose):
        model_name = "visual_marker_{:.0f}".format(rospy.Time.now().to_sec())
        model_xml = """
        <sdf version='1.6'>
            <model name='{0}'>
                <static>true</static>
                <link name='link'>
                    <visual name='visual'>
                        <geometry>
                            <sphere>
                                <radius>0.25</radius> <!-- Adjust radius as needed -->
                            </sphere>
                        </geometry>
                        <material>
                            <ambient>0.5 1 0.5 1</ambient> <!-- Green color -->
                        </material>
                    </visual>
                </link>
                <pose>0 0 0 0 0 0</pose> <!-- Ensure pose values are correct -->
            </model>
        </sdf>
        """.format(model_name, pose.position.x, pose.position.y, pose.position.z)

        try:
            self.spawn_model(model_name, model_xml, "world", pose, "")
            # rospy.loginfo("Marker spawned at position: {:.2f}, {:.2f}, {:.2f}".format(
            #     pose.position.x, pose.position.y, pose.position.z))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    spawner = MarkerSpawner()
    rospy.spin()
