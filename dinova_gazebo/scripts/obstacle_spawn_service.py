""""
Spawn obstacles in gazebo and publish them to /dinova/objects, such that they can be perceived.
"""

#!/usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from derived_object_msgs.msg import ObjectArray, Object
from dinova_gazebo.srv import SpawnObstacles, SpawnObstaclesResponse

class ObstacleSpawner:

    def __init__(self):
        rospy.init_node("spawn_obstacles_service")

        self.pub_objects = rospy.Publisher(
            "/dinova/objects",
            ObjectArray,
            queue_size=1,
            latch=True
        )

        rospack = rospkg.RosPack()
        sdf_path = rospack.get_path("dinova_gazebo") + "/urdf/simple_sphere_template.sdf"
        with open(sdf_path, "r") as f:
            self.sdf_template = f.read()

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        self.service = rospy.Service("/dinova_gazebo/spawn_obstacles", SpawnObstacles, self.handle_spawn)

        self.spawned_obstacles = []  # list to keep track of the spawned obstacles, such that they can be deleted before spawning new ones

        rospy.loginfo("SpawnObstacles service ready.")
        rospy.spin()

    def clear_previous_obstacles(self):
        """Delete previously spawned gazebo models."""
        for name in self.spawned_obstacles:
            try:
                self.delete_srv(name)
                rospy.loginfo(f"Deleted old obstacle: {name}")
            except Exception as e:
                rospy.logwarn(f"Could not delete {name}: {e}")

        self.spawned_obstacles = []


    def handle_spawn(self, req):
        self.clear_previous_obstacles()  # delete all previously spawned obstacles

        n = len(req.x)
        if not (len(req.y) == len(req.z) == len(req.radius) == n):
            return SpawnObstaclesResponse(
                success=False,
                message="Input arrays must have equal length."
            )

        msg = ObjectArray()
        msg.header.stamp = rospy.Time.now()

        for i in range(n):
            name = "obst_" + str(i)
            r = req.radius[i]

            # Fill SDF string
            sdf_filled = self.sdf_template.replace("{name}", name)
            sdf_filled = sdf_filled.replace("{radius}", str(r))

            pose = Pose()
            pose.position.x = req.x[i]
            pose.position.y = req.y[i]
            pose.position.z = req.z[i]

            try:
                self.spawn_srv(
                    model_name=name,
                    model_xml=sdf_filled,
                    robot_namespace="/",
                    initial_pose=pose,
                    reference_frame="world"
                )
                self.spawned_obstacles.append(name)

            except Exception as e:
                return SpawnObstaclesResponse(
                    success=False,
                    message=f"Failed to spawn {name}: {e}"
                )

            # Fill Object message
            obj = Object()
            obj.pose.position = pose.position
            obj.shape.dimensions = [r]   # radius
            msg.objects.append(obj)

        self.pub_objects.publish(msg)  # publish once (latch=True ensures future subscribers receive it)

        return SpawnObstaclesResponse(
            success=True,
            message=f"Spawned {n} obstacles successfully."
        )


if __name__ == "__main__":
    ObstacleSpawner()
