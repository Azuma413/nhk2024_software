using ROS2;
using UnityEngine;

namespace MyROS
{
    public class ROSControl : MonoBehaviour
    {
        internal static readonly ROSConfig Instance = new ROSConfig();
        // Start is called before the first frame update
        void Start()
        {
            if(TryGetComponent(out Instance.ros2Unity))
            {
                print("ROS2UnityComponent found");
            }
            else
            {
                print("ROS2UnityComponent not found");
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (Instance.ros2Unity.Ok())
            {
                if (Instance.ros2Node == null)
                {
                    Instance.cmd_vel_data = new geometry_msgs.msg.Twist();
                    Instance.ros2Node = Instance.ros2Unity.CreateNode("unity_node");
                    Instance.cmd_vel_sub = Instance.ros2Node.CreateSubscription<geometry_msgs.msg.Twist>("cmd_vel", Instance.cmd_vel_cb);
                    Instance.imu_data_pub = Instance.ros2Node.CreatePublisher<geometry_msgs.msg.Quaternion>("mros_output_imu");
                }
            }
        }
    }
    class ROSConfig
    {
        public ROS2UnityComponent ros2Unity;
        public ROS2Node ros2Node;
        public IPublisher<geometry_msgs.msg.Quaternion> imu_data_pub;
        public ISubscription<geometry_msgs.msg.Twist> cmd_vel_sub;
        public geometry_msgs.msg.Twist cmd_vel_data;
        public void cmd_vel_cb(geometry_msgs.msg.Twist msg)
        {
            cmd_vel_data = msg;
        }
    }
}
