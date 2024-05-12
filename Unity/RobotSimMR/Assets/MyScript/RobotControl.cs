using MyROS;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using UnityEngine;

public class RobotControl : MonoBehaviour
{
    [SerializeField] private HingeJoint _omni1;
    [SerializeField] private HingeJoint _omni2;
    [SerializeField] private HingeJoint _omni3;
    [SerializeField] private HingeJoint _omni4;
    private JointMotor motor1;
    private JointMotor motor2;
    private JointMotor motor3;
    private JointMotor motor4;
    public GameObject base_robot;
    private Quaternion q_init;
    private geometry_msgs.msg.Quaternion imu_data;
    private float max_vel = 2000;
    private float max_rot_vel = 300;

    // Start is called before the first frame update
    void Start()
    {
        motor1 = _omni1.motor;
        motor2 = _omni2.motor;
        motor3 = _omni3.motor;
        motor4 = _omni4.motor;
        motor1.force = 100;
        motor2.force = 100;
        motor3.force = 100;
        motor4.force = 100;
        _omni1.useMotor = true;
        _omni2.useMotor = true;
        _omni3.useMotor = true;
        _omni4.useMotor = true;
        q_init = base_robot.transform.rotation;
        imu_data = new geometry_msgs.msg.Quaternion();
    }

    // Update is called once per frame
    private void FixedUpdate()
    {
        if (ROSControl.Instance.cmd_vel_data != null)
        {
            //print("cmd_vel_data: " + ROSControl.Instance.cmd_vel_data.Linear.X + ", " + ROSControl.Instance.cmd_vel_data.Linear.Y + ", " + ROSControl.Instance.cmd_vel_data.Linear.Z);
            double y = (ROSControl.Instance.cmd_vel_data.Linear.X + ROSControl.Instance.cmd_vel_data.Linear.Y)* 0.707f;
            double x = (ROSControl.Instance.cmd_vel_data.Linear.X - ROSControl.Instance.cmd_vel_data.Linear.Y) * 0.707f;
            motor1.targetVelocity = (int)(-x * max_vel - ROSControl.Instance.cmd_vel_data.Angular.Z*max_rot_vel);
            motor2.targetVelocity = (int)(x * max_vel - ROSControl.Instance.cmd_vel_data.Angular.Z*max_rot_vel);
            motor3.targetVelocity = (int)(y * max_vel - ROSControl.Instance.cmd_vel_data.Angular.Z*max_rot_vel);
            motor4.targetVelocity = (int)(-y * max_vel - ROSControl.Instance.cmd_vel_data.Angular.Z*max_rot_vel);
            _omni1.motor = motor1;
            _omni2.motor = motor2;
            _omni3.motor = motor3;
            _omni4.motor = motor4;
        }
        // スクリプトがアタッチされているオブジェクトのクオタニオンを取得
        //Quaternion q = base_robot.transform.rotation*Quaternion.Inverse(q_init);
        //if(q != null)
        //{
        //    imu_data.X = q.x;
        //    imu_data.Y = q.y;
        //    imu_data.Z = q.z;
        //    imu_data.W = q.w;
        //}
        //if (ROSControl.Instance.imu_data_pub != null)
        //{
        //    ROSControl.Instance.imu_data_pub.Publish(imu_data);
        //}
    }
}