using System;
using UnityEngine;

public class SetID : MonoBehaviour
{
    void Start()
    {
        string value = Environment.GetEnvironmentVariable("ROS_DOMAIN_ID");
        Debug.Log("current ROS_DOMAIN_ID:" + value + "\n");
        // ROS_DOMAIN_ID��123��ݒ肷��
        Environment.SetEnvironmentVariable("ROS_DOMAIN_ID", "30");
        // ��肭�ݒ�ł��Ă��邩�m�F����
        value = Environment.GetEnvironmentVariable("ROS_DOMAIN_ID");
        Debug.Log("ROS_DOMAIN_ID:" + value + "\n");
    }
}
