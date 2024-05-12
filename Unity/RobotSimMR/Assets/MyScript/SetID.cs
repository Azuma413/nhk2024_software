using System;
using UnityEngine;

public class SetID : MonoBehaviour
{
    void Start()
    {
        string value = Environment.GetEnvironmentVariable("ROS_DOMAIN_ID");
        Debug.Log("current ROS_DOMAIN_ID:" + value + "\n");
        // ROS_DOMAIN_IDに123を設定する
        Environment.SetEnvironmentVariable("ROS_DOMAIN_ID", "30");
        // 上手く設定できているか確認する
        value = Environment.GetEnvironmentVariable("ROS_DOMAIN_ID");
        Debug.Log("ROS_DOMAIN_ID:" + value + "\n");
    }
}
