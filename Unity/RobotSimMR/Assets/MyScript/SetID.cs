using System;
using UnityEngine;

public class SetID : MonoBehaviour
{
    void Start()
    {
        string value = Environment.GetEnvironmentVariable("ROS_DOMAIN_ID");
        Debug.Log("current ROS_DOMAIN_ID:" + value + "\n");
        // ROS_DOMAIN_ID‚É123‚ğİ’è‚·‚é
        Environment.SetEnvironmentVariable("ROS_DOMAIN_ID", "30");
        // ãè‚­İ’è‚Å‚«‚Ä‚¢‚é‚©Šm”F‚·‚é
        value = Environment.GetEnvironmentVariable("ROS_DOMAIN_ID");
        Debug.Log("ROS_DOMAIN_ID:" + value + "\n");
    }
}
