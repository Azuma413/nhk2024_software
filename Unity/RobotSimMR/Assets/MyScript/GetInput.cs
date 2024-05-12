using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MyROS;

public class GetInput : MonoBehaviour
{
    public OVRInput.Controller controller;
    // Start is called before the first frame update
    void Start()
    {
    }

   // private void FixedUpdate()
   // {
    //    OVRInput.FixedUpdate();
    //}
    // Update is called once per frame
    void Update()
    {
        //OVRInput.Update();
        // PrimaryÇÕç∂ÅCSecondaryÇÕâE
        ROSControl.Instance.cmd_vel_data.Linear.X = OVRInput.Get(OVRInput.RawAxis2D.RThumbstick).y;
        ROSControl.Instance.cmd_vel_data.Linear.Y = OVRInput.Get(OVRInput.RawAxis2D.RThumbstick).x;
    }
}
