using UnityEngine;
 
public class WheelFrameJoint : MonoBehaviour {
 
    public GameObject RobotFrame;
    public GameObject Axel;
    public Rigidbody Frame = null;
    public HingeJoint RobotFrameWheelJoint;

    // public Vector3 anchor;
    // public Vector3 axis;
    // public Vector3 connectedAxel;
    
    void Start() {
        RobotFrameWheelJoint.axis = new Vector3(0.0f, 1.0f, 0.0f);
        RobotFrameWheelJoint.autoConfigureConnectedAnchor = false;
        RobotFrameWheelJoint.connectedAnchor = Axel.transform.position;
        // RobotFrameWheelJoint.gameObject.transform.localPosition;
        RobotFrameWheelJoint.connectedBody = Frame;
        var motor = RobotFrameWheelJoint.motor;
        motor.force = 100;
        motor.targetVelocity = 90;
        motor.freeSpin = true;
        RobotFrameWheelJoint.motor = motor;
        RobotFrameWheelJoint.useMotor = true;
    }

    void Update() {
        
    }
}