using UnityEngine;

public class MecanumDriveAgentSettings : MonoBehaviour
{
    [Header("Specific to Mecanum Drive Agent")]
    public float goalReward=50f;
    public float collisionReward=-10f;
    public float stepReward=-0.005f;

    // Motor Rotations/Minute
    public float rpm = 310.0f;

    // Wheel Diameters Millimeters    
    public float wheelDiameter = 96.0f;
    
    // Stall Torque Newton/Meter
    public float torque = 2.383f;

    // Use only 'velocityTarget' for xDrive in wheels' articulation bodies
    public bool useVelocity = false;

    // Use discrete actions for training
    public bool useDiscreteActions = true;

    // When using discrete actions, use this velocity ramping factor
    public float accelerationMultiplier = 1.128f;

    public float velocityIncrement = 0.01f;

    // In meters. Used to scale our distance detections from 0 to 1.
    public float maxMeasurableDistanceToGoal = 15f;

    // Frame size Millimeters
    Vector3 frameSize = new Vector3() {
        x = 432, 
        y = 360, 
        z = 48
    };
}   
