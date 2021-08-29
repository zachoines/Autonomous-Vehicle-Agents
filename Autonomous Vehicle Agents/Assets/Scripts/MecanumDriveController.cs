using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/*
    MecanumDriveController represents a simulation of a 4-wheeled omni-direction mecanum robot. 
    Movement is based on encoder ticks. Internally, speed and acceleration is measured 
    in quadrate of encoders. Distance traveled is inferred by wheel Diameter.
*/
public enum RotationDirection { None=0, Positive=1, Negative = -1 };

public class MecanumDriveController : MonoBehaviour
{
    // PPR
    [SerializeField] 
    private float encoderResolution = 384.5f;

    // MM
    [SerializeField] 
    private float wheelDiameter = 96.0f;

    // RPM
    [SerializeField]
    private float rpm = 310.0f;

    // Stall Torque (kg * cm) / s
    [SerializeField]
    private float torque = 24.3f;
    
    [SerializeField] 
    private ArticulationBody TL = null;
    [SerializeField] 
    private ArticulationBody TR = null;
    [SerializeField] 
    private ArticulationBody BL = null;
    [SerializeField] 
    private ArticulationBody BR = null;
    
    

    [SerializeField]
    private float thrust = 10;
    private bool _rotating;

    private float max_rads_per_second = 0;
    private float max_degs_per_second = 0;

    PlayerControls playerController;

    private void Awake() {
        playerController = new PlayerControls();
        playerController.Robot.Enable();    
        playerController.Robot.Right.Enable();
        playerController.Robot.Left.Enable();
        playerController.Robot.Switch.Enable();
    }

    void Start()
    {
        max_rads_per_second = (rpm / 60.0f) * (2f * (Mathf.PI));
        max_degs_per_second = rpm * 6f;
    }

    private float angleToEncoderTicks(float angle) {
        float ratio = encoderResolution / 360.0f; 
        float ticks = angle * ratio;
        return ticks;
    }

    /*
        @desc Calculate quadrate pulses per second
        @param np Number of pulses
        @param dt Delta time 
    */
    float getSpeed(float np, float dt) {
        return (2f * Mathf.PI * np) / (encoderResolution * dt);
    } 

    /*
        @desc Calculate quadrate pulses given angular velocity
        @param dt Delta time 
    */
    float angularVelocityToEncoderTicks(float v, float dt) {
        return (v * encoderResolution * dt) / (2f * Mathf.PI);
    }

    // angular acceleration (radians/s^2) = Torque / (0.5 * mass * radius^2)
    // angular velocity (deg/s) = angular acceleration * time =
    
    void FixedUpdate()
    {
        var swithed = playerController.Robot.Switch.ReadValue<float>();
        Vector2 r = playerController.Robot.Right.ReadValue<Vector2>();
        Vector2 l = playerController.Robot.Left.ReadValue<Vector2>();
        
        
        if (swithed <= 0.0) { 
            setWheel(TR, Mathf.Clamp(r.y, -1.0f, 1.0f));
            setWheel(BL, Mathf.Clamp(r.y, -1.0f, 1.0f));
            setWheel(TL, Mathf.Clamp(l.y, -1.0f, 1.0f));
            setWheel(BR, Mathf.Clamp(l.y, -1.0f, 1.0f));
            
        } else {
            setWheel(TR, Mathf.Clamp(r.y, -1.0f, 1.0f));
            setWheel(BR, Mathf.Clamp(r.y, -1.0f, 1.0f));
            setWheel(TL, Mathf.Clamp(l.y, -1.0f, 1.0f));
            setWheel(BL, Mathf.Clamp(l.y, -1.0f, 1.0f));
        }
    }

    private float GetAxisRotation(ArticulationBody articulation)
    {
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    private void setDrive(float primaryAxisRotation, ArticulationBody articulation)
    {
        ArticulationDrive drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        drive.forceLimit = torque;
        articulation.xDrive = drive;
    }

    private void setVelocity(float v, ArticulationBody articulation) {
        ArticulationDrive drive = articulation.xDrive;
        drive.targetVelocity = v;
        drive.forceLimit = torque;
        articulation.xDrive = drive;
    }

    void setWheel(ArticulationBody wheel, float speed) {
        float degrees_sec = (speed * max_degs_per_second);
        float rotationGoal = GetAxisRotation(wheel) + (degrees_sec * Time.fixedDeltaTime);
        setDrive(rotationGoal, wheel);
    }

    // private void SetDriveRotation(ArticulationBody body, Quaternion targetLocalRotation)
    // {
    //     Vector3 target = this.ToTargetRotationInReducedSpace(body, targetLocalRotation);

    //     // assign to the drive targets...
    //     ArticulationDrive xDrive = body.xDrive;
    //     xDrive.target = target.x;
    //     body.xDrive = xDrive;
    // }

    // private Vector3 ToTargetRotationInReducedSpace(ArticulationBody body, Quaternion targetLocalRotation)
    // {
    //     if (body.isRoot)
    //         return Vector3.zero;
    //     Vector3 axis;
    //     float angle;

    //     //Convert rotation to angle-axis representation (angles in degrees)
    //     targetLocalRotation.ToAngleAxis(out angle, out axis);

    //     // Converts into reduced coordinates and combines rotations (anchor rotation and target rotation)
    //     Vector3 rotInReducedSpace = Quaternion.Inverse(body.anchorRotation) * axis * angle;

    //     return rotInReducedSpace;
    // }

    // // TODO: Figure this shit out
    // private Quaternion calcTargetLocalRotation(ArticulationBody body, float speed) {
    //     // float currentRotation = GetAxisRotation(body);
    //     // float rotationGoal = currentRotation + (Time.fixedDeltaTime * (speed * Mathf.Rad2Deg));
    //     // return Quaternion.Euler(rotationGoal, 0, 0);
    //     // Quaternion rotation = body.transform.localRotation;
    //     return Quaternion.Euler(Vector3.forward * 10.0f);
    // }

    // private void rotate(ArticulationBody body) {
    //     SetDriveRotation(body, calcTargetLocalRotation(body, max_rads_per_second));
    // }
}
