using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;


/*
    MecanumDriveController represents a simulation of a 4-wheeled omni-direction mecanum robot. 
    Movement is based on encoder ticks. Internally, speed and acceleration is measured 
    in quadrate of encoders. Distance traveled is inferred by wheel Diameter.
*/

public class MecanumDriveController : Agent
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

   
    public override void OnEpisodeBegin()
    {
        // transform.localPosition = new Vector3(Random.Range(-6.0f, 6.0f), Random.Range(0.0f, 0.0f), Random.Range(-6.0f, 6.0f));
        // targetTransform.localPosition = new Vector3(Random.Range(-6.0f, 6.0f), Random.Range(0.0f, 0.0f), Random.Range(-6.0f, 6.0f));
        transform.localPosition = new Vector3(0.0f, 1.1f, 0.0f);
    }

    public override void CollectObservations(VectorSensor sensor) 
    {
        
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float trSpeed = actions.ContinuousActions[0];
        float brSpeed = actions.ContinuousActions[1];
        float tlSpeed = actions.ContinuousActions[2];
        float blSpeed = actions.ContinuousActions[3];
    
        setWheel(TR, trSpeed);
        setWheel(BL, blSpeed);
        setWheel(TL, tlSpeed);
        setWheel(BR, brSpeed);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
            
        Vector2 r = playerController.Robot.Right.ReadValue<Vector2>();
        Vector2 l = playerController.Robot.Left.ReadValue<Vector2>();
        float switched = playerController.Robot.Switch.ReadValue<float>();

        if (switched <= 0.0) {
            continousActions[0] = r.y;
            continousActions[1] = l.y;
            continousActions[2] = l.y;
            continousActions[3] = r.y;
            
        } else {
            continousActions[0] = r.y;
            continousActions[1] = r.y;
            continousActions[2] = l.y;
            continousActions[3] = l.y;
        }
    }

     private void OnTriggerEnter(Collider other) {
        if (other.TryGetComponent<Goal>(out Goal goal)) {
            AddReward(1f);
            EndEpisode();
        }

        if (other.TryGetComponent<Wall>(out Wall wall)) {
            AddReward(-1f);
            EndEpisode();
        }
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

    private void setVelocity(float v, ArticulationBody articulation) 
    {
        ArticulationDrive drive = articulation.xDrive;
        drive.targetVelocity = v;
        drive.forceLimit = torque;
        articulation.xDrive = drive;
    }

    void setWheel(ArticulationBody wheel, float speed) 
    {
        float degrees_sec = (speed * max_degs_per_second);
        float rotationGoal = GetAxisRotation(wheel) + (degrees_sec * Time.fixedDeltaTime);
        setDrive(rotationGoal, wheel);
    }
}
