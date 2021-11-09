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

public enum Q {
    I,
    II,
    III,
    IV, 
    none          
};

struct MecanumDrives {
    public float TL;
    public float TR;
    public float BL;
    public float BR;

    public static MecanumDrives operator +(MecanumDrives a, MecanumDrives b) {
        return new MecanumDrives() {
            TL = Mathf.Clamp(a.TL + b.TL, -1f, 1f),
            TR = Mathf.Clamp(a.TR + b.TR, -1f, 1f),
            BL = Mathf.Clamp(a.BL + b.BL, -1f, 1f),
            BR = Mathf.Clamp(a.BR + b.BR, -1f, 1f)
        };
    }
}

public struct LineVector 
{
    private Vector2 p;
    private Vector2 q;

    public float X; // X component
    public float Y; // Y component
    public float M; // Magnitude
    public float A; // Angle of unit circle 

    public LineVector(Vector2 p1, Vector2 p2) {
        p = p1;
        q = p2;
        X = q.x - p.x;
        Y = q.y - p.y;
        M = Mathf.Sqrt(Mathf.Pow(X, 2) + Mathf.Pow(Y, 2));
        A = Mathf.Atan2(Y, X);
        A *= 180f / Mathf.PI; // To degrees -180 <-> 180
        A = A < 0f ? A + 360f : A;  // To 0 <-> 360
    }
};

/* 
  Ensure motors are laid out in this fashion:

  (M1)  TL _____TR (M2)
          |     |
          |     |
          |_____|
  (M2)  BL       BR (M1)
  (Left)         (Right) 
  
  Left motor channel: TR <---> BR
  Right motor channel: TL <---> BL
  
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
    
    // Wheel components here
    [SerializeField] 
    private ArticulationBody TL = null;
    [SerializeField] 
    private ArticulationBody TR = null;
    [SerializeField] 
    private ArticulationBody BL = null;
    [SerializeField] 
    private ArticulationBody BR = null;
    
    // Lidar component here
    [SerializeField]
    private Lidar lidar = null;


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
    
        List<Packet> packets = lidar.UpdateSensor();
        
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float tlSpeed = actions.ContinuousActions[0];
        float trSpeed = actions.ContinuousActions[1];
        float blSpeed = actions.ContinuousActions[2];
        float brSpeed = actions.ContinuousActions[3];
        
        setWheel(TR, -trSpeed);
        setWheel(BL, blSpeed);
        setWheel(TL, -tlSpeed);
        setWheel(BR, brSpeed);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
            
        Vector2 r = playerController.Robot.Right.ReadValue<Vector2>();
        Vector2 l = playerController.Robot.Left.ReadValue<Vector2>();

        MecanumDrives fullDrives = GetAngularDrives(new LineVector(
            Vector2.zero, 
            l
        )) + GetLinearDrives(new LineVector(
            Vector2.zero,
            r
        ));

        continousActions[0] = fullDrives.TL;
        continousActions[1] = fullDrives.TR;
        continousActions[2] = fullDrives.BL;
        continousActions[3] = fullDrives.BR;
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

    private Q quadrant(double angle) {
        if (angle >= 0.0 && angle < 90.0) return Q.I;
        if (angle >= 90.0 && angle < 180.0) return Q.II;
        if (angle >= 180.0 && angle < 270.0) return Q.III;
        if (angle >= 270.0 && angle < 360.0) return Q.IV;
        return Q.none;
    }

    MecanumDrives GetAngularDrives(LineVector rotation) {
        MecanumDrives drives;
        float dir = rotation.X > 0f ? 1f : -1f;
        float r = -dir * rotation.M;
        float l = dir * rotation.M;
        drives.TL = -l;
        drives.TR = -r;
        drives.BL = l;
        drives.BR = r;
        return drives;
    }

    MecanumDrives GetLinearDrives(LineVector direction) {
        MecanumDrives currentDrives;
        float angle = direction.A;
        float magnitude = direction.M;
        float RRatio = 1.0f; // Ratio between Left and Right motor drivers 
        float LRatio = 1.0f;
        float LDrive = 1.0f; // Controls M1 <---> M4
        float RDrive = 1.0f; // Controls M2 <---> M3

        switch (quadrant(angle)) {
            case Q.I:
                // LRatio constant: 1.0
                // angle = 45 * (1.0 + RRatio)
                RRatio = (angle / 45.0f) - 1.0f;
                RDrive = RRatio * magnitude;
                LDrive = LRatio * magnitude;    
                currentDrives.TL = -LDrive;
                currentDrives.TR = -RDrive;
                currentDrives.BL = RDrive;
                currentDrives.BR = LDrive;
                break;
            case Q.II:
                // RRatio constant: 1.0
                // angle = 45 * (1.0 - LRatio) + 90
                LRatio = -((angle - 90.0f) / 45.0f) + 1.0f;
                RDrive = RRatio * magnitude;
                LDrive = LRatio * magnitude;
                currentDrives.TL = -LDrive;
                currentDrives.TR = -RDrive;
                currentDrives.BL = RDrive;
                currentDrives.BR = LDrive;
                break;
            case Q.III:
                // LRatio constant: -1.0
                // angle = 45 * ((-RRatio) - (- 1)) + 180
                RRatio = -((angle - 180.0f) / 45.0f) + 1.0f;
                RDrive = RRatio * magnitude;
                LDrive = LRatio * magnitude;
                currentDrives.TL = LDrive;
                currentDrives.TR = -RDrive;
                currentDrives.BL = RDrive;
                currentDrives.BR = -LDrive;
                break;
            case Q.IV:
                // RRatio constant: -1.0
                // angle = 45 * (1 + LRatio)) + 270.0
                LRatio = ((angle - 270.0f) / 45.0f) - 1.0f;
                RDrive = RRatio * magnitude;
                LDrive = LRatio * magnitude;
                currentDrives.TL = -LDrive;
                currentDrives.TR = RDrive;
                currentDrives.BL = -RDrive;
                currentDrives.BR = LDrive;
                break;
            default:
                currentDrives.TL = 0f;
                currentDrives.TR = 0f;
                currentDrives.BL = 0f;
                currentDrives.BR = 0f;
                break;
        }

        return currentDrives;
    }
}
