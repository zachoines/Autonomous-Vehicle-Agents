using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;


/*
    MecanumDriveController represents a simulation of a 4-wheeled omni-direction mecanum robot. 
    Movement is based on encoder ticks. Internally, speed and acceleration is measured 
    in quadrate of encoders. Distance traveled is inferred by wheel Diameter.
*/

public enum DiscreteActionClasses {
    DEACCELERATE = 1,
    CONSTANT = 2,
    ACCELERATE = 3,
    STOP = 4
}

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

public struct State
{
    public Quaternion localRotation;
    public Vector3 localPosition;
    public Vector3 linearVelocity;
    public Vector3 angularVelocity;

    public Vector3 forwardVector;
    public Vector3 upVector;
    public float distanceToGoal;
    
}

public struct Actions
{
    public Vector2 AngularDirection;
    public Vector2 LinearDirection;
    public List<float> rawComponents;
}
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

    MecanumDriveAgentSettings settings;
    DecisionRequester decisionRequester;

    BehaviorParameters behaviorParameters;

    // The target to reach
    [SerializeField] 
    private Transform targetTransform;
    
    // Wheel components here
    [SerializeField] 
    private ArticulationBody TL = null;
    [SerializeField] 
    private ArticulationBody TR = null;
    [SerializeField] 
    private ArticulationBody BL = null;
    [SerializeField] 
    private ArticulationBody BR = null;

    private List<ArticulationBody> wheels;
    
    [SerializeField] 
    private ArticulationBody frame = null;
    
    // Lidar component here
    [SerializeField]
    private Lidar lidar = null;

    [SerializeField]
    [Header("Goal Spawn Radias", order = 999)]
    private float spawnRadius = 1f;

    [SerializeField]
    [Header("Arena", order = 999)]
    public GameObject arena;

    [SerializeField]
    [Header("Arena bounds", order = 999)]
    public Bounds arenaBounds;

    // No-serialized private variables here
    private float max_rads_per_second = 0;
    private float max_degs_per_second = 0;
    PlayerControls playerController;
    private Actions previousActions;
    private Actions currentActions; 
    private State previousState;
    private State currentState;
    private Vector3 robotStartingPosition;
    private Vector3 goalStartingPosition;
    private float lastDistanceToGoal;
    Collider m_collider = null;

    // [SerializeField]
    // [Header("Buffer Lidar", order = 999)]
    // BufferSensorComponent m_BufferSensorLidar;

    // [SerializeField]
    // [Header("Buffer Wheels", order = 999)]
    // BufferSensorComponent m_BufferSensorWheels;
    
    
    // Communicate with other objects here
    private void Start() {
        m_collider = GetComponent<Collider>();    

        // Init connected controllers 
        settings = FindObjectOfType<MecanumDriveAgentSettings>();    
        decisionRequester = FindObjectOfType<DecisionRequester>();
        behaviorParameters = FindObjectOfType<BehaviorParameters>();
        arenaBounds = arena.GetComponent<Collider>().bounds;

        // Init game controller
        playerController = new PlayerControls();
        playerController.Robot.Enable();    
        playerController.Robot.Right.Enable();
        playerController.Robot.Left.Enable();
        playerController.Robot.Switch.Enable();

        // Init other local variables
        max_rads_per_second = (settings.rpm / 60.0f) * (2f * (Mathf.PI));
        max_degs_per_second = settings.rpm * 6f;
        robotStartingPosition = transform.position;    
        goalStartingPosition = targetTransform.position;    

        // Keeps the robot moving in straight lines
        frame.solverVelocityIterations = 25;

        // Less noise to AI
        frame.solverIterations = 25;

        wheels = new List<ArticulationBody>() { TL, TR, BL, BR };

    }

    private void ResetGoal() {
        Vector3 pos = GenerateNewSpawn();
        pos.y = goalStartingPosition.y;
        targetTransform.SetPositionAndRotation(pos, targetTransform.localRotation);
    }

    public override void OnEpisodeBegin()
    {   
        // Reset Robot and Goal positions
        ResetRobot();
        ResetGoal();

        // Init initial state and action/drive variables
        previousState = currentState = GetState();
        float[] driveComponents = { 0f, 0f, 0f, 0f };
        currentActions.rawComponents = new List<float>(driveComponents);
        currentActions.LinearDirection = Vector3.zero;
        currentActions.AngularDirection = Vector3.zero;
        previousActions = currentActions; 
        lastDistanceToGoal = currentState.distanceToGoal;
    }
    private void StopRobot() {
        // Zero out Articulation velocities for frame and wheels
        TL.velocity = Vector3.zero;
        TR.velocity = Vector3.zero;
        BL.velocity = Vector3.zero;
        BR.velocity = Vector3.zero;
        frame.velocity = Vector3.zero;
    }
    private void ResetRobot() {
        
        StopRobot();

        // Set new location for frame
        Quaternion newRotation = Quaternion.Euler(-90, 0, Random.Range(0.0f, 360.0f));
        Vector3 newPosition = GenerateNewSpawn();
        newPosition.y = transform.position.y;
        transform.position = newPosition;
        transform.rotation = newRotation;
        frame.TeleportRoot(transform.position, transform.rotation);
    }

    private float getReward(State CurrentState, State PreviousState) {

        float totalReward = 0f;

        // Reward if agent is facing goal
        Vector3 toPosition = Vector3.Normalize(targetTransform.position - transform.position);
        float lookAtTargetReward = Vector3.Dot(transform.right, toPosition);
        if (float.IsNaN(lookAtTargetReward)) {
            lookAtTargetReward = 0f;
        }

        totalReward += (settings.lookAtTargetReward * lookAtTargetReward / MaxStep);

        // Reward marginal movements towards goal
        // float milliPerSecond = ((max_degs_per_second) / 360f) * (settings.wheelDiameter * Mathf.PI);
        // float maxTravel = milliPerSecond * Time.fixedDeltaTime * decisionRequester.DecisionPeriod;
        // float deltaDistanceToGoalReward = Mathf.Clamp(NearestNth(previousState.distanceToGoal - CurrentState.distanceToGoal, 3) / (maxTravel / 1000f), -1f, 1f);
        // totalReward += (settings.deltaDistanceReward * deltaDistanceToGoalReward / MaxStep);

        // Punish if agent's forward movement deviates to much relative to previous action (by 45 in any direction)
        // Vector3 lastDirection = Quaternion.Inverse(PreviousState.localRotation) * PreviousState.linearVelocity;
        // Vector3 currentDirection = Quaternion.Inverse(CurrentState.localRotation) * CurrentState.linearVelocity;
        // float relativeDirectionAngle;
        // if (
        //     NearestNth(currentActions.LinearDirection.magnitude, 2) == 0f || 
        //     NearestNth(PreviousState.linearVelocity.x, 2) == 0f || 
        //     NearestNth(CurrentState.linearVelocity.magnitude, 2) == 0f
        // ) {
        //     relativeDirectionAngle = 0.0f;
        // } else {
        //     relativeDirectionAngle = Vector3.Angle(lastDirection, currentDirection);
        // }
        // if (Mathf.Abs(relativeDirectionAngle) > 45f) {    
        //     AddReward(-1f);
        // }
        
        // Reward if withing goal radius (end condition)
        if (CurrentState.distanceToGoal < settings.minDistanceToGoal) { 
            totalReward += settings.goalReward;
            ResetGoal();
        }

        // Step reward
        totalReward += (settings.totalStepReward / MaxStep);

        return totalReward;
    }

    private State GetState() {
        Vector3 trueCenter =  GetComponent<Renderer>().bounds.center;
        return new State(){
            forwardVector = transform.forward,
            upVector = transform.up,
            localPosition = transform.localPosition,
            localRotation = transform.localRotation,
            linearVelocity = frame.velocity,
            angularVelocity = frame.angularVelocity,
            distanceToGoal = Vector3.Distance(trueCenter, targetTransform.position),
        };
    }

    public override void CollectObservations(VectorSensor sensor) 
    {   


        previousState = currentState;     
        currentState = GetState();

        /*
        // Linear/Angular Velocities of the main body
        sensor.AddObservation(currentState.linearVelocity.x);
        sensor.AddObservation(currentState.linearVelocity.y);
        sensor.AddObservation(currentState.angularVelocity.z);

        // Calculate distance to goal. Make detectable distance the same as lidar range
        float distanceToGoal = currentState.distanceToGoal;
        sensor.AddObservation(Mathf.Clamp(distanceToGoal / lidar.LaserLength, 0f, 1f));

        // Relative angle to goal
        Vector3 toPosition = Vector3.Normalize(targetTransform.position - transform.position);
        float angleToPosition = Vector3.SignedAngle(transform.right, toPosition, Vector3.up);
        sensor.AddObservation(Mathf.Clamp(angleToPosition / 180f, -1f, 1f));

        // Uses attention model to track wheel positions
        for (int wheel = 0; wheel < 4; wheel++) {
            float[] wheelObservation = new float[5];
            ArticulationBody w = wheels[wheel];
            Vector3 trueCenter = w.GetComponent<Renderer>().bounds.center;
            wheelObservation[0] = (float)wheel / 4f; // ID of this wheel
            wheelObservation[1] = GetJointVelocity(w) / max_rads_per_second; // Relative velocity of the wheels
            wheelObservation[2] = (GetAxisRotation(w) % 360f) / 360f; // Relative location of the wheels
            wheelObservation[3] = previousActions.rawComponents[wheel]; // Add the last drive components
            wheelObservation[4] = Mathf.Clamp(Vector3.Distance(trueCenter, targetTransform.position) / lidar.LaserLength, 0f, 1f); // Distance of this wheel to goal
            m_BufferSensorWheels.AppendObservation(wheelObservation);
        }
        
        // Use attention model to track lidar at angles
        List<Packet> packets = lidar.UpdateSensor(false, true, settings.numLidarSamples, 360.0f);
        for (int packet = 0; packet < settings.numLidarSamples; packet++) {
            float[] lidarObservation = new float[4];
            Packet p = packets[packet];
            lidarObservation[0] = (float)packet / (float)settings.numLidarSamples; // ID of this lidar sample
            lidarObservation[1] = p.distance; // Distance to detected object
            lidarObservation[2] = p.theta; // Local Angle
            lidarObservation[3] = Mathf.Clamp(to360(angleToPosition + (p.theta * 360f)) / 360f, 0f, 1f); // Relative Angle to goal
            m_BufferSensorLidar.AppendObservation(lidarObservation);
        } */
        
        // Linear/Angular Velocities of the main body and wheels 
        sensor.AddObservation(currentState.linearVelocity.x);
        sensor.AddObservation(currentState.linearVelocity.y);
        sensor.AddObservation(currentState.angularVelocity.z);

        // Relative angle to goal
        Vector3 toPosition = Vector3.Normalize(targetTransform.position - transform.position);
        float angleToPosition = Vector3.SignedAngle(transform.right, toPosition, Vector3.up);
        sensor.AddObservation(Mathf.Clamp(angleToPosition / 180f, -1f, 1f));

        // Calculate distance to goal. Make detectable distance the same as lidar range
        float distanceToGoal = currentState.distanceToGoal;
        sensor.AddObservation(Mathf.Clamp(distanceToGoal / lidar.LaserLength, 0f, 1f));

         for (int wheel = 0; wheel < 4; wheel++) {
            float[] wheelObservation = new float[5];
            ArticulationBody w = wheels[wheel];
            sensor.AddObservation(GetJointVelocity(w) / max_rads_per_second); // Linear/Angular Velocities of wheels 
            sensor.AddObservation((GetAxisRotation(w) % 360f) / 360f); // Relative location of the wheels
            sensor.AddObservation(previousActions.rawComponents[wheel]); // Add the last drive components
        }

        // Current lidar readings
        List<Packet> packets = lidar.UpdateSensor(false, true, settings.numLidarSamples, 360.0f);
        foreach (Packet packet in packets) {
            sensor.AddObservation(packet.distance);            
            // sensor.AddObservation(packet.theta);
        }
    }

    private float ToMovement(int x) {
        switch (x) {
            case (int)DiscreteActionClasses.DEACCELERATE:
                return -1f;
            case (int)DiscreteActionClasses.CONSTANT:
                return 0f;
            case (int)DiscreteActionClasses.ACCELERATE:
                return 1f; 
            case (int)DiscreteActionClasses.STOP:
                return 0f;
            default:
                return 0f;
        }
    }
    
    public override void OnActionReceived(ActionBuffers actions)
    {

        Actions newActions = new Actions();
        Actions lastActions = currentActions;

        if (settings.useDiscreteActions) {
            newActions.rawComponents = new List<float>();

            // Either accelerate, de-accelerate, same velocity, or full stop
            float xLinear = ToMovement(actions.DiscreteActions[0]);
            float yLinear = ToMovement(actions.DiscreteActions[1]);
            float zRotation = ToMovement(actions.DiscreteActions[2]);

            // For complete stop
            if (actions.DiscreteActions[0] == (int)DiscreteActionClasses.STOP) { lastActions.LinearDirection.x = 0f; }
            if (actions.DiscreteActions[1] == (int)DiscreteActionClasses.STOP) { lastActions.LinearDirection.y = 0f; }
            if (actions.DiscreteActions[2] == (int)DiscreteActionClasses.STOP) { lastActions.AngularDirection.x = 0f; }

            /* Linear drive with increasing acceleration */
            // newAction.LinearDirection = Vector3.ClampMagnitude((
            //     - new Vector2() {
            //         x = (xLinear * settings.velocityIncrement),
            //         y = (yLinear * settings.velocityIncrement)} 
            //     + lastActions.LinearDirection) 
            //     * new Vector2() {
            //         x = Mathf.Max(Mathf.Abs(xLinear * settings.accelerationMultiplier), 1f),
            //         y = Mathf.Max(Mathf.Abs(yLinear * settings.accelerationMultiplier), 1f)}
            // , 1.0f);

            /* Linear drive with constant acceleration */
            newActions.LinearDirection = Vector2.ClampMagnitude((
                new Vector2() {
                    x = (xLinear * settings.velocityIncrement),
                    y = (yLinear * settings.velocityIncrement)} 
                + lastActions.LinearDirection), 1.0f);

            /* Angular drive with constant acceleration */
            newActions.AngularDirection = Vector2.ClampMagnitude((
                new Vector2() {
                    x = (zRotation * settings.velocityIncrement),
                    y = 0f}
                    + lastActions.AngularDirection), 1.0f);


            /* Add linear and angular components to get 4 drives which represent 4 wheel speeds */
            MecanumDrives fullDrives =  GetLinearDrives( newActions.LinearDirection ) + GetAngularDrives( newActions.AngularDirection );
            setWheel(TR, -fullDrives.TR);
            setWheel(BL, fullDrives.BL);
            setWheel(TL, -fullDrives.TL);
            setWheel(BR, fullDrives.BR);

            newActions.rawComponents = new List<float>();
            newActions.rawComponents.Add(-fullDrives.TL);
            newActions.rawComponents.Add(-fullDrives.TR);
            newActions.rawComponents.Add(fullDrives.BL);
            newActions.rawComponents.Add(fullDrives.BR);
            
        } else {
            newActions.rawComponents = new List<float>();
            for (int i = 0; i < 4; i++) {
                float action = actions.ContinuousActions[i];
                newActions.rawComponents.Add(action);
            }

            newActions.LinearDirection = new Vector2() {
                x = actions.ContinuousActions[2],
                y = actions.ContinuousActions[3]
            }; 

            newActions.AngularDirection = new Vector2() {
                x = actions.ContinuousActions[0],
                y = actions.ContinuousActions[1],
            };
            
            MecanumDrives fullDrives = GetAngularDrives( newActions.AngularDirection ) + GetLinearDrives( newActions.LinearDirection );
            setWheel(TR, -fullDrives.TR);
            setWheel(BL, fullDrives.BL);
            setWheel(TL, -fullDrives.TL);
            setWheel(BR, fullDrives.BR);
        }

        AddReward(getReward(currentState, previousState));
        previousActions = currentActions;
        currentActions = newActions;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        Vector2 r = playerController.Robot.Right.ReadValue<Vector2>();
        Vector2 l = playerController.Robot.Left.ReadValue<Vector2>();

        if (settings.useDiscreteActions) {
            ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
            bool cruzControl = playerController.Robot.Switch.ReadValue<float>() > 0f;

            // Controls 'x' component of linear drive
            if (NearestNth(r.x, 1) == 0f) {
                if (cruzControl) {
                    discreteActions[0] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[0] = (int)DiscreteActionClasses.STOP;
                }
            } else if (r.x < 0f) {
                if (cruzControl) {
                    discreteActions[0] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[0] = (int)DiscreteActionClasses.DEACCELERATE;
                }
            } else {
                if (cruzControl) {
                    discreteActions[0] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[0] = (int)DiscreteActionClasses.ACCELERATE;
                }
            }

            // Controls 'y' component of linear drive
            if (NearestNth(r.y, 1) == 0f) {
                if (cruzControl) {
                    discreteActions[1] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[1] = (int)DiscreteActionClasses.STOP;
                }
            } else if (r.y < 0f) {
                if (cruzControl) {
                    discreteActions[1] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[1] = (int)DiscreteActionClasses.DEACCELERATE;
                }
            } else {
                if (cruzControl) {
                    discreteActions[1] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[1] = (int)DiscreteActionClasses.ACCELERATE;
                }
            }

            // Control 'z' component of angular drive
            if (NearestNth(l.x, 1) == 0f) {
                if (cruzControl) {
                    discreteActions[2] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[2] = (int)DiscreteActionClasses.STOP;
                }
            } else if (l.x < 0f) {
                if (cruzControl) {
                    discreteActions[2] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[2] = (int)DiscreteActionClasses.DEACCELERATE;
                }
            } else {
                if (cruzControl) {
                    discreteActions[2] = (int)DiscreteActionClasses.CONSTANT;
                } else {
                    discreteActions[2] = (int)DiscreteActionClasses.ACCELERATE;
                }
            }

        } else {
            ActionSegment<float> continousActions = actionsOut.ContinuousActions;
            continousActions[0] = l.x;
            continousActions[1] = l.y;
            continousActions[2] = r.x;
            continousActions[3] = r.y;
        }
    }

    private void OnTriggerEnter(Collider other) {
        if (other.gameObject.CompareTag("Goal")) {
            AddReward(settings.goalReward);
            ResetGoal();
        } else if (other.gameObject.CompareTag("Wall")) {
            AddReward(settings.collisionReward);
            EndEpisode();
        }
    }

    // private void OnTriggerStay(Collider other) {
    //     if (other.TryGetComponent<Wall>(out Wall wall)) {
    //         AddReward(settings.collisionReward);
    //         // EndEpisode();
    //     }
    // }

    // private void OnCollisionStay(Collision other) {
    //     if (other.gameObject.CompareTag("Wall")) {
    //         EndEpisode();
    //         AddReward(settings.collisionReward);
    //     }
    // }    

    // private void OnCollisionEnter(Collision other) {
    //     if (other.gameObject.CompareTag("Wall")) {
    //         AddReward(settings.collisionReward);
    //         EndEpisode();
    //     } else if (other.gameObject.CompareTag("Goal")) {
    //         AddReward(settings.goalReward);
    //         ResetGoal();
    //     }
    // }

    private float GetAxisRotation(ArticulationBody articulation, bool radians=false)
    {
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    //  In rad/s (radians per second)
    private float GetJointVelocity(ArticulationBody articulation) {
        return articulation.jointVelocity[0];
    }

    private void setDrive(float rotationGoal, float currentRotation,  float targetVelocity, float forceLimit, ArticulationBody articulation)
    {
        ArticulationDrive drive = articulation.xDrive;
        drive.target = rotationGoal;
        drive.forceLimit = forceLimit;
        drive.stiffness = 1000;
        drive.damping = 0;
        drive.targetVelocity = targetVelocity;
        articulation.xDrive = drive;
    }

    private void setVelocity(float targetVelocity, float forceLimit, ArticulationBody articulation) 
    {
        ArticulationDrive drive = articulation.xDrive;
        articulation.solverVelocityIterations = 25;
        drive.targetVelocity = targetVelocity; 
        drive.forceLimit = forceLimit;
        drive.stiffness = 0f;
        drive.damping = 1000f;
        articulation.xDrive = drive;
    }

    void setWheel(ArticulationBody wheel, float speed) 
    {
        float interval = decisionRequester.DecisionPeriod * Time.fixedDeltaTime;
        float deltaDegrees = (speed * max_degs_per_second) * interval;
        float angularVelocity = (speed * max_rads_per_second) / Time.fixedDeltaTime;
        float forceLimit = settings.torque / Time.fixedDeltaTime;

        if (settings.useVelocity) {
            setVelocity(angularVelocity, forceLimit, wheel);
        } else {
            float currentRotation = GetAxisRotation(wheel);
            float rotationGoal = currentRotation + deltaDegrees;
            setDrive(rotationGoal, currentRotation, angularVelocity, forceLimit, wheel);
        } 
    }

    private Q quadrant(double angle) {
        if (angle >= 0.0 && angle < 90.0) return Q.I;
        if (angle >= 90.0 && angle < 180.0) return Q.II;
        if (angle >= 180.0 && angle < 270.0) return Q.III;
        if (angle >= 270.0 && angle < 360.0) return Q.IV;
        return Q.none;
    }

    private float to360(float angle) {

        // Constrain to 0 <--> 360
        angle %= 360f;
        angle = angle < 0.0 ? angle + 360 : angle;
        // Convert 0 <-> 360 to 0 <-> 180 
        // angle = angle > 180f ? angle - 180f : angle;
        return angle;    
    }
    // private Vector3 clipAngles(Vector3 angles) {
    //     angles.x = to360(angles.x);
    //     angles.y = to360(angles.y);
    //     angles.z = to360(angles.z);
    //     return angles;
    // }

    MecanumDrives GetAngularDrives(Vector3 rotation) {
        MecanumDrives drives;
        float dir = rotation.x > 0f ? 1f : -1f;
        float r = Mathf.Clamp(-dir * rotation.magnitude, -1f, 1f);
        float l = Mathf.Clamp(dir * rotation.magnitude, -1f, 1f);
        drives.TL = -l;
        drives.TR = -r;
        drives.BL = l;
        drives.BR = r;
        return drives;
    }

    MecanumDrives GetLinearDrives(Vector3 direction) {
        MecanumDrives currentDrives;
        float angle = Mathf.Atan2(direction.y, direction.x);
        angle *= 180f / Mathf.PI; // To degrees -180 <-> 180
        angle = angle < 0f ? angle + 360f : angle;  // To 0 <-> 360
        float magnitude = Mathf.Clamp(direction.magnitude, -1f, 1f);
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

    private float NearestNth(float n, int place) 
    {
        if (place <= 0) {
            return Mathf.Floor(n);
        }

        float scalar = Mathf.Pow(10f, place);
        return (float)((int)(n * scalar)) / scalar;
    }

    private float map(float value, float fromLow, float fromHigh, float toLow, float toHigh) 
    {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

    public Vector3 GenerateNewSpawn()
    {
        bool foundNewSpawnLocation = false;
        Vector3 newSpawnPos = Vector3.zero;

        while (foundNewSpawnLocation == false)
        {
            float randomPosX = Random.Range(-arenaBounds.extents.x * spawnRadius,
                arenaBounds.extents.x * spawnRadius);

            float randomPosZ = Random.Range(-arenaBounds.extents.z * spawnRadius,
                arenaBounds.extents.z * spawnRadius);
            newSpawnPos = arena.transform.position + new Vector3(randomPosX, 1f, randomPosZ);
            if (Physics.CheckBox(newSpawnPos, new Vector3(.5f, 0.01f, .5f)) == false)
            {
                foundNewSpawnLocation = true;
            }
        }
       
        return newSpawnPos;
    }
}
