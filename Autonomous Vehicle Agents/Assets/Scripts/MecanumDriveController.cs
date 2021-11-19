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
    
    // Lidar component here
    [SerializeField]
    private Lidar lidar = null;

    // No-serialized private variables here
    private float max_rads_per_second = 0;
    private float max_degs_per_second = 0;
    PlayerControls playerController;
    float initialGoalDistance;
    
    private Actions previousActions;
    private Actions currentActions; 
    private State previousState;
    private State currentState;

    private void Awake() {
        // Vector3 objectSize = Vector3.Scale(transform.localScale, GetComponent<MeshFilter>().mesh.bounds.size);
        settings = FindObjectOfType<MecanumDriveAgentSettings>();    
        decisionRequester = FindObjectOfType<DecisionRequester>();
        behaviorParameters = FindObjectOfType<BehaviorParameters>();
        playerController = new PlayerControls();
        playerController.Robot.Enable();    
        playerController.Robot.Right.Enable();
        playerController.Robot.Left.Enable();
        playerController.Robot.Switch.Enable();
        max_rads_per_second = (settings.rpm / 60.0f) * (2f * (Mathf.PI));
        max_degs_per_second = settings.rpm * 6f;
    }
   
    public override void OnEpisodeBegin()
    {   
        previousState = currentState = GetState();
        initialGoalDistance = NearestNth(Vector3.Distance(previousState.localPosition, targetTransform.localPosition), 1);
        float[] actions = {0f, 0f, 0f, 0f};
        currentActions.rawComponents = new List<float>(actions);
        currentActions.LinearDirection = Vector3.zero;
        currentActions.AngularDirection = Vector3.zero;
        previousActions = currentActions;
    }

    private void TransitionDependantRewards(State PreviousState, State CurrentState, Actions PreviousActions, Actions CurrentActions) {
        
        // Punish each step
        AddReward(settings.stepReward);
        
        // Reward/punish if we get closer/farthur to/from goal
        Vector3 currentGoalPosition = targetTransform.localPosition;
        float lastDistance = NearestNth(Vector3.Distance(PreviousState.localPosition, currentGoalPosition) * 1000f, 1);
        float currentDistance = NearestNth(Vector3.Distance(CurrentState.localPosition, currentGoalPosition) * 1000f, 1);
        float deltaDistance = lastDistance - currentDistance;
        float milliPerSecond = ((max_degs_per_second) / 360f) * (settings.wheelDiameter * Mathf.PI);
        float maxTravel = milliPerSecond * Time.fixedDeltaTime * decisionRequester.DecisionPeriod; 
        maxTravel /= 5f; // TODO: Magic number. Need to fix the physics on articulation xDrive to move at more accurate speed. Currently torque is too low.
        
        // assuming we are not just spinning in circles
        if (NearestNth(currentState.linearVelocity.magnitude * 1000f, 1) > 0f) {
            float deltaDistanceReward = Mathf.Clamp(deltaDistance / maxTravel, -1f, 1f);
            AddReward(deltaDistanceReward);
        } 

        // Reward if agent is facing goal
        float lookAtTargetReward = Vector3.Dot(targetTransform.forward, transform.up); // (   + 1f) * .5F; // for 0 to 1
        AddReward(lookAtTargetReward);

        // Punish if agent's forward movement deviates to much relative to previous action (by 90 in any direction)
        Vector3 lastDirection = Quaternion.Inverse(PreviousState.localRotation) * PreviousState.linearVelocity;
        Vector3 currentDirection = Quaternion.Inverse(CurrentState.localRotation) * CurrentState.linearVelocity;

        if (!settings.useDiscreteActions) {
            float relativeDirectionAngle;
            if (
                NearestNth(currentActions.LinearDirection.magnitude, 2) == 0f || 
                NearestNth(PreviousState.linearVelocity.x, 2) == 0f || 
                NearestNth(CurrentState.linearVelocity.magnitude, 2) == 0f
            ) {
                relativeDirectionAngle = 0.0f;
            } else {
                relativeDirectionAngle = Vector3.Angle(lastDirection, currentDirection);
            }
            if (Mathf.Abs(relativeDirectionAngle) > 45f) {    
                AddReward(-1f);
            }
        }
    }

    private State GetState() {
        return new State(){
            forwardVector = transform.forward,
            upVector = transform.up,
            localPosition = transform.localPosition,
            localRotation = transform.localRotation,
            linearVelocity = transform.InverseTransformDirection(gameObject.GetComponent<ArticulationBody>().velocity),
            angularVelocity = transform.InverseTransformDirection(gameObject.GetComponent<ArticulationBody>().angularVelocity)
        };
    }

    public override void CollectObservations(VectorSensor sensor) 
    {
        previousState = currentState;
        currentState = GetState();
        TransitionDependantRewards(previousState, currentState, previousActions, currentActions);   

        // Current lidar readings
        List<Packet> packets = lidar.UpdateSensor(false, true, 20, 360.0f);
        foreach (Packet packet in packets) {
            sensor.AddObservation(packet.distance);
            sensor.AddObservation(packet.theta);
        }

        // Current agent velocity
        sensor.AddObservation(currentState.linearVelocity.x);
        sensor.AddObservation(currentState.linearVelocity.y);
        sensor.AddObservation(currentState.angularVelocity.z);

        // Current angle to goal
        float angleToGoal = Vector3.SignedAngle( currentState.localPosition - targetTransform.localPosition, transform.forward, Vector3.up) / 180f;
        sensor.AddObservation(angleToGoal);

        // Calculate distance to goal
        float distanceToGoal = NearestNth(Vector3.Distance(currentState.localPosition, targetTransform.localPosition), 1);
        sensor.AddObservation(distanceToGoal);

        // Add the last actions taken by the agent
        foreach (float action in previousActions.rawComponents) {
            sensor.AddObservation(action);
        }
    }

    private float ToMovement(int x) {
        switch (x) {
            case 1:
                return -1f; // Reverse
            case 2:
                return 0f; // No change
            case 3:
                return 1f; // Forward
            case 4:
                return 0f; // Complete stop
            default:
                return 0f;
        }
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
         Actions newAction = new Actions();

        if (settings.useDiscreteActions) {
            newAction.rawComponents = new List<float>();

            // Either accelerate, de-accelerate, same velocity, or full stop
            float xLinear = ToMovement(actions.DiscreteActions[0]);
            float yLinear = ToMovement(actions.DiscreteActions[1]);
            float zRotation = ToMovement(actions.DiscreteActions[2]);

            // For complete stop
            // if (actions.DiscreteActions[0] == 4) {
            //     currentActions.LinearDirection.x = 0f;
            // }

            // if (actions.DiscreteActions[1] == 4) {
            //     currentActions.LinearDirection.y = 0f;
            // }

            // if (actions.DiscreteActions[2] == 4) {
            //     currentActions.AngularDirection.x = 0f;
            // }

            // Create new movement from previous and current
            // newAction.LinearDirection = Vector3.ClampMagnitude((
            //     - new Vector2() {
            //         x = (xLinear * settings.speedIncrement),
            //         y = (yLinear * settings.speedIncrement)} 
            //     + currentActions.LinearDirection) 
            //     * new Vector2() {
            //         x = Mathf.Max(Mathf.Abs(xLinear * settings.velocityRampFactor), 1f),
            //         y = Mathf.Max(Mathf.Abs(yLinear * settings.velocityRampFactor), 1f)}
            // , 1.0f);

            newAction.LinearDirection = Vector3.ClampMagnitude((
                - new Vector2() {
                    x = (xLinear * settings.speedIncrement),
                    y = (yLinear * settings.speedIncrement)} 
                + currentActions.LinearDirection), 1.0f);


            // newAction.AngularDirection = Vector2.zero - new Vector2() {
            //     x = Mathf.Clamp(currentActions.AngularDirection.x + (zRotation * settings.velocityRampFactor), -1f, 1f),
            //     y = 0f
            // };

            newAction.rawComponents = new List<float>();
            newAction.rawComponents.Add(xLinear);
            newAction.rawComponents.Add(yLinear);
            newAction.rawComponents.Add(zRotation);

    
            // GetAngularDrives( newAction.AngularDirection ) + 
            MecanumDrives fullDrives = GetLinearDrives( newAction.LinearDirection );
            setWheel(TR, -fullDrives.TR);
            setWheel(BL, fullDrives.BL);
            setWheel(TL, -fullDrives.TL);
            setWheel(BR, fullDrives.BR);
        } else {
            newAction.rawComponents = new List<float>();
            for (int i = 0; i < 4; i++) {
                float action = actions.ContinuousActions[i];
                newAction.rawComponents.Add(action);
            }

            newAction.LinearDirection = Vector2.zero - new Vector2() {
                x = actions.ContinuousActions[2],
                y = actions.ContinuousActions[3]
            }; 

            newAction.AngularDirection = Vector2.zero - new Vector2() {
                x = actions.ContinuousActions[0],
                y = actions.ContinuousActions[1],
            };
            
            MecanumDrives fullDrives = GetAngularDrives( newAction.AngularDirection ) + GetLinearDrives( newAction.LinearDirection );
            setWheel(TR, -fullDrives.TR);
            setWheel(BL, fullDrives.BL);
            setWheel(TL, -fullDrives.TL);
            setWheel(BR, fullDrives.BR);
        }
       
        previousActions = currentActions;
        currentActions = newAction;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (settings.useDiscreteActions) {
            ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
            Vector2 r = playerController.Robot.Right.ReadValue<Vector2>();
            Vector2 l = playerController.Robot.Left.ReadValue<Vector2>();
            float cruzControl = playerController.Robot.Switch.ReadValue<float>();

            if (NearestNth(r.x, 1) == 0f) {
                if (cruzControl > 0f) {
                    discreteActions[0] = 2;
                } else {
                    discreteActions[0] = 4;
                }
            } else if (NearestNth(r.x, 1) > 0f) {
                if (cruzControl > 0f) {
                    discreteActions[0] = 2;
                } else {
                    discreteActions[0] = 1;
                }
            } else {
                if (cruzControl > 0f) {
                    discreteActions[0] = 2;
                } else {
                    discreteActions[0] = 3;
                }
            }

            if (NearestNth(r.y, 1) == 0f) {
                if (cruzControl > 0f) {
                    discreteActions[1] = 2;
                } else {
                    discreteActions[1] = 4;
                }
            } else if (NearestNth(r.y, 1) > 0f) {
                if (cruzControl > 0f) {
                    discreteActions[1] = 2;
                } else {
                    discreteActions[1] = 1;
                }
            } else {
                if (cruzControl > 0f) {
                    discreteActions[1] = 2;
                } else {
                    discreteActions[1] = 3;
                }
            }


            // if (NearestNth(l.x, 1) == 0f) {
            //     if (cruzControl > 0f) {
            //         discreteActions[0] = 2;
            //     } else {
            //         discreteActions[0] = 4;
            //     }
            // } else if (NearestNth(r.x, 1) > 0f) {
            //     if (cruzControl > 0f) {
            //         discreteActions[0] = 2;
            //     } else {
            //         discreteActions[0] = 1;
            //     }
            // } else {
            //     if (cruzControl > 0f) {
            //         discreteActions[0] = 2;
            //     } else {
            //         discreteActions[0] = 3;
            //     }
            // }

            if (NearestNth(l.x, 1) == 0f) {
                discreteActions[2] = 2;
            } else if (NearestNth(l.x, 1) > 0f) {
                discreteActions[2] = 1;
            } else {
                discreteActions[2] = 3;
            }

        } else {
            ActionSegment<float> continousActions = actionsOut.ContinuousActions;
            Vector2 r = playerController.Robot.Right.ReadValue<Vector2>();
            Vector2 l = playerController.Robot.Left.ReadValue<Vector2>();

            continousActions[0] = l.x;
            continousActions[1] = l.y;
            continousActions[2] = r.x;
            continousActions[3] = r.y;
        }
        
    }

    private void OnTriggerEnter(Collider other) {
        if (other.TryGetComponent<Goal>(out Goal goal)) {
            AddReward(settings.goalReward);
            EndEpisode();
        }

        if (other.TryGetComponent<Wall>(out Wall wall)) {
            AddReward(settings.collisionReward);
            EndEpisode();
        }
    }

    private float GetAxisRotation(ArticulationBody articulation)
    {
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    private void setDrive(float rotationGoal, float currentRotation,  float targetVelocity, float forceLimit, ArticulationBody articulation)
    {
        if (rotationGoal == currentRotation) {
            return; 
        }

        ArticulationDrive drive = articulation.xDrive;
        drive.target = rotationGoal;
        drive.forceLimit = forceLimit;
        drive.stiffness = 10000;
        drive.damping = 100;
        drive.targetVelocity = targetVelocity;
        articulation.xDrive = drive;
    }

    private void setVelocity(float targetVelocity, float forceLimit, ArticulationBody articulation) 
    {
        ArticulationDrive drive = articulation.xDrive;
        drive.targetVelocity = targetVelocity; 
        drive.forceLimit = forceLimit; 
        drive.stiffness = 0f;
        drive.damping = 10000;
        articulation.xDrive = drive;
    }

    void setWheel(ArticulationBody wheel, float speed) 
    {
        float interval = decisionRequester.DecisionPeriod * Time.fixedDeltaTime;
        float deltaDegrees = (speed * max_degs_per_second) * interval;
        float angularVelocity = (speed * max_rads_per_second) / Time.fixedDeltaTime;
        float forceLimit = settings.torque; // TODO: settings.torque / Time.deltaTime (right now it makes the robot FLIP around)

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
    private Vector3 clipAngles(Vector3 angles) {
        angles.x = to360(angles.x);
        angles.y = to360(angles.y);
        angles.z = to360(angles.z);
        return angles;
    }
    // private Vector3 angularAmount(LineVector rotation) {
    //     float speed = (rotation.M );
    //     Vector3 direction = rotation.X > 0 ? originalRight : -originalRight;
    //     Quaternion qTo = Quaternion.LookRotation(Vector3.right);
    //     Quaternion qFrom = Quaternion.LookRotation(Vector3.forward);
    //     Vector3 angle = Quaternion.RotateTowards(qFrom, qTo, speed * max_degs_per_second * Time.fixedDeltaTime).eulerAngles;
    //     return clipAngles(angle);
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
        float scalar = Mathf.Pow(10f, place);
        return (float)((int)(n * scalar)) / scalar;
    }
}
