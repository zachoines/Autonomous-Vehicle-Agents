using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

public class MoveToGoalAgent : Agent
{
    Vector3 lastPos;
    public override void Initialize()
    {

        SetResetParameters();
    }

    public override void OnEpisodeBegin()
    {
        // transform.localPosition = new Vector3(Random.Range(-6.0f, 6.0f), Random.Range(0.0f, 0.0f), Random.Range(-6.0f, 6.0f));
        // targetTransform.localPosition = new Vector3(Random.Range(-6.0f, 6.0f), Random.Range(0.0f, 0.0f), Random.Range(-6.0f, 6.0f));
        transform.localPosition = new Vector3(0.0f, 1.1f, 0.0f);
        lastPos = transform.localPosition;
    }

    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    public override void CollectObservations(VectorSensor sensor)
    {
        
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTransform.localPosition);

        float dist1 = Vector3.Distance(transform.localPosition, targetTransform.localPosition);
        float dist2 = Vector3.Distance(lastPos, targetTransform.localPosition);
        lastPos = transform.localPosition;

        if (dist1 < dist2) {
            AddReward(.1f);
        } else {
            AddReward(-.1f);
        }

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];
        float moveSpeed = 1;
        transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * moveSpeed;
    }
    private void OnTriggerEnter(Collider other) {
        if (other.TryGetComponent<Goal>(out Goal goal)) {
            AddReward(1f);
            floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }

        if (other.TryGetComponent<Wall>(out Wall wall)) {
            AddReward(-1f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
    }
    

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        continousActions[0] = Input.GetAxisRaw("Horizontal");
        continousActions[1] = Input.GetAxisRaw("Vertical");
    }

    public void SetResetParameters()
    {
     
    }
}
