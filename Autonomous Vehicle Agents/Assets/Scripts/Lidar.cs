using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;
public struct LaserInput
{
    /// <summary>
    /// Laser beam length
    /// </summary>
    public float LaserLength;

    /// <summary>
    /// Detectable Object Types
    /// </summary>
    public IReadOnlyList<string> DetectableTags;

    /// <summary>
    /// Orientation relative to gameobject
    /// </summary>
    public Vector3 LaserOrientation;

    /// <summary>
    /// Filtering options for the casts.
    /// </summary>
    public int LayerMask;
}

public struct LaserOutput
{

    /// <summary>
    /// Has Laser Hit something
    /// </summary>
    public bool HasHit;

    /// <summary>
    /// Is the hit object in the Tagged objects list
    /// </summary>
    public bool HitTaggedObject;

    /// <summary>
    /// The index of the hit object's tag in the DetectableTags list
    /// -1 if there was no hit
    /// </summary>
    public int HitClass;

    /// <summary>
    /// Normalized distance to the hit object.
    /// </summary>
    public float distance;

    /// <summary>
    /// Current planar rotation relative to attached gameobject's forward vector.
    /// </summary>
    public Vector3 theta;

    /// <summary>
    /// The hit GameObject (or null if there was no hit).
    /// </summary>
    public GameObject HitGameObject;

    /// <summary>
    /// Start position of the Laser in world space.
    /// </summary>
    public Vector3 StartPositionWorld;

    /// <summary>
    /// End position of the ray in world space.
    /// </summary>
    public Vector3 EndPositionWorld;


    /// <summary>
    /// End position of the ray in local space.
    /// </summary>
    public Vector3 LocalPosition;


    /// <summary>
    /// The scaled length of the ray.
    /// </summary>
    /// <remarks>
    /// If there is non-(1,1,1) scale, |EndPositionWorld - StartPositionWorld| will be different from
    /// the input LaserLength.
    /// </remarks>
    public float ScaledLaserLength
    {
        get
        {
            var rayDirection = EndPositionWorld - StartPositionWorld;
            return rayDirection.magnitude;
        }
    }
}


public struct Packet {
    public float distance;
    public float theta;
}

public class Lidar : MonoBehaviour {

    /// <summary>
    /// Color to code a ray that hits another object.
    /// </summary>
    [SerializeField]
    [Header("Debug Gizmos", order = 999)]
    internal Color _hitColor = Color.red;

    /// <summary>
    /// Color to code a ray that avoid or misses all other objects.
    /// </summary>
    [SerializeField]
    internal Color _missColor = Color.white;

    // The value of the default layers.
    const int _physicsDefaultLayers = -5;
    [SerializeField, FormerlySerializedAs("LaserLayerMask")]
    [Tooltip("Controls which layers the rays can hit.")]
    LayerMask _LaserLayerMask = _physicsDefaultLayers;

    /// <summary>
    /// Controls which layers the rays can hit.
    /// </summary>
    public LayerMask LaserLayerMask
    {
        get => _LaserLayerMask;
        set { _LaserLayerMask = value; }
    }

    [SerializeField, FormerlySerializedAs("LaserLength")]
    [Range(1, 1000)]
    [Tooltip("Length of the rays to cast.")]
    float _LaserLength = 20f;
    public float LaserLength
    {
        get => _LaserLength;
        set { _LaserLength = value; }
    }


    [SerializeField, FormerlySerializedAs("detectableTags")]
    [Tooltip("List of object tags that will trigger a hit of the Laser.")]
    List<string> _detectableTags;
    

    [SerializeField, FormerlySerializedAs("angleOffsets")]
    [Tooltip("Vector of offsets to change orientation of Laser")]
    public Vector3 LaserOrientation;

    /// <summary>
    /// List of tags in the scene to compare against.
    /// Note that this should not be changed at runtime.
    /// </summary>
    public List<string> DetectableTags
    {
        get { return _detectableTags; }
        set { _detectableTags = value; }
    }

    [SerializeField, FormerlySerializedAs("rotationSpeed")]
    [Tooltip("Speed at which lidar spins clockwise")]
    private float rotationSpeed = 45.0f;


    [SerializeField, FormerlySerializedAs("rotationDirection")]
    [Tooltip("Rotation vector applied to GameObject. Default is Vector3.up")]
    private Vector3 rotationDirection = Vector3.up;


    [SerializeField, FormerlySerializedAs("detectionsPerSecond")]
    [Tooltip("Number of laser detections per second")]
    private int numDetections = 1000;


    private LaserOutput lastOutput;
    private Vector3 lastRotation;
    private bool running = false;
    

    public List<Packet> UpdateSensor(bool debug = false, bool normalize = true, int numSamples = 360, float degrees = 360.0f) {
        List<Packet> packets = new List<Packet>();
        List<LaserInput> inputs = GenerateInputScanData(numSamples, degrees);
        List<LaserOutput> scanResults = FullScan(inputs);
        int count = 0;
        foreach (LaserOutput scan in scanResults) {
            if (count > numSamples - 1) {
                return packets;
            }
            Packet packet = new Packet() {
                theta = normalize ? getPlanarRotation(scan.theta) / 360f : getPlanarRotation(scan.theta),
                distance = normalize ? scan.distance : scan.distance * _LaserLength
            };
            packets.Add(packet); 
            count++;
        }

        if (debug) {
            DrawOutput(scanResults.Last());
        }

        return packets;
    }
    void Update()
    {        
        // Rotate the objects transform to new angle location
        float desiredRotation = Time.deltaTime * rotationSpeed;
        
        // Rotation is for introducing non-deterministic angles when calling UpdateSensor(). Also looks cool.
        transform.Rotate(rotationDirection, desiredRotation, Space.World);

        // Lock orientation
        Vector3 currentRotation = transform.eulerAngles;
        currentRotation.x = currentRotation.x * rotationDirection.x;
        currentRotation.y = currentRotation.y * rotationDirection.y;
        currentRotation.z = currentRotation.z * rotationDirection.z;
        transform.eulerAngles = currentRotation;
    }

    private void LateUpdate() {
        
    }

    private void Start() {
        running = true;
    }

    LaserInput GetInput() {
        LaserInput input = new LaserInput();
        input.LaserOrientation = LaserOrientation;
        input.LaserLength = _LaserLength;
        input.DetectableTags = _detectableTags;
        input.LayerMask = LaserLayerMask;
        return input;
    }

    /// <summary>
    /// Evaluate results of a single ray from the LaserInput.
    /// </summary>
    /// <param name="input"></param>
    /// <returns></returns>
    public LaserOutput Perceive( LaserInput input )
    {
        (Vector3 StartPositionWorld, Vector3 EndPositionWorld) = LaserExtents(input);
        Vector3 startPositionWorld = StartPositionWorld;
        Vector3 endPositionWorld = EndPositionWorld;
        Vector3 rayDirection = endPositionWorld - startPositionWorld;
        float scaledLaserLength = rayDirection.magnitude;    
        RaycastHit hit;
        Ray ray = new Ray(startPositionWorld, rayDirection);
        bool castHit = Physics.Raycast(ray, out hit, scaledLaserLength, input.LayerMask);
        GameObject hitObject = castHit ? hit.collider.gameObject : null;

        LaserOutput output = new LaserOutput()
        {
            HasHit = castHit,
            distance = 1.0f,
            HitTaggedObject = false,
            HitClass = -1,
            HitGameObject = hitObject,
            StartPositionWorld = startPositionWorld,
            EndPositionWorld = endPositionWorld,
            theta = input.LaserOrientation,
            LocalPosition = transform.InverseTransformPoint(EndPositionWorld)
        };

        if (castHit)
        {
            // Find the index of the tag of the object that was hit.
            int numTags = input.DetectableTags?.Count ?? 0;
            for (int i = 0; i < numTags; i++)
            {
                bool tagsEqual = false;
                try
                {
                    var tag = input.DetectableTags[i];
                    if (!string.IsNullOrEmpty(tag))
                    {
                        tagsEqual = hitObject.CompareTag(tag);
                    }
                }
                catch (UnityException) { }

                if (tagsEqual)
                {
                    output.HitTaggedObject = true;
                    output.HitClass = i;
                    output.distance = hit.distance / scaledLaserLength;
                    break;
                }
            }
        }


        return output;
    }

    private List<LaserInput> GenerateInputScanData(int numberOfScans, float range) {
        
        List<LaserInput> inputs = new List<LaserInput>();
        float currentRotation = getPlanarRotation(transform.localRotation.eulerAngles);
        
        float step = range / (float)(numberOfScans);
        for (float angle = currentRotation; angle <= (currentRotation + range); angle += step) {
            LaserInput input = GetInput();
            input.LaserOrientation = (rotationDirection * (angle % 360f)) + LaserOrientation;
            inputs.Add(input);
        }

        return inputs;
    }

    // <summary>
    // Evaluates the Laser along a scan path
    // </summary>
    // <param name="inputs">List of inputs defining Laser cast.</param>
    // <returns>Output struct containing the Laser hit results.</returns>
    private List<LaserOutput> FullScan(List<LaserInput> inputs) {
        List<LaserOutput> outputs = new List<LaserOutput>();
        foreach (LaserInput input in inputs) {
            outputs.Add(Perceive(input));
        } 
        return outputs;
    }

    // <summary>
    // Get the current planar rotation based on rotation direction
    // </summary>
    // <returns>The current orientation along the configured plane of rotation </returns>
    public float getPlanarRotation(Vector3 rotation) {
        float currentPlanarRotation = 0.0f;
        if (rotationDirection.y > 0.0f) {
            currentPlanarRotation = rotation.y;
        } else if (rotationDirection.x > 0.0f) {
            currentPlanarRotation = rotation.x;
        } else {
            currentPlanarRotation = rotation.z;
        }

        return currentPlanarRotation;
    }

    void OnDrawGizmosSelected()
    {
        if (!running) {
            LaserInput LaserInput = GetInput();
            LaserOutput output = Perceive(LaserInput);
            DrawRaycastGizmos(output);
        } else {
            LaserInput LaserInput = GetInput();
            LaserOutput output = Perceive(LaserInput);
            DrawOutput(output);
        }
    }

    /// <summary>
    /// Draw the debug information from the sensor (if available).
    /// </summary>
    void DrawRaycastGizmos(LaserOutput output)
    {
        Vector3 startPositionWorld = output.StartPositionWorld;
        Vector3 endPositionWorld = output.EndPositionWorld;
        Vector3 rayDirection = endPositionWorld - startPositionWorld;
        rayDirection *= output.distance;

        if (output.HasHit) {
            Gizmos.color = _hitColor;
        } else {
            Gizmos.color = _missColor;
        }
        
        Gizmos.DrawRay(startPositionWorld, rayDirection);
    }

    void DrawOutput(LaserOutput output)
    {
        Vector3 startPositionWorld = output.StartPositionWorld;
        Vector3 endPositionWorld = output.EndPositionWorld;
        Vector3 rayDirection = endPositionWorld - startPositionWorld;
        rayDirection *= output.distance;

        if (output.HasHit) {
            Debug.DrawRay(startPositionWorld, rayDirection, _hitColor, .05f);
        } else {
            Debug.DrawRay(startPositionWorld, rayDirection, _missColor, .05f);
        }
    }
 
    public Vector3 RotatePivot(Vector3 vector, Quaternion rotation, Vector3 pivot = default(Vector3)) 
    {
        return rotation * (vector - pivot) + pivot;
    }

    /// <summary>
    /// Get the cast start and end points for the Laser
    /// </summary>
    /// <returns>A tuple of the start and end positions in world space.</returns>
    public (Vector3 StartPositionWorld, Vector3 EndPositionWorld) LaserExtents(LaserInput input)
    {
        Vector3 currentGlobalPosition = transform.position;
        Vector3 globalEndPosition = transform.TransformPoint(Vector3.forward * input.LaserLength);
        Vector3 endPositionGlobal = RotatePivot(globalEndPosition, Quaternion.Euler(input.LaserOrientation), currentGlobalPosition);

        return (
            StartPositionWorld: currentGlobalPosition,
            EndPositionWorld: endPositionGlobal
        );
    }
}


