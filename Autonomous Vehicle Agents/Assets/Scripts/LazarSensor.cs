using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents.Sensors;
public struct LazarInput
{
    /// <summary>
    /// Lazar beam length
    /// </summary>
    public float LazarLength;

    /// <summary>
    /// Detectable Object Types
    /// </summary>
    public IReadOnlyList<string> DetectableTags;


    /// <summary>
    /// Starting height offset of lazar from center of agent
    /// </summary>
    public float StartOffset;

    /// <summary>
    /// Ending height offset of lazar from center of agent.
    /// </summary>
    public float EndOffset;

    /// <summary>
    /// Transform of the GameObject.
    /// </summary>
    public Transform Transform;

    /// <summary>
    /// Angle of object relative to parent
    /// </summary>
    public float CurrentAngle;

    /// <summary>
    /// Filtering options for the casts.
    /// </summary>
    public int LayerMask;

    /// <summary>
    /// Returns the expected number of floats in the output.
    /// </summary>
    /// <returns></returns>
    public int OutputSize()
    {
        return ((DetectableTags?.Count ?? 0) + 2);
    }


    /// <summary>
    /// Get the cast start and end points for the Lazer/
    /// </summary>
    /// <returns>A tuple of the start and end positions in world space.</returns>
    public (Vector3 StartPositionWorld, Vector3 EndPositionWorld) LazarExtents()
    {
        Vector3 startPositionLocal = new Vector2();
        Vector3 endPositionLocal = PolarToCartesian(LazarLength, CurrentAngle);

        var startPositionWorld = Transform.TransformPoint(startPositionLocal);
        var endPositionWorld = Transform.TransformPoint(endPositionLocal);

        return (StartPositionWorld: startPositionWorld, EndPositionWorld: endPositionWorld);
    }


    /// <summary>
    /// Converts polar coordinate to cartesian coordinate.
    /// </summary>
    static internal Vector2 PolarToCartesian(float radius, float angleDegrees)
    {
        var x = radius * Mathf.Cos(Mathf.Deg2Rad * angleDegrees);
        var y = radius * Mathf.Sin(Mathf.Deg2Rad * angleDegrees);
        return new Vector2(x, y);
    }
}

public struct LazarOutput
{

    /// <summary>
    /// Has Lazar Hit something
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
    public int HitTagIndex;

    /// <summary>
    /// Normalized distance to the hit object.
    /// </summary>
    public float HitFraction;

    /// <summary>
    /// The hit GameObject (or null if there was no hit).
    /// </summary>
    public GameObject HitGameObject;

    /// <summary>
    /// Start position of the Lazar in world space.
    /// </summary>
    public Vector3 StartPositionWorld;

    /// <summary>
    /// End position of the ray in world space.
    /// </summary>
    public Vector3 EndPositionWorld;

    /// <summary>
    /// The scaled length of the ray.
    /// </summary>
    /// <remarks>
    /// If there is non-(1,1,1) scale, |EndPositionWorld - StartPositionWorld| will be different from
    /// the input LazarLength.
    /// </remarks>
    public float ScaledLazarLength
    {
        get
        {
            var rayDirection = EndPositionWorld - StartPositionWorld;
            return rayDirection.magnitude;
        }
    }

    /// <summary>
    /// The scaled size of the cast.
    /// </summary>
    /// <remarks>
    /// If there is non-(1,1,1) scale, the cast radius will be also be scaled.
    /// </remarks>
    public float ScaledCastRadius;


    /// <param name="numDetectableTags"></param>
    /// <param name="rayIndex"></param>
    /// <param name="buffer">Output buffer. The size must be equal to (numDetectableTags+2) * RayOutputs.Length</param>
    public void ToFloatArray(int numDetectableTags, float[] buffer)
    {
        for (int i = 0; i < numDetectableTags; i++) {   
            buffer[i] = 0.0f;
        }
        if (HitTaggedObject)
        {
            buffer[HitTagIndex] = 1f;
        }
        buffer[numDetectableTags] = HasHit ? 0f : 1f;
        buffer[numDetectableTags + 1] = HitFraction;
    }
}

public class LazarSensor : ISensor {

  
    float[] _observations;
    ObservationSpec _observationSpec;
    string _name;

    LazarInput _lazarInput;
    LazarOutput _lazarOutput;

    public LazarOutput CurrentLazarOutput
    {
        get { return _lazarOutput; }
    }

    /// <summary>
    /// Time.frameCount at the last time Update() was called. This is only used for display in gizmos.
    /// </summary>
    int _debugLastFrameCount;

    internal int DebugLastFrameCount
    {
        get { return _debugLastFrameCount; }
    }

    /// <summary>
    /// Creates a LazarSensor.
    /// </summary>
    /// <param name="name">The name of the sensor.</param>
    /// <param name="input">The inputs for the lazar sensor.</param>
    public LazarSensor(string name, LazarInput input)
    {
        _name = name;
        _lazarInput = input;

        SetNumObservations(input.OutputSize());

        _debugLastFrameCount = Time.frameCount;
        _lazarOutput = new LazarOutput();
    }

    void SetNumObservations(int numObservations)
    {
        _observationSpec = ObservationSpec.Vector(numObservations);
        _observations = new float[numObservations];
    }


    internal void setLazarInput(LazarInput input) {
        _lazarInput = input;
    }

    /// <summary>
    /// Computes the ray perception observations and saves them to the provided
    /// <see cref="ObservationWriter"/>.
    /// </summary>
    /// <param name="writer">Where the ray perception observations are written to.</param>
    /// <returns></returns>
    public int Write(ObservationWriter writer)
    {
  
        Array.Clear(_observations, 0, _observations.Length);
        var numDetectableTags = _lazarInput.DetectableTags.Count;

        // For each ray, write the information to the observation buffer
        _lazarOutput.ToFloatArray(numDetectableTags, _observations);

        // Finally, add the observations to the ObservationWriter
        writer.AddList(_observations);

        return _observations.Length;
    }

    /// <inheritdoc/>
    public void Update()
    {
        _debugLastFrameCount = Time.frameCount;

        _lazarOutput = new LazarOutput();

        _lazarOutput = PerceiveSingleRay(_lazarInput);
    }

    /// <inheritdoc/>
    public void Reset() { }

    /// <inheritdoc/>
    public ObservationSpec GetObservationSpec()
    {
        return _observationSpec;
    }

    /// <inheritdoc/>
    public string GetName()
    {
        return _name;
    }

    /// <inheritdoc/>
    public virtual byte[] GetCompressedObservation()
    {
        return null;
    }

    /// <inheritdoc/>
    public CompressionSpec GetCompressionSpec()
    {
        return CompressionSpec.Default();
    }

    /// <inheritdoc/>
    public BuiltInSensorType GetBuiltInSensorType()
    {
        return BuiltInSensorType.RayPerceptionSensor;
    }

    /// <summary>
    /// Evaluates the lazar
    /// </summary>
    /// <param name="input">Input defining lazar cast.</param>
    /// <returns>Output struct containing the lazar hit results.</returns>
    public static LazarOutput Perceive(LazarInput input)
    {
        return PerceiveSingleRay(input);
    }

    /// <summary>
    /// Evaluate the raycast results of a single ray from the LazarInput.
    /// </summary>
    /// <param name="input"></param>
    /// <returns></returns>
    internal static LazarOutput PerceiveSingleRay( LazarInput input )
    {
        var unscaledLazarLength = input.LazarLength;

        var extents = input.LazarExtents();
        var startPositionWorld = extents.StartPositionWorld;
        var endPositionWorld = extents.EndPositionWorld;
        var rayDirection = endPositionWorld - startPositionWorld;
        var scaledLazarLength = rayDirection.magnitude;


        // Do the cast and assign the hit information for each detectable tag.
        var castHit = false;
        var hitFraction = 1.0f;
        GameObject hitObject = null;
        RaycastHit2D rayHit;
        rayHit = Physics2D.Raycast(startPositionWorld, rayDirection, scaledLazarLength, input.LayerMask);
        castHit = rayHit;
        hitFraction = castHit ? rayHit.fraction : 1.0f;
        hitObject = castHit ? rayHit.collider.gameObject : null;
    

        LazarOutput output = new LazarOutput()
        {
            HasHit = castHit,
            HitFraction = hitFraction,
            HitTaggedObject = false,
            HitTagIndex = -1,
            HitGameObject = hitObject,
            StartPositionWorld = startPositionWorld,
            EndPositionWorld = endPositionWorld
        };

        if (castHit)
        {
            // Find the index of the tag of the object that was hit.
            var numTags = input.DetectableTags?.Count ?? 0;
            for (var i = 0; i < numTags; i++)
            {
                var tagsEqual = false;
                try
                {
                    var tag = input.DetectableTags[i];
                    if (!string.IsNullOrEmpty(tag))
                    {
                        tagsEqual = hitObject.CompareTag(tag);
                    }
                }
                catch (UnityException)
                {
                    // If the tag is null, empty, or not a valid tag, just ignore it.
                }

                if (tagsEqual)
                {
                    output.HitTaggedObject = true;
                    output.HitTagIndex = i;
                    break;
                }
            }
        }


        return output;
    }
}

