


// using System;
// // using System.Collections;
// using System.Collections.Generic;
// using System.Linq;
// using UnityEngine;
// using UnityEngine.Serialization;
// public struct LazarInput
// {
//     /// <summary>
//     /// Lazar beam length
//     /// </summary>
//     public float LazarLength;

//     /// <summary>
//     /// Detectable Object Types
//     /// </summary>
//     public IReadOnlyList<string> DetectableTags;

//     /// <summary>
//     /// Orientation relative to gameobject
//     /// </summary>
//     public Vector2 LazerOrientation;

//     /// <summary>
//     /// Transform of the GameObject.
//     /// </summary>
//     public Transform Transform;

//     /// <summary>
//     /// Filtering options for the casts.
//     /// </summary>
//     public int LayerMask;

//     /// <summary>
//     /// Returns the expected number of floats in the output.
//     /// </summary>
//     /// <returns></returns>
//     public int OutputSize()
//     {
//         return ((DetectableTags?.Count ?? 0) + 2);
//     }


//     /// <summary>
//     /// Get the cast start and end points for the Lazar/
//     /// </summary>
//     /// <returns>A tuple of the start and end positions in world space.</returns>
//     public (Vector3 StartPositionWorld, Vector3 EndPositionWorld) LazarExtents()
//     {
//         Vector3 startPositionLocal = new Vector2();
//         Vector3 endPositionLocal = PolarToCartesian(LazarLength, LazerOrientation);
//         return (
//             StartPositionWorld: Transform.TransformPoint(startPositionLocal), 
//             EndPositionWorld: Transform.TransformPoint(endPositionLocal)
//         );
//     }


//     /// <summary>
//     /// Converts polar coordinate to cartesian coordinate.
//     /// </summary>
//     static internal Vector2 PolarToCartesian(float radius, Vector2 angleDegrees)
//     {
//         var x = radius * Mathf.Cos(Mathf.Deg2Rad * angleDegrees.x);
//         var y = radius * Mathf.Sin(Mathf.Deg2Rad * angleDegrees.y);
//         return new Vector2(x, y);
//     }
// }

// public struct LazarOutput
// {

//     /// <summary>
//     /// rotation relative to parent object
//     /// </summary>
//     public Vector3 localRotation;

//     /// <summary>
//     /// Has Lazar Hit something
//     /// </summary>
//     public bool HasHit;

//     /// <summary>
//     /// Is the hit object in the Tagged objects list
//     /// </summary>
//     public bool HitTaggedObject;

//     /// <summary>
//     /// The index of the hit object's tag in the DetectableTags list
//     /// -1 if there was no hit
//     /// </summary>
//     public int HitTagIndex;

//     /// <summary>
//     /// Normalized distance to the hit object.
//     /// </summary>
//     public float HitFraction;

//     /// <summary>
//     /// The hit GameObject (or null if there was no hit).
//     /// </summary>
//     public GameObject HitGameObject;

//     /// <summary>
//     /// Start position of the Lazar in world space.
//     /// </summary>
//     public Vector3 StartPositionWorld;

//     /// <summary>
//     /// End position of the ray in world space.
//     /// </summary>
//     public Vector3 EndPositionWorld;

//     /// <summary>
//     /// Angle of object relative to parent (along rotation 'x' axis)
//     /// </summary>
//     public float CurrentAngle;

//     /// <summary>
//     /// The scaled length of the ray.
//     /// </summary>
//     /// <remarks>
//     /// If there is non-(1,1,1) scale, |EndPositionWorld - StartPositionWorld| will be different from
//     /// the input LazarLength.
//     /// </remarks>
//     public float ScaledLazarLength
//     {
//         get
//         {
//             var rayDirection = EndPositionWorld - StartPositionWorld;
//             return rayDirection.magnitude;
//         }
//     }

//     /// <param name="numDetectableTags"></param>
//     /// <param name="rayIndex"></param>
//     /// <param name="buffer">Output buffer. The size must be equal to (numDetectableTags+2) * RayOutputs.Length</param>
//     public void ToFloatArray(int numDetectableTags, float[] buffer)
//     {
//         for (int i = 0; i < numDetectableTags; i++) {   
//             buffer[i] = 0.0f;
//         }
//         if (HitTaggedObject)
//         {
//             buffer[HitTagIndex] = 1f;
//         }
//         buffer[numDetectableTags] = HasHit ? 0f : 1f;
//         buffer[numDetectableTags + 1] = HitFraction;
//     }
    
// }

// public class Lidar : MonoBehaviour {

//     /// <summary>
//     /// Color to code a ray that hits another object.
//     /// </summary>
//     [SerializeField]
//     [Header("Debug Gizmos", order = 999)]
//     internal Color _hitColor = Color.red;

//     /// <summary>
//     /// Color to code a ray that avoid or misses all other objects.
//     /// </summary>
//     [SerializeField]
//     internal Color _missColor = Color.white;

//     // The value of the default layers.
//     const int _physicsDefaultLayers = -5;
//     [SerializeField, FormerlySerializedAs("LazarLayerMask")]
//     [Tooltip("Controls which layers the rays can hit.")]
//     LayerMask _lazarLayerMask = _physicsDefaultLayers;

//     /// <summary>
//     /// Controls which layers the rays can hit.
//     /// </summary>
//     public LayerMask LazarLayerMask
//     {
//         get => _lazarLayerMask;
//         set { _lazarLayerMask = value; }
//     }

//     [SerializeField, FormerlySerializedAs("LazarLength")]
//     [Range(1, 1000)]
//     [Tooltip("Length of the rays to cast.")]
//     float _lazarLength = 20f;


//     [SerializeField, FormerlySerializedAs("detectableTags")]
//     [Tooltip("List of object tags that will trigger a hit of the lazar.")]
//     List<string> _detectableTags;
    

//     [SerializeField, FormerlySerializedAs("angleOffsets")]
//     [Tooltip("Vector of offsets to change orientation of lazar")]
//     public Vector2 LazerOrientation;

//     /// <summary>
//     /// List of tags in the scene to compare against.
//     /// Note that this should not be changed at runtime.
//     /// </summary>
//     public List<string> DetectableTags
//     {
//         get { return _detectableTags; }
//         set { _detectableTags = value; }
//     }

//     [SerializeField, FormerlySerializedAs("rotationSpeed")]
//     [Tooltip("Speed at which lidar spins clockwise")]
//     private float rotationSpeed = 360.0f;


//     [SerializeField, FormerlySerializedAs("rotationDirection")]
//     [Tooltip("Rotation vector applied to GameObject. Default is Vector3.up")]
//     private Vector3 rotationDirection = Vector3.up;

    
//     void FixedUpdate()
//     {
//         transform.Rotate(rotationDirection, Time.deltaTime * rotationSpeed, Space.World);
//     }

//     private void LateUpdate() {
        
//     }

//     LazarInput GetInput() {
//         LazarInput input = new LazarInput();
//         input.Transform = transform;
//         input.LazerOrientation = LazerOrientation;
//         input.LazarLength = _lazarLength;
//         input.DetectableTags = _detectableTags;
//         input.LayerMask = LazarLayerMask;
//         return input;
//     }

//     /// <summary>
//     /// Evaluate results of a single ray from the LazarInput.
//     /// </summary>
//     /// <param name="input"></param>
//     /// <returns></returns>
//     internal static LazarOutput Perceive( LazarInput input )
//     {
//         float unscaledLazarLength = input.LazarLength;
//         (Vector3 StartPositionWorld, Vector3 EndPositionWorld) = input.LazarExtents();
//         Vector3 startPositionWorld = StartPositionWorld;
//         Vector3 endPositionWorld = EndPositionWorld;
//         Vector3 rayDirection = endPositionWorld - startPositionWorld;
//         float scaledLazarLength = rayDirection.magnitude;    
//         RaycastHit hit;
//         Ray ray = new Ray(startPositionWorld, rayDirection);
//         bool castHit = Physics.Raycast(ray, out hit, scaledLazarLength, input.LayerMask);
//         GameObject hitObject = castHit ? hit.collider.gameObject : null;

//         LazarOutput output = new LazarOutput()
//         {
//             HasHit = castHit,
//             HitFraction = 1.0f,
//             HitTaggedObject = false,
//             HitTagIndex = -1,
//             HitGameObject = hitObject,
//             StartPositionWorld = startPositionWorld,
//             EndPositionWorld = endPositionWorld,
//             CurrentAngle = input.Transform.localEulerAngles.x
//         };

//         if (castHit)
//         {
//             // Find the index of the tag of the object that was hit.
//             int numTags = input.DetectableTags?.Count ?? 0;
//             for (int i = 0; i < numTags; i++)
//             {
//                 bool tagsEqual = false;
//                 try
//                 {
//                     var tag = input.DetectableTags[i];
//                     if (!string.IsNullOrEmpty(tag))
//                     {
//                         tagsEqual = hitObject.CompareTag(tag);
//                     }
//                 }
//                 catch (UnityException) { }

//                 if (tagsEqual)
//                 {
//                     output.HitTaggedObject = true;
//                     output.HitTagIndex = i;
//                     output.HitFraction = hit.distance / scaledLazarLength;
//                     break;
//                 }
//             }
//         }


//         return output;
//     }

//     void OnDrawGizmosSelected()
//     {
//         LazarInput lazarInput = GetInput();
//         LazarOutput output = Perceive(lazarInput);
//         DrawRaycastGizmos(output);
//     }

//     /// <summary>
//     /// Draw the debug information from the sensor (if available).
//     /// </summary>
//     void DrawRaycastGizmos(LazarOutput output)
//     {
//         Vector3 startPositionWorld = output.StartPositionWorld;
//         Vector3 endPositionWorld = output.EndPositionWorld;
//         Vector3 rayDirection = endPositionWorld - startPositionWorld;
//         rayDirection *= output.HitFraction;

//         if (output.HasHit) {
//             Gizmos.color = _hitColor;
//         } else {
//             Gizmos.color = _missColor;
//         }
        
//         Gizmos.DrawRay(startPositionWorld, rayDirection);
//     }
// }


// // List<LazarInput> GenerateInputScanData(int numberOfScans, float range) {

//     //     List<LazarInput> inputs = new List<LazarInput>();
//     //     List<float> angles = GenerateAngles(_currentAngle, _currentAngle + range, numberOfScans);

//     //     foreach (float angle in angles) {
//     //         LazarInput input = new LazarInput();
//     //         Vector3 localOrientation = transform.localEulerAngles;
//     //         localOrientation.x = angle;
//     //         input.Transform.localEulerAngles = localOrientation;
            
//     //     }

//     //     _currentAngle = _currentAngle + range;
//     //     return inputs;
//     // }

//     /// <summary>
//     /// Evaluates the lazar along a scan path
//     /// </summary>
//     /// <param name="inputs">List of inputs defining lazar cast.</param>
//     /// <returns>Output struct containing the lazar hit results.</returns>
//     // public static List<LazarOutput> FullScan(List<LazarInput> inputs) {
//     //     List<LazarOutput> outputs = new List<LazarOutput>();
//     //     foreach (LazarInput input in inputs) {
//     //         outputs.Add(Perceive(input));
//     //     } 
//     //     return outputs;
//     // }

//     // List<float> GenerateAngles(float min, float max, int steps) {
//     //     if (steps <= 1) {
//     //         return (List<float>)Enumerable.Repeat(min, 1);
//     //     } else {
//     //         return (List<float>)Enumerable.Range(0, steps).Select(i => min + (max - min) * ((float)i / (steps - 1)));
//     //     }
//     // }
