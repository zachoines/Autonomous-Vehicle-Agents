using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.MLAgents.Sensors;

/// <summary>
/// 360 RPLidar sensor implementation
/// Shamelessly Based off MLAgents RayPerceptionSensorComponentBase class
/// </summary>

namespace Unity.MLAgents.Sensors
{
    [AddComponentMenu("MLAgents Lidar Sensor", 0)]
    public class RPLidar : SensorComponent
    {
        [HideInInspector, SerializeField, FormerlySerializedAs("sensorName")]
        string _sensorName = "RPLidar";

        /// <summary>
        /// The name of the Sensor that this component wraps.
        /// Note that changing this at runtime does not affect how the Agent sorts the sensors.
        /// </summary>
        public string SensorName
        {
            get { return _sensorName; }
            set { _sensorName = value; }
        }

        [SerializeField, FormerlySerializedAs("detectableTags")]
        [Tooltip("List of object tags that will trigger a hit of the lazar.")]
        List<string> _detectableTags;

        /// <summary>
        /// List of tags in the scene to compare against.
        /// Note that this should not be changed at runtime.
        /// </summary>
        public List<string> DetectableTags
        {
            get { return _detectableTags; }
            set { _detectableTags = value; }
        }

        [SerializeField, FormerlySerializedAs("LazarLength")]
        [Range(1, 1000)]
        [Tooltip("Length of the rays to cast.")]
        float _lazarLength = 20f;

        /// <summary>
        /// Length of the rays to cast.
        /// </summary>
        public float LazarLength
        {
            get => _lazarLength;
            set { _lazarLength = value; UpdateSensor(); }
        }

        [SerializeField, FormerlySerializedAs("LazarLength")]
        [Range(1, 1000)]
        [Tooltip("Length of the rays to cast.")]
        float _currentAngle = 20f;

        /// <summary>
        /// Length of the rays to cast.
        /// </summary>
        public float CurrentAngle
        {
            get => _currentAngle;
            set { _currentAngle = value; UpdateSensor(); }
        }

        // The value of the default layers.
        const int _physicsDefaultLayers = -5;
        [SerializeField, FormerlySerializedAs("LazarLayerMask")]
        [Tooltip("Controls which layers the rays can hit.")]
        LayerMask _lazarLayerMask = _physicsDefaultLayers;

        /// <summary>
        /// Controls which layers the rays can hit.
        /// </summary>
        public LayerMask LazarLayerMask
        {
            get => _lazarLayerMask;
            set { _lazarLayerMask = value; UpdateSensor(); }
        }

        [SerializeField, FormerlySerializedAs("observationStacks")]
        [Range(1, 50)]
        [Tooltip("Number of raycast results that will be stacked before being fed to the neural network.")]
        int _observationStacks = 1;

        /// <summary>
        /// Whether to stack previous observations. Using 1 means no previous observations.
        /// Note that changing this after the sensor is created has no effect.
        /// </summary>
        public int ObservationStacks
        {
            get { return _observationStacks; }
            set { _observationStacks = value; }
        }

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

        [NonSerialized]
        LazarSensor _lazarSensor;

        /// <summary>
        /// Get the Lazar Sensor that was created.
        /// </summary>
        public LazarSensor LazarSensor
        {
            get => _lazarSensor;
        }

        /// <summary>
        /// Returns the amount that the ray start is offset up or down by.
        /// </summary>
        /// <returns></returns>
        public virtual float GetStartVerticalOffset()
        {
            return 0f;
        }

        /// <summary>
        /// Returns the amount that the ray end is offset up or down by.
        /// </summary>
        /// <returns></returns>
        public virtual float GetEndVerticalOffset()
        {
            return 0f;
        }

        /// <summary>
        /// Returns an initialized raycast sensor.
        /// </summary>
        /// <returns></returns>
        public override ISensor[] CreateSensors()
        {
            _lazarSensor = new LazarSensor(_sensorName, GetLazarSensorInput());

            if (ObservationStacks > 1)
            {
                var stackingSensor = new StackingSensor(_lazarSensor, ObservationStacks);
                return new ISensor[] { stackingSensor };
            }

            return new ISensor[] { _lazarSensor };
        }

        /// <summary>
        /// Get the LazarSensorInput
        /// </summary>
        /// <returns></returns>
        public LazarInput GetLazarSensorInput()
        {
            var rayPerceptionInput = new LazarInput();
            rayPerceptionInput.LazarLength = LazarLength;
            rayPerceptionInput.DetectableTags = DetectableTags;
            rayPerceptionInput.CurrentAngle = CurrentAngle;
            rayPerceptionInput.StartOffset = GetStartVerticalOffset();
            rayPerceptionInput.EndOffset = GetEndVerticalOffset();
            rayPerceptionInput.Transform = transform;
            rayPerceptionInput.LayerMask = LazarLayerMask;

            return rayPerceptionInput;
        }

        internal void UpdateSensor()
        {
            _lazarSensor?.setLazarInput(GetLazarSensorInput());
        }

        internal int SensorObservationAge()
        {
            if (_lazarSensor != null)
            {
                return Time.frameCount - _lazarSensor.DebugLastFrameCount;
            }

            return 0;
        }

        void OnDrawGizmosSelected()
        {
            if (_lazarSensor?.CurrentLazarOutput != null)
            {

                var alpha = Mathf.Pow(.5f, SensorObservationAge());
                DrawRaycastGizmos(_lazarSensor.CurrentLazarOutput, alpha);
    
            }
            else
            {
                var rayInput = GetLazarSensorInput();
                rayInput.DetectableTags = null;

                LazarOutput output = LazarSensor.PerceiveSingleRay(rayInput);
                DrawRaycastGizmos(output);
            }
        }

        /// <summary>
        /// Draw the debug information from the sensor (if available).
        /// </summary>
        void DrawRaycastGizmos(LazarOutput output, float alpha = 1.0f)
        {
            var startPositionWorld = output.StartPositionWorld;
            var endPositionWorld = output.EndPositionWorld;
            var rayDirection = endPositionWorld - startPositionWorld;
            rayDirection *= output.HitFraction;

            // hit fraction ^2 will shift "far" hits closer to the hit color
            var lerpT = output.HitFraction * output.HitFraction;
            var color = Color.Lerp(_hitColor, _missColor, lerpT);
            color.a *= alpha;
            Gizmos.color = color;
            Gizmos.DrawRay(startPositionWorld, rayDirection);

            // Draw the hit point as a sphere. If using rays to cast (0 radius), use a small sphere.
            if (output.HasHit)
            {
                var hitRadius = Mathf.Max(output.ScaledCastRadius, .05f);
                Gizmos.DrawWireSphere(startPositionWorld + rayDirection, hitRadius);
            }
        }
    }
}