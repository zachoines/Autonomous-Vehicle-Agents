// GENERATED AUTOMATICALLY FROM 'Assets/Scripts/PlayerControls.inputactions'

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Utilities;

public class @PlayerControls : IInputActionCollection, IDisposable
{
    public InputActionAsset asset { get; }
    public @PlayerControls()
    {
        asset = InputActionAsset.FromJson(@"{
    ""name"": ""PlayerControls"",
    ""maps"": [
        {
            ""name"": ""Robot"",
            ""id"": ""9d4a5d2c-cc73-42bd-b9c1-d4cb0a1d24a5"",
            ""actions"": [
                {
                    ""name"": ""Right"",
                    ""type"": ""Value"",
                    ""id"": ""a6241936-cd48-4502-a7f2-df1b6af4890c"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """"
                },
                {
                    ""name"": ""Left"",
                    ""type"": ""Value"",
                    ""id"": ""6bb1fb69-40f0-415f-a7b8-7f23b13410bb"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """"
                },
                {
                    ""name"": ""Switch"",
                    ""type"": ""Value"",
                    ""id"": ""ce885c8b-0903-471c-b4a7-08f0a1ca65ae"",
                    ""expectedControlType"": ""Double"",
                    ""processors"": """",
                    ""interactions"": """"
                }
            ],
            ""bindings"": [
                {
                    ""name"": """",
                    ""id"": ""a6360fbb-dae7-404a-82a1-00e87d3b1efd"",
                    ""path"": ""<Gamepad>/rightStick/"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Right"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": """",
                    ""id"": ""d51ab291-6ebb-4bb6-a83b-2c3496aeac97"",
                    ""path"": ""<Gamepad>/leftStick/"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Left"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": """",
                    ""id"": ""33e1ff6e-9c34-437c-9e00-03b9052eceb9"",
                    ""path"": ""<Gamepad>/rightTrigger"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Switch"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                }
            ]
        }
    ],
    ""controlSchemes"": []
}");
        // Robot
        m_Robot = asset.FindActionMap("Robot", throwIfNotFound: true);
        m_Robot_Right = m_Robot.FindAction("Right", throwIfNotFound: true);
        m_Robot_Left = m_Robot.FindAction("Left", throwIfNotFound: true);
        m_Robot_Switch = m_Robot.FindAction("Switch", throwIfNotFound: true);
    }

    public void Dispose()
    {
        UnityEngine.Object.Destroy(asset);
    }

    public InputBinding? bindingMask
    {
        get => asset.bindingMask;
        set => asset.bindingMask = value;
    }

    public ReadOnlyArray<InputDevice>? devices
    {
        get => asset.devices;
        set => asset.devices = value;
    }

    public ReadOnlyArray<InputControlScheme> controlSchemes => asset.controlSchemes;

    public bool Contains(InputAction action)
    {
        return asset.Contains(action);
    }

    public IEnumerator<InputAction> GetEnumerator()
    {
        return asset.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    public void Enable()
    {
        asset.Enable();
    }

    public void Disable()
    {
        asset.Disable();
    }

    // Robot
    private readonly InputActionMap m_Robot;
    private IRobotActions m_RobotActionsCallbackInterface;
    private readonly InputAction m_Robot_Right;
    private readonly InputAction m_Robot_Left;
    private readonly InputAction m_Robot_Switch;
    public struct RobotActions
    {
        private @PlayerControls m_Wrapper;
        public RobotActions(@PlayerControls wrapper) { m_Wrapper = wrapper; }
        public InputAction @Right => m_Wrapper.m_Robot_Right;
        public InputAction @Left => m_Wrapper.m_Robot_Left;
        public InputAction @Switch => m_Wrapper.m_Robot_Switch;
        public InputActionMap Get() { return m_Wrapper.m_Robot; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(RobotActions set) { return set.Get(); }
        public void SetCallbacks(IRobotActions instance)
        {
            if (m_Wrapper.m_RobotActionsCallbackInterface != null)
            {
                @Right.started -= m_Wrapper.m_RobotActionsCallbackInterface.OnRight;
                @Right.performed -= m_Wrapper.m_RobotActionsCallbackInterface.OnRight;
                @Right.canceled -= m_Wrapper.m_RobotActionsCallbackInterface.OnRight;
                @Left.started -= m_Wrapper.m_RobotActionsCallbackInterface.OnLeft;
                @Left.performed -= m_Wrapper.m_RobotActionsCallbackInterface.OnLeft;
                @Left.canceled -= m_Wrapper.m_RobotActionsCallbackInterface.OnLeft;
                @Switch.started -= m_Wrapper.m_RobotActionsCallbackInterface.OnSwitch;
                @Switch.performed -= m_Wrapper.m_RobotActionsCallbackInterface.OnSwitch;
                @Switch.canceled -= m_Wrapper.m_RobotActionsCallbackInterface.OnSwitch;
            }
            m_Wrapper.m_RobotActionsCallbackInterface = instance;
            if (instance != null)
            {
                @Right.started += instance.OnRight;
                @Right.performed += instance.OnRight;
                @Right.canceled += instance.OnRight;
                @Left.started += instance.OnLeft;
                @Left.performed += instance.OnLeft;
                @Left.canceled += instance.OnLeft;
                @Switch.started += instance.OnSwitch;
                @Switch.performed += instance.OnSwitch;
                @Switch.canceled += instance.OnSwitch;
            }
        }
    }
    public RobotActions @Robot => new RobotActions(this);
    public interface IRobotActions
    {
        void OnRight(InputAction.CallbackContext context);
        void OnLeft(InputAction.CallbackContext context);
        void OnSwitch(InputAction.CallbackContext context);
    }
}
