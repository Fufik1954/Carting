using System;
using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class SimpleDroneController : MonoBehaviour
{
    [Header("Settings")]
    [SerializeField] private float _maxThrust = 30f;
    [SerializeField] private float _thrustResponce = 0.6f;//чуствительность тиги

    [SerializeField] private float _torquePowerRoll = 2.5f;//момент по крену за ед входа
    [SerializeField] private float _torquePowerPitch = 2.5f;//момент по тангажу за ед входа
    [SerializeField] private float _torquePowerYaw = 1.8f;//момент по рысканью за ед входа
    
    [Header("Dampig")]
    [SerializeField] private float _angularDamping = 0.5f;//демпфиромание угловой скорости
    //больше-стабильнее, но "тягучий"
    [SerializeField] private float _linearDamping = 0.2f;//линейный демпфер скорости

    [SerializeField] private InputActionAsset _actionAsset;
    
    private Rigidbody _rigidbody;
    
    private InputAction _throttleUp, _throttleDown; 
    private InputAction _yawRight, _yawLeft; 
    private InputAction _pitchUp, _pitchDown; 
    private InputAction _rollRight, _rollLeft;

    private float _throttleInput;
    private float _yawInput;
    private float _pitchInput;
    private float _rollInput;
    private float _hoverShare;//доля от Tmax для висения mg/Tmax

    private void Awake()
    {
        _rigidbody = GetComponent<Rigidbody>();
        
        _hoverShare = (_rigidbody.mass*Physics.gravity.magnitude)/
                      Mathf.Max(0.01f,_maxThrust);

        InitializeActions();
    }

    private void InitializeActions()
    {
        var map = _actionAsset.FindActionMap("Drone");

        _throttleUp = map.FindAction("ThrottleUp");
        _throttleDown = map.FindAction("ThrottleDown");
        
        _yawRight = map.FindAction("YawRight");
        _yawLeft = map.FindAction("YawLeft");
        
        _pitchUp = map.FindAction("PitchUp");
        _pitchDown = map.FindAction("PitchDown");
        
        _rollRight = map.FindAction("RollRight");
        _rollLeft = map.FindAction("RollLeft");
    }

    private void OnEnable()
    {
        _throttleUp.Enable();
        _throttleDown.Enable();
        _yawRight.Enable();
        _yawLeft.Enable();
        _pitchUp.Enable();
        _pitchDown.Enable();
        _rollRight.Enable();
        _rollLeft.Enable();
    }

    private void OnDisable()
    {
        _throttleUp.Disable();
        _throttleDown.Disable();
        _yawRight.Disable();
        _yawLeft.Disable();
        _pitchUp.Disable();
        _pitchDown.Disable();
        _rollRight.Disable();
        _rollLeft.Disable();
    }

/// ЗАДАЧА - ДОБАВИЬТ ВРАЩЕНИЕ ВИНТОВ. ВРАЩЕНИЕ ДОЛЖНО ЗАВИСЕТЬ ОТ ТЯГИ
    private void Update() => ReadInput();

    private void ReadInput()
    {
        _throttleInput = Boll01(_throttleUp) - Boll01(_throttleDown);
        _yawInput = Boll01(_yawRight) - Boll01(_yawLeft);
        _pitchInput = Boll01(_pitchUp) - Boll01(_pitchDown);
        _rollInput = Boll01(_rollRight) - Boll01(_rollLeft);
    }

    private float Boll01(InputAction action) => action.IsPressed() ? 1 : 0;

    private void FixedUpdate()
    {
        AplyForces();
        ApplyTorque();
        ApplyDamping();
    }

    private void AplyForces()
    {
        //тяга вдоль локальной ВВЕРХ
        float thrustShare = Mathf.Clamp01(_hoverShare + _thrustResponce * _throttleInput);
        float thrust = thrustShare * _maxThrust;
        
        _rigidbody.AddRelativeForce(Vector3.up * thrust, ForceMode.Force);
    }

    private void ApplyTorque()
    {
        Vector3 localTorque = new Vector3(
            _pitchInput * _torquePowerPitch,
            _yawInput * _torquePowerYaw,
            -_rollInput * _torquePowerRoll);
        
        _rigidbody.AddRelativeTorque(localTorque, ForceMode.Force);
        
        //демпфер угловой скоррости t_damp=-k*w
        _rigidbody.AddTorque(-_rigidbody.angularVelocity * _angularDamping, ForceMode.Force);
    }

    private void ApplyDamping()
    {
        //упрощённая аэродинамика F_d = -c*v
        _rigidbody.AddForce(-_rigidbody.linearVelocity*_linearDamping, ForceMode.Force);
    }
}
