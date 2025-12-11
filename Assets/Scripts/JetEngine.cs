using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class JetEngine : MonoBehaviour
{   
    [SerializeField] private Transform _nozzle; //сопло двигателя
    [SerializeField] private InputActionAsset _actionAsset;
    
    [Header("Тяга")] 
    [SerializeField] private float _thrustDrySL = 79000f;//сухой режим
    [SerializeField] private float _thrustABSL = 129000f;//форсаж режим

    //скорость изменения РУД - рычаг упарвления двигателя
    [SerializeField] private float _throttleRate = 1.0f; 
    [SerializeField] private float _throttleStep = 0.05f; // шаг изменения по X/Z
    
    private Rigidbody _rigidbody;
    
    //текущее состояние двигателя
    private float _throttle01;//0..1
    private bool _afterBurner; //AB on/off
    
    private float _speedMS;
    private float _lastAppliedThrust;

    //input
    private InputAction _throttleUpHold;//shift
    private InputAction _throttleDownHold;//LCtrl
    private InputAction _throttleStepUp; //шаг по X
    private InputAction _throttleStepDown; //шаг по Y
    private InputAction _toggleAB; //LAlt
    
    private void Awake()
    {
        _rigidbody = GetComponent<Rigidbody>();

        _throttle01 = 0.0f;
        _afterBurner = false;

        InitializeActions();
    }

    private void InitializeActions()
    {
        var map = _actionAsset.FindActionMap("JetEngine");
        
        _throttleUpHold = map.FindAction("ThrottleUp");
        _throttleDownHold = map.FindAction("ThrottleDown");
        _throttleStepUp = map.FindAction("ThrottleStepUp");
        _throttleStepDown = map.FindAction("ThrottleStepDown");
        _toggleAB = map.FindAction("ToggleAB");

        _throttleStepUp.performed += _ => AdjustThrottle(+_throttleStep);
        _throttleStepDown.performed += _ => AdjustThrottle(-_throttleStep);
        _toggleAB.performed += _ => { _afterBurner = !_afterBurner; };
    }

    private void AdjustThrottle(float delta)
    {
        _throttle01 = Mathf.Clamp01(_throttle01 * delta);
    }

    private void OnEnable()
    {
        _throttleUpHold.Enable();
        _throttleDownHold.Enable();
        _throttleStepUp.Enable();
        _throttleStepDown.Enable();
        _toggleAB.Enable();
    }
    
    private void OnDisable()
    {
        _throttleUpHold.Disable();
        _throttleDownHold.Disable();
        _throttleStepUp.Disable();
        _throttleStepDown.Disable();
        _toggleAB.Disable();
    }

    private void FixedUpdate()
    {
        _speedMS = _rigidbody.linearVelocity.magnitude;
        
        //плавное изменение РУД по удержанию
        float dt = Time.fixedDeltaTime;
        
        if (_throttleUpHold.IsPressed())
            _throttle01 = Mathf.Clamp01(_throttle01 + _throttleRate * dt);

        if (_throttleDownHold.IsPressed())
            _throttle01 = Mathf.Clamp01(_throttle01 - _throttleRate * dt);
        
        //расчёт тяги (без поправок высокт/скорости)
        float throttle = _throttle01 * (_afterBurner?  _thrustABSL: _thrustDrySL);
        _lastAppliedThrust = throttle;
      
        Vector3 force = _nozzle.forward * throttle;
        _rigidbody.AddForceAtPosition(force,_nozzle.position, ForceMode.Force);
    }
}
