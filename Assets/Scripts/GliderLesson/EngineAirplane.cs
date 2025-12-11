using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GliderLesson
{
    [RequireComponent(typeof(Rigidbody))]
    public class EngineAirplane:MonoBehaviour
    {
        [Header("Точка приложения силы")]
        [SerializeField] private Transform _nozzle;

        [SerializeField] private float _thrustDrySL = 79_000f;// сухой режим
        [SerializeField] private float _thrustABSL = 129_000f;// форсаж режим

        [SerializeField] private InputActionAsset _actionAsset;//scheme actions
        private Rigidbody _rigidbody;
        
        //текущее состояние двигателя
        private float _throttle01;//0..1
        private bool _afterBurner; //AB on/off
    
        private float _speedMS;
        private float _lastAppliedThrust;

        //input
        private InputAction _throttleUpHold;//shift
        private InputAction _throttleDownHold;//LCtrl
        private InputAction _toggleAB; //LAlt
//ЗАДАНИЕ РЕАЛИЗОВАТЬ РАСХОД ТОПЛИВА
//780 г/см^3 - плостность топлива
//1250-1500 кг - топлива (мск-питер) на 1 двигатель
//вес самолёта макс вес самолёта для взлёта Airbus A320
//расход (кг/с) = SFC * тягу
//SFC примерно 0,5
        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();

            _rigidbody.mass = 2000;
            _throttle01 = 0;
            _afterBurner = false;
                
            InitializeActions();
        }

        private void InitializeActions()
        {
            var map = _actionAsset.FindActionMap("JetEngine");

            _throttleUpHold = map.FindAction("ThrottleUp");
            _throttleDownHold = map.FindAction("ThrottleDown");
            _toggleAB = map.FindAction("ToggleAB");

            _toggleAB.performed += _ => { _afterBurner = !_afterBurner; };
        }

        private void OnEnable()
        {
            _throttleUpHold.Enable();
            _throttleDownHold.Enable();
            _toggleAB.Enable();
        }
        
        private void OnDisable()
        {
            _throttleUpHold.Disable();
            _throttleDownHold.Disable();
            _toggleAB.Disable();
        }

        private void FixedUpdate()
        {
            _speedMS = _rigidbody.linearVelocity.magnitude;

            float dt = Time.fixedDeltaTime;

            if (_throttleUpHold.IsPressed())
                _throttle01 = Mathf.Clamp01(_throttle01 + 1f * dt);
            
            if (_throttleDownHold.IsPressed())
                _throttle01 = Mathf.Clamp01(_throttle01-0.05f * dt);
            
            float thrust = _throttle01 * (_afterBurner?_thrustABSL:_thrustDrySL);

            Vector3 force = _nozzle.forward * thrust;
            _rigidbody.AddForceAtPosition(force,_nozzle.position, ForceMode.Force);
        }
    }
}