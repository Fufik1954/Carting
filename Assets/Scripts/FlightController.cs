using System;
using UnityEngine;
using UnityEngine.InputSystem;

public class FlightController:MonoBehaviour
{
        [SerializeField] private InputActionAsset _playerInput;
        [SerializeField] private FlightProtection _flightProtection;
        [Header("Rate Control (PD)")]
        [SerializeField] private Vector3 _maxRateDeg = new Vector3(90, 90, 120);//pith yaw roll
        [SerializeField] private Vector3 _kp = new Vector3(3,2,3);
        [SerializeField] private Vector3 _kd = new Vector3(0.8f,0.6f,0.9f); //deg/s kd демпфер
        
        [SerializeField] private Vector3 _maxTorque = new Vector3(30,25,35);
        [SerializeField] private float _deadZone = 0.05f;
        
        [SerializeField] private Vector2 _attHoldKp = new Vector2(2, 2);
        [SerializeField] private float _attHoldMaxRate = 45f;
        
        private Rigidbody _rigidbody;
        private InputAction _yaw;
        private InputAction _pitch;
        private InputAction _roll;
        private InputAction _hold;

        private float _targetPitchDeg;
        private float _targetRollDeg;
        private bool _isHolding;

        private Vector3 _omegaBodyDeg;

        private void Awake() => Initialize();

        private void Initialize()
        {
                _rigidbody = GetComponent<Rigidbody>();

                var map = _playerInput.FindActionMap("Player");
                _pitch = map.FindAction("Pitch");
                _roll = map.FindAction("Roll");
                _yaw = map.FindAction("Yaw");
                _hold = map.FindAction("HoldAttribute");
        }

        private void OnEnable()
        {
                _hold.performed += OnHoldOn;
                _hold.canceled += OnHoldOff;
        }

        private void OnDisable()
        {
                _hold.performed -= OnHoldOn;
                _hold.canceled -= OnHoldOff;
        }

        private void OnHoldOn(InputAction.CallbackContext _)
        {
                _isHolding = true;
                var e = GetLocalPitchRollDeg();
                _targetPitchDeg = e.xPitch;
                _targetRollDeg = e.zRoll;
        }
        
        private void OnHoldOff(InputAction.CallbackContext _)
        {
                _isHolding = false;
        }

        private void FixedUpdate()
        {
                Vector3 omega = _rigidbody.angularVelocity;//текущая угловая скорость рад/с
                Vector3 omegaBody = transform.InverseTransformDirection(omega);//телеметрии
                 _omegaBodyDeg = omegaBody * Mathf.Rad2Deg;
                
                //считываем ввод со стиков/клавиатуры
                Vector3 rateCmdDeg = ReadRateCommnadDeg();

                if (_isHolding)
                        rateCmdDeg = GenerateHoldRateDeg();//генерация кгловой скорости при газе
                
                //3) PD по каждой оси tau=Kp*(w_cmd - w)-Kd*w
                //PD -Пропорциональный-дифференциальный регулятор
                //PID - используетсья интегрирование
                //e = x_target - x_сurrent
                //PD управляющее воздействие
                //u = k_p*e+k_d*(de/dt)
                //----------
                //e - ошибка по скорости вращения
                //k_p - "На сколько сильнее крутим, если ошиблись"
                //k_d - тормозим
                //-> tau=Kp*(w_cmd - w)-Kd*w
                //tau - момент который передаёться в RigidBody
                //P - отклонение на 10, необходимо дать момент 10*k_p
                //D - "армотизатор". гасит скорость
                //tau=Kp*(w_cmd - w)-Kd*w
                //------
                //медленно регаааирует -> увеличить k_p
                //колебания -> уменьшаем k_p либо увеличиваем k_d
                //Слишком резко замирает -> уменьшение k_d
                //слишком резко крутит -> добавить ограничение момента/тангенс

                rateCmdDeg = _flightProtection.ApplyLimiters(rateCmdDeg);
                
                Vector3 errDeg = rateCmdDeg - _omegaBodyDeg;
                Vector3 tau = new Vector3(
                        _kp.x * errDeg.x - _kd.x*_omegaBodyDeg.x,
                        _kp.y * errDeg.y - _kd.y*_omegaBodyDeg.y,
                        _kp.z * errDeg.z - _kd.z*_omegaBodyDeg.z);
                
                //момент в локальных осях
                _rigidbody.AddTorque(tau, ForceMode.Force);
        }

        private Vector3 GenerateHoldRateDeg()
        {
                //генерация скорости с учётом ошибок при расчётах
                var e = GetLocalPitchRollDeg();//
                
                float errPitch = Mathf.DeltaAngle(e.xPitch, _targetPitchDeg);
                float errRoll = Mathf.DeltaAngle(e.zRoll, _targetRollDeg);
                
                float wPitch = Mathf.Clamp(errPitch*_attHoldKp.x, -_attHoldMaxRate, _attHoldMaxRate);
                float wRoll = Mathf.Clamp(errRoll*_attHoldKp.y, -_attHoldMaxRate, _attHoldMaxRate);
                
                return new Vector3(wPitch,0,wRoll);
        }

        private (float xPitch, float zRoll) GetLocalPitchRollDeg()
        {
                //берём локальные Элейры в последовательности для Unity (ZXY)
                Vector3 e = transform.localEulerAngles;
                float pitch = NormalizeAngle(e.x);
                float roll = NormalizeAngle(e.z);
                return (pitch, roll);
        }
        
        private float NormalizeAngle(float a)
                =>(a > 180) ? a - 360 : a;

        private Vector3 ReadRateCommnadDeg()
        {
                float uPitch = _pitch .ReadValue<float>();
                float uRoll= _roll .ReadValue<float>();  
                float uYaw = _yaw .ReadValue<float>();
                
                Vector3 max = _maxRateDeg;
                return new Vector3(uPitch * max.x, uYaw * max.y,  uRoll* max.z);
        }

        private void OnGUI()
        {
                GUI.color = Color.black;
                GUILayout.BeginArea(new Rect(12,220,420,220), GUI.skin.box);
                GUILayout.Label("FlightController");
                GUILayout.Label($"p={_omegaBodyDeg.x:0}");

                GUILayout.EndArea();
        }
}