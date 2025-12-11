using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{

    [Header("Configuration")]
    [SerializeField] private KartConfig _kartConfig;

    [Header("Physics")]
    [SerializeField] private float _gravity = 9.81f;

    [Header("Engine & drivetrain")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;

    [Header("Wheel attachment points")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    // For wheel rotation
    private Quaternion _frontLeftInitialLocalRot;
    private Quaternion _frontRightInitialLocalRot;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionReference _moveActionRef;

    // Input values
    private float _throttleInput; // -1..1 (газ/тормоз)
    private float _steerInput;    // -1..1 (руль)

    [Header("Handbrake")]
    [SerializeField] private InputActionReference _handbrakeActionRef;
    [SerializeField] private float _handbrakeDragMultiplier = 3f;
    [SerializeField] private bool _handbrakeEnabled = true;

    // Для сбора данных телеметрии
    private float _totalRearLongitudinalForce;
    private float _totalFrontLateralForce;
    private float _frontLeftVLat, _frontRightVLat, _rearLeftVLat, _rearRightVLat;
    private Vector3 _lastVelocity;
    private float _acceleration;

    private bool _isHandbrakePressed;
    private float _originalLateralStiffness; // Для сохранения оригинального значения

    // Private fields
    private Rigidbody _rb;

    // Normal forces (for debugging/display)
    private float _frontLeftNormalForce;
    private float _frontRightNormalForce;
    private float _rearLeftNormalForce;
    private float _rearRightNormalForce;


    private void Start()
    {
        _rb = GetComponent<Rigidbody>();
        ComputeStaticWheelLoads();
        Initialize();
        _originalLateralStiffness = _kartConfig.frontLateralStiffness;
    }

    private void Update()
    {
        ReadInput();     // Читаем ввод каждый кадр
        RotateFrontWheels(); // Поворачиваем колеса
    }

    private void FixedUpdate()
    {
        // Считаем ускорение
        Vector3 velocityChange = _rb.linearVelocity - _lastVelocity;
        _acceleration = velocityChange.magnitude / Time.fixedDeltaTime;
        _lastVelocity = _rb.linearVelocity;

        // Обнуляем силы каждый кадр
        _totalRearLongitudinalForce = 0f;
        _totalFrontLateralForce = 0f;

        ApplyWheelForces(_frontLeftWheel, _frontLeftNormalForce, isDriven: false);
        ApplyWheelForces(_frontRightWheel, _frontRightNormalForce, isDriven: false);

        ApplyWheelForces(_rearLeftWheel, _rearLeftNormalForce, isDriven: true);
        ApplyWheelForces(_rearRightWheel, _rearRightNormalForce, isDriven: true);
    }


    /// <summary>
    /// Calculate static weight distribution (Этап 1)
    /// </summary>
    private void ComputeStaticWheelLoads()
    {
        // 1. Получаем массу из Rigidbody
        float mass = _rb.mass;

        // 2. Рассчитываем общий вес: W = m * g
        float totalWeight = mass * _gravity;

        // 3. Распределяем вес по осям
        //    W_f = frontShare * W
        //    W_r = (1 - frontShare) * W
        float frontWeight = totalWeight * _kartConfig.frontAxleShare;
        float rearWeight = totalWeight * (1f - _kartConfig.frontAxleShare);

        // 4. Делим поровну между левым и правым колесом на каждой оси
        //    N_FL = N_FR = W_f / 2
        //    N_RL = N_RR = W_r / 2
        _frontLeftNormalForce = frontWeight * 0.5f;
        _frontRightNormalForce = frontWeight * 0.5f;

        _rearLeftNormalForce = rearWeight * 0.5f;
        _rearRightNormalForce = rearWeight * 0.5f;
    }

    /// <summary>
    /// Initialize wheel rotations (Этап 2)
    /// </summary>
    private void Initialize()
    {

        if (_frontLeftWheel != null)
            _frontLeftInitialLocalRot = _frontLeftWheel.localRotation;
        if (_frontRightWheel != null)
            _frontRightInitialLocalRot = _frontRightWheel.localRotation;
    }

    /// <summary>
    /// Read input from New Input System (Этап 2)
    /// </summary>
    private void ReadInput()
    {
        Vector2 move = _moveActionRef.action.ReadValue<Vector2>();
        _steerInput = Mathf.Clamp(move.x, -1f, 1f);
        _throttleInput = Mathf.Clamp(move.y, -1f, 1f);

        _isHandbrakePressed = _handbrakeActionRef.action.ReadValue<float>() > 0.5f;
 
    }

    /// <summary>
    /// Rotate front wheels based on input (Этап 2)
    /// </summary>
    private void RotateFrontWheels()
    {
        float steerAngle = _kartConfig.maxSteerAngle * _steerInput;
        Quaternion steerRotation = Quaternion.Euler(0f, steerAngle, 0f);

        if (_frontLeftWheel != null)
            _frontLeftWheel.localRotation = _frontLeftInitialLocalRot * steerRotation;
        if (_frontRightWheel != null)
            _frontRightWheel.localRotation = _frontRightInitialLocalRot * steerRotation;
    }

    /// <summary>
    /// Enable input actions (Этап 2)
    /// </summary>
    private void OnEnable()
    {
        if (_moveActionRef != null && _moveActionRef.action != null)
            _moveActionRef.action.Enable();

        // Включаем ручной тормоз (ЭТАП 7)
        if (_handbrakeActionRef != null && _handbrakeActionRef.action != null)
            _handbrakeActionRef.action.Enable();
    }

    /// <summary>
    /// Disable input actions (Этап 2)
    /// </summary>
    private void OnDisable()
    {
        if (_moveActionRef != null && _moveActionRef.action != null)
            _moveActionRef.action.Disable();

        // Выключаем ручной тормоз (ЭТАП 7)
        if (_handbrakeActionRef != null && _handbrakeActionRef.action != null)
            _handbrakeActionRef.action.Disable();
    }

    /// <summary>
    /// Apply engine force to rear wheels (Этап 3)
    /// </summary>
    //private void ApplyDriverForceToRearWheels()
    //{
    //    Vector3 bodyForward = transform.forward;
    //    float speedAlongForward = Vector3.Dot(_rb.linearVelocity, bodyForward);

    //    // Ограничение максимальной скорости вперед
    //    if (_throttleInput > 0f && speedAlongForward > _maxSpeed)
    //        return;

    //    // Ограничение скорости назад (опционально)
    //    if (_throttleInput < 0f && speedAlongForward < -_maxSpeed * 0.5f)
    //        return;

    //    float driveTorque = _engineTorque * _throttleInput;
    //    float driveForce = driveTorque / _wheelRadius; // F = M / r

    //    // Делим на два задних колеса
    //    Vector3 forcePerWheel = bodyForward * (driveForce * 0.5f);

    //    // Применяем силу к точкам крепления колес
    //    if (_rearLeftWheel != null)
    //        _rb.AddForceAtPosition(forcePerWheel, _rearLeftWheel.position, ForceMode.Force);

    //    if (_rearRightWheel != null)
    //        _rb.AddForceAtPosition(forcePerWheel, _rearRightWheel.position, ForceMode.Force);
    //}

    private void ApplyWheelForces(Transform wheel, float normalForce, bool isDriven)
    {

        if (wheel == null || _rb == null) return;

        Vector3 wheelPos = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight = wheel.right;
        Vector3 v = _rb.GetPointVelocity(wheelPos);

        float vLong = Vector3.Dot(v, wheelForward);
        float vLat = Vector3.Dot(v, wheelRight);

        float Fx = 0f;
        float Fy = 0f;

        // Записываем боковое скольжение для телеметрии
        if (wheel == _frontLeftWheel) _frontLeftVLat = vLat;
        if (wheel == _frontRightWheel) _frontRightVLat = vLat;
        if (wheel == _rearLeftWheel) _rearLeftVLat = vLat;
        if (wheel == _rearRightWheel) _rearRightVLat = vLat;

        // 1) продольная сила от двигателя — только задняя ось
        if (isDriven)
        {
            Vector3 bodyForward = transform.forward;
            float speedAlongForward = Vector3.Dot(_rb.linearVelocity, bodyForward);

            // Получаем момент от двигателя
            float engineTorque = _engine.Simulate(
                _throttleInput,
                speedAlongForward,
                Time.fixedDeltaTime
            );

            float totalWheelTorque = engineTorque * _kartConfig.gearRatio * _drivetrainEfficiency;
            float wheelTorque = totalWheelTorque * 0.5f; // два задних колеса

            // ЕСЛИ РУЧНОЙ ТОРМОЗ НАЖАТ - ИГНОРИРУЕМ ДВИГАТЕЛЬ
            if (!(_isHandbrakePressed && isDriven && _handbrakeEnabled))
            {
                Fx += wheelTorque / _kartConfig.wheelRadius;
            }
        }

        // 2) сопротивление качению
        float currentRollingResistance = _kartConfig.rollingResistance;

        // Ручной тормоз: увеличиваем сопротивление для задних колес
        if (_isHandbrakePressed && isDriven && _handbrakeEnabled)
        {
            currentRollingResistance *= _handbrakeDragMultiplier;

            // ДОБАВЛЯЕМ СИЛУ БЛОКИРОВКИ КОЛЕС
            // Сила, направленная ПРОТИВ движения колеса
            float brakeForce = -Mathf.Sign(vLong) * normalForce * _kartConfig.frictionCoefficient * 0.8f;
            Fx += brakeForce;
        }

        Fx += -currentRollingResistance * vLong;

        // 3) боковая сила
        float currentLateralStiffness = _kartConfig.frontLateralStiffness;

        // Ручной тормоз: убираем боковое сцепление у задних колес
        if (_isHandbrakePressed && isDriven && _handbrakeEnabled)
        {
            currentLateralStiffness = 0f; // Полное отсутствие бокового сцепления
        }

        Fy += -currentLateralStiffness * vLat;

        // 4) фрикционный круг (ВАЖНО: применяется к СУММАРНОЙ силе)
        // При ручном тормозе ослабляем фрикционный круг для задних колес
        float currentFrictionCoefficient = _kartConfig.frictionCoefficient;
        if (_isHandbrakePressed && isDriven && _handbrakeEnabled)
        {
            currentFrictionCoefficient *= 0.3f; // Уменьшаем общее сцепление на 70%
        }

        float frictionLimit = currentFrictionCoefficient * normalForce;
        float forceLength = Mathf.Sqrt(Fx * Fx + Fy * Fy);

        if (forceLength > frictionLimit && forceLength > 1e-6f)
        {
            float scale = frictionLimit / forceLength;
            Fx *= scale;
            Fy *= scale;
        }

        // Собираем силы для телеметрии
        if (isDriven) // Задние колеса
        {
            _totalRearLongitudinalForce += Fx;
        }
        else // Передние колеса
        {
            _totalFrontLateralForce += Mathf.Abs(Fy);
        }

        // 5) мировая сила
        Vector3 force = wheelForward * Fx + wheelRight * Fy;

        _rb.AddForceAtPosition(force, wheelPos, ForceMode.Force);
    }

    private void OnGUI()
    {
        float speedMs = _rb.linearVelocity.magnitude;
        float speedKmh = speedMs * 3.6f;

        float x = 10;
        float y = 10;
        float lineHeight = 20;

        // 1. Скорость
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Скорость: {speedKmh:F1} км/ч ({speedMs:F1} м/с)");
        y += lineHeight;

        // 2. RPM
        GUI.Label(new Rect(x, y, 300, lineHeight), $"RPM: {_engine.CurrentRpm:F0}");
        y += lineHeight;

        // 3. Момент двигателя
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Момент: {_engine.CurrentTorque:F0} Н·м");
        y += lineHeight;

        // 4. Ускорение
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Ускорение: {_acceleration:F1} м/с²");
        y += lineHeight;

        // 5. Fx суммарная продольная сила задней оси
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Fx задняя ось: {_totalRearLongitudinalForce:F0} Н");
        y += lineHeight;

        // 6. Fy суммарная боковая сила передней оси
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Fy передняя ось: {_totalFrontLateralForce:F0} Н");
        y += lineHeight;

        // 7. Боковое скольжение колес
        GUI.Label(new Rect(x, y, 300, lineHeight),
            $"Скольжение перед: L={_frontLeftVLat:F2} R={_frontRightVLat:F2}");
        y += lineHeight;

        GUI.Label(new Rect(x, y, 300, lineHeight),
            $"Скольжение зад: L={_rearLeftVLat:F2} R={_rearRightVLat:F2}");
        y += lineHeight;

        // 8. Ручной тормоз
        if (_isHandbrakePressed)
        {
            GUI.Label(new Rect(x, y, 300, lineHeight), "Ручной тормоз: АКТИВЕН");
        }
    }
}