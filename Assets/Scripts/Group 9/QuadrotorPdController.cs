using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Group_9
{
    [RequireComponent(typeof(Rigidbody))]
    public class QuadrotorPdController:MonoBehaviour
    {
        [Header("Physics Parameters")]
        [SerializeField] private float _mass = 1.5f;
        [SerializeField] private float _maxThrottle = 30f;
        [SerializeField] private float _maxTorque = 5f;

        [SerializeField] private float _maxPitchDeg = 20f;//предел задания тангажа
        [SerializeField] private float _maxRollDeg = 20f;//крена
        [SerializeField] private float _yawRateDegPerSec = 90f;//скорость изменения курса ввода
        
        private Rigidbody _rigidbody;
        private float _desiredYawDeg; // желаемый курс в градусах

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
            _rigidbody.mass = Mathf.Max(0.01f, _mass);
            
            _desiredYawDeg = transform.eulerAngles.y;
        }

        private void Update()
        {
            //целевой курс из оси поворта
            float yawInput = Mathf.Clamp(Input.GetAxis("Mouse X"), -1, 1);
            _desiredYawDeg += yawInput * _yawRateDegPerSec * Time.deltaTime;
        }

        private void FixedUpdate()
        {
            //вводные данные
            float pitchInput = Mathf.Clamp(Input.GetAxis("Vertical"), -1f, 1f);
            float rollInput = Mathf.Clamp(Input.GetAxis("Horizontal"), -1f, 1f);
            
            //тяга
            float throttleInput = Keyboard.current.spaceKey.isPressed?  1f: 0f;
            
            float targetPitch = pitchInput * _maxPitchDeg;// тангаж
            float targetRoll = -rollInput * _maxRollDeg;//крен
            
            float targetyawDeg = _desiredYawDeg;
            
            //целевая ориентация qTarget
            Quaternion qTarget = Quaternion.Euler(targetPitch,targetyawDeg,targetRoll);
            Quaternion qCurrent = _rigidbody.rotation;

            //q_err = qTarget * inverse(qCurrent) ошибка ориентации
            Quaternion qError = qTarget * Quaternion.Inverse(qCurrent);
            
            //поиск крайтчайшей дуги для поворота
            if (qError.w < 0)
                qError = new Quaternion(-qError.x,-qError.y,-qError.z,-qError.w);
            
            //угол-ось 
            qError.ToAngleAxis(out float angleDeq, out Vector3 axis);
            
            // перевод в радианы
            float angleRed = Mathf.Deg2Rad * angleDeq;

            //PD момент по ориентации tau=Kp*theta*axis-Kd*omega
            Vector3 omega = _rigidbody.angularVelocity;
            Vector3 torque = 8 * angleRed * axis - 2.5f*omega;//Kp=8,Kd = 2.5f

            float maxT = Mathf.Max(0, _maxTorque);
            
            if (torque.sqrMagnitude > maxT * maxT)
                torque = torque.normalized * maxT;
            
            //_rigidbody.AddTorque(torque);//подключили повороты
            
            //ТЯГА Базовое висение - силы mg
            float g = Physics.gravity.magnitude;
            float hover = g * _rigidbody.mass;//H

            float centerd = (throttleInput - 0.5f) * 2f;//преобразование от -1 до 1
            float comandedForce = hover + centerd * (0.5f * _maxThrottle);//добавление 50% силы
            comandedForce = Mathf.Clamp(comandedForce, 0, _maxThrottle);
            
            _rigidbody.AddForce(transform.up*comandedForce,ForceMode.Force);
        }
    }
}