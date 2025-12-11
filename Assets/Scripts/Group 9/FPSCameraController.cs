using System;
using UnityEngine;
using UnityEngine.InputSystem;

public class FPSCameraController : MonoBehaviour
{
    [SerializeField] private float _yawSensity = 180f;
    [SerializeField] private float _pitchSensity = 180f;
    [SerializeField] private float _maxPitchDrag = 89f;

    [SerializeField, Range(0, 1)] private float _rotationDamping;//доля интерполяции при 60 FPS
    
    private float _yawDeg; //градус вокруг вертикальной оси
    private float _pitchDeg; //градус вокруг правой оси
    private Quaternion _targetRotation;//целевая ориентация камеры 
    
    private void Awake()
    {
        _yawDeg = transform.eulerAngles.y;
        _pitchDeg = transform.eulerAngles.x;
        
        //цель из текущего поворота
        _targetRotation = transform.rotation;
        
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    private void Update()
    {
        float dx = Input.GetAxis("Mouse X"); // поворот камеры по горизонтальной оси
        float dy = Input.GetAxis("Mouse Y");//вертикальной
        
        //обновляем накопленные углы
        _yawDeg += dx * _yawSensity * Time.deltaTime;
        _pitchDeg -= dy *_pitchSensity * Time.deltaTime;//инверсия
        _pitchDeg = Mathf.Clamp(_pitchDeg,-_maxPitchDrag,_maxPitchDrag);
        
        Quaternion yawRot = Quaternion.AngleAxis(_yawDeg, Vector3.up);//курс будет всегда вертикальной сцены а не локальных координат
        
        //вокруг правой оси уже поверноутой раки yaw
        Vector3 rifghtAxis = yawRot * Vector3.right;
        Quaternion pitchRot = Quaternion.AngleAxis(_pitchDeg, rifghtAxis);
        
        _targetRotation=pitchRot * yawRot;
        
        //плавное наведение чере Slerp. t = 1-(1-d)^(dt*60)
        float t = 1 - Mathf.Pow(1 - Mathf.Clamp01(_rotationDamping), Time.deltaTime * 60f); //60 - целевое колличесвто кадров.
        transform.rotation = Quaternion.Slerp(transform.rotation, _targetRotation, t);
    }
}
