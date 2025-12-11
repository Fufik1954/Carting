using System;
using UnityEngine;
using UnityEngine.InputSystem;

public class BalisticCalculator : MonoBehaviour
{
    [SerializeField] private float _mussleVelocity = 50f;//нач скорость
    //[SerializeField] private float _angle = 45f;//угол
    [SerializeField] private Transform _canon;//пушка
    [SerializeField] private GameObject _core;//ядро
    [SerializeField] private LineRenderer _trajectoryLine;//для отображения траектории
    [SerializeField] private int _pointsCount;//качество отбражение траектории
    
    
    //расчёт координат по пораболе
    //x=v*cos * t
    //y=v*sin * t - 0.5*g*t^2

    private void Start()
    {
        _trajectoryLine.gameObject.transform.position = Vector3.zero;
        _trajectoryLine.gameObject.transform.rotation = Quaternion.identity;
    }

    private void Update()
    {
        DrawTrajectory(_canon.rotation.eulerAngles.z);
        
        if(Keyboard.current.spaceKey.wasPressedThisFrame)
            FireProjectile();
        
        
    }

    private void FireProjectile()
    {
        GameObject newCore = Instantiate(_core, _canon.position, Quaternion.identity);
        Rigidbody rb = newCore.GetComponent<Rigidbody>();

        if (rb != null)
        {
            Vector3 velocity = _canon.up * _mussleVelocity;
            rb.linearVelocity = velocity;
        }
    }


    private Vector3 CalculateVelocity(float angle)
{
    var vx = _mussleVelocity * Mathf.Cos(angle * Mathf.Deg2Rad);
    var vy = _mussleVelocity * Mathf.Sin(angle * Mathf.Deg2Rad);
    return _canon.right * vx + _canon.up * vy;
}


private void DrawTrajectory(float angle)
{
    _trajectoryLine.positionCount = _pointsCount;
    var startPoint = _canon.position;
    
    var velocity = CalculateVelocity(angle);
    var position = _canon.position;

        var gravity = Physics.gravity;
    for (var i = 0; i < _pointsCount; i++)
    {
        var t = i * 0.1f;
        
        //var point = startPoint + velocity * t;
        //point.y -= 0.5f * 9.81f * t * t;

        Vector3 gravityForce = gravity * _core.GetComponent<Rigidbody>().mass;
        Vector3 totalForce = gravity ;//+ добавлять сопротивления
        
        Vector3 acceleration = totalForce/_core.GetComponent<Rigidbody>().mass;
        
        velocity+=acceleration*t;

        position += velocity * t;

        
        _trajectoryLine.SetPosition(i,position);
    }
}

}
