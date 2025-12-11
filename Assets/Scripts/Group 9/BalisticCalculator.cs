using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Group_9
{
    [RequireComponent(typeof(TraectoryRenderer))]
    public class BalisticCalculator:MonoBehaviour
    {
        [SerializeField] private Transform _launchPoint;
        [SerializeField] private float _muzzleVelocity = 20;
        [SerializeField, Range(0,85)] private float _muzzleAngle = 20;
        [Space] [SerializeField] private QuadraticDrag _shootRound;

        [Header("General    params")]
        [SerializeField] private float _mass= 1f;
        [SerializeField] private float _radius = 0.1f;
        [SerializeField] private float _dragCoefficient = 0.47f;
        [SerializeField] private float _airDensity = 1.225f;
        [SerializeField] private Vector3 _wind = Vector3.zero;//  м/c
        
        private TraectoryRenderer _traectoryRenderer;
        
        private void Start()
        {
            _traectoryRenderer = GetComponent<TraectoryRenderer>();
        }

        private void Update()
        {
            if(_launchPoint==null) return;
            
            Vector3 v0 = CalculateVelocityVector(_muzzleAngle);
            _traectoryRenderer.DrawVacuum(_launchPoint.position, v0);
            
            if(Keyboard.current.spaceKey.wasPressedThisFrame)
                Fire(v0);
        }

        private void Fire(Vector3 initialVelocity)
        {
            if(_shootRound == null) return;
            GameObject newShootRound = Instantiate(_shootRound.gameObject,_launchPoint.position,Quaternion.identity);
            
            QuadraticDrag quadraticDrag = newShootRound.GetComponent<QuadraticDrag>();
            quadraticDrag.SetPhysicalParams(_mass, _radius, _dragCoefficient, 
                _airDensity,_wind, initialVelocity);
        }

        //vx=v0*cos(a)
        //vy=v0*sin(a)
        private Vector3 CalculateVelocityVector(float angle)
        {
            float vx = _muzzleVelocity * Mathf.Cos(angle * Mathf.Deg2Rad);
            float vy = _muzzleVelocity * Mathf.Sin(angle * Mathf.Deg2Rad);
            return _launchPoint.forward * vx + _launchPoint.up * vy;
        }
    }
}