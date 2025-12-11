using System;
using UnityEngine;

namespace Group_9
{
[RequireComponent(typeof(ForceVisualizers))]
    public class SimplyPhysicsEngine : MonoBehaviour
    {
  
        [Header("Физические параметры")]
        [SerializeField] private float _mass;
        [SerializeField] private bool _isGravity;
        [SerializeField] private float _dragCoefficient = 0.1f;
        [SerializeField] private Vector3 _windForce;
        
        private ForceVisualizers _forceVisualizers;
        private Vector3 _netForce;
        private Vector3 _velocity = Vector3.zero;

    
        private void Start()
        {
            _forceVisualizers = GetComponent<ForceVisualizers>();
        }

        private void FixedUpdate()
        {
            _netForce = Vector3.zero;
            _forceVisualizers.ClearForces();

            if (_isGravity)
            {
                Vector3 gravity = Physics.gravity * _mass;//F=mg
                ApplyForce(gravity,Color.cyan, "Gravity");
            }
            //wind
            ApplyForce(_windForce,Color.blue, "WindForce");

            Vector3 acceleration=_netForce/_mass;
            IntegrateMotion(acceleration);
            
            _forceVisualizers.AddForce(_netForce,Color.red,"ForceMAIN");
        }
        
        private void IntegrateMotion(Vector3 acceleration)
        {
            _velocity+=acceleration * Time.fixedDeltaTime;// euler method
            transform.position+=_velocity*Time.fixedDeltaTime;
        }
        

        private void ApplyForce(Vector3 force, Color colorForce, string name)
        {
            _netForce += force;
            _forceVisualizers.AddForce(force, colorForce, name);
        }
    }
}
