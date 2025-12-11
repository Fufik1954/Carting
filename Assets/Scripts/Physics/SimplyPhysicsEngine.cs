using System;
using UnityEngine;



[RequireComponent(typeof(ForceVisuliizers))]
public class SimplyPhysicsEngine : MonoBehaviour
{
    private const float  GravityConst = 9.8f;
    
    [SerializeField] private float _mass = 1f;
    [SerializeField] private bool _useGravity = true;
    [SerializeField] private Vector3 _windForce;
    
    private Vector3 _netForce;
    private ForceVisuliizers _visuliizers;
    private Vector3 _velocity = Vector3.zero;

    private void Start()
    {
        _visuliizers = GetComponent<ForceVisuliizers>();
    }

    private void FixedUpdate()
    {
        _netForce = Vector3.zero;   
        _visuliizers.ForceClear();
        
        if (_useGravity)
        {
            Vector3 gravity = Vector3.down * (_mass * GravityConst);
            ApplyForce(gravity,Color.cyan, "Gravity");
        }
        
        //  сила ветра
        ApplyForce(_windForce,Color.yellow, "WindForce");
        
        _visuliizers.AddForce(_netForce,Color.blue, "NetForce");

        Vector3 acceleration = _netForce/_mass;//F=ma
        
        _velocity += acceleration * Time.fixedDeltaTime;//v=v0+a*t
        
        transform.position += _velocity * Time.fixedDeltaTime;
    }
    
    

    private void ApplyForce(Vector3 force, Color color, string name)
    {
        _netForce += force;
        _visuliizers.AddForce(force, color, name);
    }
}
