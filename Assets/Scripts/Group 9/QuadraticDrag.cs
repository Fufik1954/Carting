

using UnityEngine;

namespace Group_9
{
    [RequireComponent(typeof(Rigidbody))]
    public class QuadraticDrag:MonoBehaviour
    {
        private float _radius;
        private float _dragCoefficient;
        private float _airDensity;
        private Vector3 _wind = Vector3.zero;

        private Rigidbody _rigidbody;
        private float _area;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            Vector3 vReal = _rigidbody.linearVelocity - _wind;
            float speed = vReal.magnitude;
            //Fdrag = -1/2 * p *Cd*A*v*v(vector) расчёт сопротивления
            Vector3 drag = -0.5f * _airDensity * _dragCoefficient * _area * speed * vReal;
            _rigidbody.AddForce(drag, ForceMode.Force);
        }

        public void SetPhysicalParams(float mass, float radius,
            float dragCoefficient, float airDensity, Vector3 wind,Vector3 initialVelocity)
        {
          _radius = radius;
          _dragCoefficient = dragCoefficient;
          _airDensity = airDensity;
          _wind = wind;
          
          _rigidbody.mass = mass;
          _rigidbody.useGravity = true;
          _rigidbody.linearVelocity = initialVelocity;

          _area = _radius * _radius * Mathf.PI;// площадь окружности
        }
    }
}