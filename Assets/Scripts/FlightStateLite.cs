    using System;
    using UnityEngine;

    public class FlightStateLite:MonoBehaviour
    {
        [SerializeField] private Transform _wingChord;

        //скорость самолёта относительно воздуха
        public float IAS { get;private set; }// м/с 
        //угол атаки
        public float AoAdeg { get; private set; }//deg
        //перегрузка
        public float Nz { get; private set; }
        
        private Rigidbody _rigidbody;
        private Vector3 _vPrev;
        private float _tPrev;

        private void Awake() => Initialize();

        private void Initialize()
        {
            _rigidbody = GetComponent<Rigidbody>();
            _vPrev=_rigidbody.linearVelocity;
            _tPrev=Time.time;
        }

        private void FixedUpdate()
        {
            //скорость
            Vector3 v = _rigidbody.linearVelocity;
            IAS = v.magnitude;
            
            //AoA кгол атаки
            if(IAS>1e-3)
            {
                Vector3 flow =(-v).normalized;
                float flowX = Vector3.Dot(flow, _wingChord.forward);
                float flowZ = Vector3.Dot(flow, _wingChord.up);
                AoAdeg = Mathf.Deg2Rad * Mathf.Atan2(flowZ , flowX);
            }
            else
            {
                AoAdeg = 0;
            }
            
            //перегрузка Nz ~=1+a_vertical/g
            float t = Time.time;
            float dt = Mathf.Max(1e-3f,t - _tPrev);
            Vector3 aWorld = (v-_vPrev )/ dt;
            float  aVert = Vector3.Dot(aWorld+Physics.gravity, transform.up);

            Nz = 1f + (aVert / Mathf.Abs(Physics.gravity.y));
            _vPrev = v;
            _tPrev = t;
        }
    }
