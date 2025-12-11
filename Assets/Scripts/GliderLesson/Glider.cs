using System;
using UnityEngine;

namespace GliderLesson
{
    [RequireComponent(typeof(Rigidbody))]
    public class Glider:MonoBehaviour
    {
        [Header("Atmsophere")]
        [SerializeField] private float _airDensity = 1.225f;//плостность атмосферы
        
        [Header("References")]
        [SerializeField] private Transform _wingCP;//контрольная точка крыла

        [Header("Wing Geometry & Aero")]
        [SerializeField] private float _wingAero = 1.5f;//площадь крыла м*м
        [SerializeField] private float _wingAspectRatio = 8f;//удлинение крыла b*b/s
        [SerializeField] private float _oswaldEfficiency = 0.85f;//фактор освальда

        [SerializeField] private float _wingCD0 = 0.02f;//паразитное сопростиваление - сопротивление вызаное трением формы
        [SerializeField] private float _wingCLapla = 5.5f;//Подъём на радиан. Для тонког профиля ~ 2pi

        [SerializeField] private float _alphaLimitDeg = 18f;//оагрничение угла, чтобы не произошёл срыв
        
        //телеметрия
        private Vector3 _vPoint;
        private float _speedMS;
        private float _alphaRad;
        private float _cl, _cd, _qDyn, _lMag, _dMagm, _glider;
        
        private Rigidbody _rigidbody;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            if(_wingCP == null)
                return;
            
            //1) скорость крыла в точке
            _vPoint = _rigidbody.GetPointVelocity(_wingCP.position);
            _speedMS = _vPoint.magnitude;
            
            if(_speedMS<0)
                return;
            
           //2)угол атаки
           Vector3 flowDir = (-_vPoint).normalized;//направление набегающего потока
           Vector3 xChord = _wingCP.forward;//вдоль хорды
           Vector3 zUp = _wingCP.up;//нормаль к поверхности
           
           Vector3 ySpan = _wingCP.right;//для определения направления поъёмной силы
           
           float flowX = Vector3.Dot(flowDir,xChord);
           float flowZ = Vector3.Dot(flowDir,zUp);

           float aplhaRaw = Mathf.Atan2(flowZ, flowX);//угол атаки
           
           //мягкое ограничение, чтобы модель не уходило в не устойчевую область
           float aLim  = Mathf.Deg2Rad*Mathf.Abs(_alphaLimitDeg);
           _alphaRad = Mathf.Clamp(aplhaRaw,-aLim,+aLim);
           
           //3) Аэродинамические коэффициенты
           _cl = _wingCLapla * _alphaRad;//CL=CLa*a
           
           //индуциорванное сопротивление CD = CD0+ (CL*CL)/(PI AR e)
           var kInduced = 1f/(Mathf.PI * 
                              Mathf.Max(_wingAspectRatio,0)*
                          Mathf.Max(_oswaldEfficiency,0));
           
           _cd = _wingCD0 + kInduced *_cl * _cl;
           
           //4) Силы

           //динамическое давление
           _qDyn = 0.5f * _airDensity * _speedMS * _speedMS;
           _lMag = _qDyn * _wingAero * _cl;
           _dMagm= _qDyn * _wingAero * _cd;   
           
           //направление
           Vector3 Ddir = -flowDir;//против потока
           
           //подъёмная сила перпендикулярная потоку в плоскости
           Vector3 liftDir = Vector3.Cross(flowDir,ySpan);
           liftDir.Normalize();
           
           Vector3 L = _lMag * liftDir;
           Vector3 D = _dMagm * Ddir;
           
           //5) Приложение силы к точке крыла
           
           _rigidbody.AddForceAtPosition(D+L,_wingCP.position, ForceMode.Force);
        }

        private void OnGUI()
        {
            GUI.color = Color.black;
            
            GUILayout.BeginArea(new Rect(10, 10, 300, 150), GUI.skin.box);
            
            GUILayout.Label("Glider HUD");
            GUILayout.Label($"Скорость {_speedMS:0.0}");
            GUILayout.Label($"Угол атаки {Mathf.Rad2Deg * _alphaRad:0.0}");
            GUILayout.Label($"Динамическое давление {_qDyn:0.0}");

            GUILayout.EndArea();
        }
    }
}