using System;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Gilder : MonoBehaviour
{
    [SerializeField] private Transform _wingCP;

    [Header("Плотность воздуха")]
    [SerializeField] private float _airDensity = 1.225f;
    
    [Header("Арэродинамические характеристики крыла")]
    [SerializeField] private float _wingArea = 1.5f;// площадь
    [SerializeField] private float _wingAspect = 8.0f;// удлинение AR=b^2/S

    [SerializeField] private float _wingCDO = 0.02f;// паразитное сопротивление
    [SerializeField] private float _wingCLaplha = 5.5f; //2Pi для тонкого профиля
    
    private Rigidbody _rigidbody;

    private Vector3 _worldVelocity;
    private Vector3 _vPoint;//скорость в точке крыла

    private float _speedMS;
    private float _alphaRad;

    private float _cd, _cl, _qDyn, _lMag, _dMag, _glideK;
    
    private void Awake()
    {
        _rigidbody = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        //1 скорость в точке крыла
        _vPoint = _rigidbody.GetPointVelocity(_wingCP.position);
        _speedMS = _vPoint.magnitude;

        //2 угол атаки из локальных осей
        Vector3 flowDir = (-_vPoint).normalized; // напаравление набегающего потока
        Vector3 xChord = _wingCP.forward;//  вдоль хорды
        Vector3 zUP = _wingCP.up;//нормаль к поверхности
        Vector3 ySpan = _wingCP.right;// для определения поъёмной силы (по размах)
        
        float flowX = Vector3.Dot(flowDir, xChord);
        float flowZ = Vector3.Dot(flowDir, zUP);
        _alphaRad = MathF.Atan2(flowZ, flowX);
        
        //3 Ародинамические коэффцициенты
        //CL = CLa * a 
        _cl = _wingCLaplha * _alphaRad;
        //CD = CDO  + (CL^2)/(Pi*AR*e) сопротивление|  e - фактор освальда
        _cd = _wingCDO + _cl * _cl / (Mathf.PI*_wingAspect * 0.85f);

        //4 силы 
        _qDyn = 0.5f * _airDensity * _speedMS * _speedMS;//динамическое давление
        _lMag = _qDyn * _wingArea * _cl;
        _dMag = _qDyn * _wingArea * _cd;

        //напарвление
        Vector3 Ddir = -flowDir;//против потока
        
        //подъёмную силу - перпендикулярно потоку в плоскости
        Vector3 liftDir = Vector3.Cross(flowDir, ySpan);
        liftDir.Normalize();
        
        Vector3 L = _lMag * liftDir;
        Vector3 D = _dMag * Ddir;

        _rigidbody.AddForceAtPosition(L+D, _wingCP.position, ForceMode.Force);
    }

    private void StepOne()
    {
        Vector3 xChord = _wingCP.forward;//  вдоль хорды
        Vector3 zUP = _wingCP.up;//нормаль к поверхности
        Vector3 flowDir = _speedMS>0 ? -_worldVelocity.normalized: _wingCP.forward;
        float flowX = Vector3.Dot(flowDir, xChord);
        float flowZ = Vector3.Dot(flowDir, zUP);
        _alphaRad = MathF.Atan2(flowZ, flowX); 
    }


    private void OnGUI()
    {
        GUI.color = Color.black;
        GUILayout.Label($"Скорость: {_speedMS:0.0} m/s");
        GUILayout.Label($"Угол атакт: {_alphaRad * Mathf.Deg2Rad:0.0} ");
    }
}
