using System;
using UnityEngine;

public class GimblLock : MonoBehaviour
{
    public enum RightMode { EulerMode, QuaternionMode }

    [Header("Эйлер углы")] 
    [Range(-180, 180), SerializeField] private float _yawDeg;// вращение по y 
    [Range(-180, 180), SerializeField] private float _pitchDeg;// вращение по x 
    [Range(-180, 180), SerializeField] private float _rollDeg;// вращение по z
    [SerializeField] private RightMode _rightMode = RightMode.QuaternionMode;
    
    // эйлер
    private Transform _yawTransform, _pitchTransform, _rollTransform, _leftArrow;

    //quaterinos
    private Transform _rightRoot, _rightArrow;
    Quaternion _qRght = Quaternion.identity;
    private float  _yawPrew, _pitchPrew, _rollPrew;
    
    private void Start() => Initialize();

    private void Initialize()
    {
        var leftRoot = new GameObject("EulerGimbals").transform;
        
        _yawTransform = new GameObject("yaw").transform;
        _yawTransform.SetParent(leftRoot, false);
        
        _pitchTransform = new GameObject("pitch").transform;
        _pitchTransform.SetParent(_yawTransform, false);
        
        _rollTransform = new GameObject("roll").transform;
        _rollTransform.SetParent(_pitchTransform, false); 
        
        _leftArrow = CreateArrow("Left Arrow",_rollTransform);
        
        _rightRoot = new GameObject("right_rig").transform;
        _rightRoot.position = new Vector3(2.5f,0, 0);
        _rightArrow = CreateArrow("Right Arrow", _rightRoot);

        _yawPrew = _yawDeg;
        _pitchPrew = _pitchDeg;
        _rollPrew = _rollDeg;

    }

    private void Update()
    {
        if (_rightMode == RightMode.EulerMode)
        {
            _yawTransform.localRotation = Quaternion.Euler(0, _yawDeg, 0);
            _pitchTransform.localRotation = Quaternion.Euler(_pitchDeg, 0, 0);
            _rollTransform.localRotation = Quaternion.Euler(0, 0, _rollDeg);  
        }
        else if (_rightMode == RightMode.QuaternionMode)
        {
            //detaQ = AngleAxis(dRoll,z) * AngleAxis(dPitch,x) * AngleAxis(dYaw,y)
            //d* = Deg * dt
            
            float dt = 1;
            float dYaw = _yawDeg ;
            float dPitch = _pitchDeg ;
            float dRoll = _rollDeg;
            
            dYaw = _yawDeg - _yawPrew;
            dPitch = _pitchDeg - _pitchPrew;
            dRoll = _rollDeg - _rollPrew;

            //Quaternion dQEuler = Quaternion.Euler(dYaw, dPitch, dRoll);
            
            Quaternion dQ = Quaternion.AngleAxis(dRoll, _rightRoot.forward) *
                            Quaternion.AngleAxis(dPitch, _rightRoot.right) *
                            Quaternion.AngleAxis(dYaw, _rightRoot.up);
            _qRght = Normalized(_qRght * dQ);
            
            _rightRoot.rotation = _qRght;
            
            _yawPrew = _yawDeg;
            _pitchPrew = _pitchDeg;
            _rollPrew = _rollDeg;
        }
    }

    //НОРМАЛИЗАЦИЯ ПРИ НАКАПЛИВАНИИ ОШИБОК
    private Quaternion Normalized(Quaternion q)
    {
        float m = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z +q.w * q.w);
        
        return (m>1e-6f)?
            new Quaternion(q.x/m, q.y/m, q.z/m, q.w/m) : 
            Quaternion.identity;
    }

    // Helper
    private Transform CreateArrow(string name, Transform parent)
    {
        var root = new GameObject(name).transform;
        root.SetParent(parent, false); 
        
        var shaft = GameObject.CreatePrimitive(PrimitiveType.Cube);
        shaft.name = "shaft";
        shaft.transform.SetParent(parent, false); 
        shaft.transform.localScale = new Vector3(0.05f,  0.05f,1.5f);
        shaft.GetComponent<Renderer>().material.color = Color.yellow;

        return root;
    }
}
