using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class ForceVisuliizers : MonoBehaviour
{
    [SerializeField] private List<Force> _forceList = new();

    public void OnDrawGizmos()
    {
        if(_forceList is null || _forceList.Count == 0)
            return;
        
        float cubeSize = 0.1f; 
        foreach (var force in _forceList)
        {
            Gizmos.color = force.Color;
            Vector3 startPoint = transform.position;
            Vector3 endPoint= startPoint + force.Vector;
            Gizmos.DrawLine(startPoint, endPoint);
            Gizmos.DrawCube(endPoint, Vector3.one * cubeSize);
#if UNITY_EDITOR
            UnityEditor.Handles.Label(endPoint, force.Name);
#endif
        }
    }

    public void AddForce(Vector3 force, Color color, string name)
    {
        Force newForce = new Force(force, color, name);
        _forceList.Add(newForce);
    }
    
    public void ForceClear() =>_forceList.Clear();
}

[System.Serializable]
public class Force
{
    public Vector3 Vector;
    public Color Color;
    public string Name;

    public Force(Vector3 vector, Color color, string name)
    {
        Vector = vector;
        Color = color;
        Name = name;
    }
}


