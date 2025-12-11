using System;
using System.Collections.Generic;
using UnityEngine;

namespace Group_9
{
    public class ForceVisualizers : MonoBehaviour
    {
        [SerializeField] private List<Force> _forces = new();

        public void AddForce(Vector3 force, Color colorForce, string name)
            => _forces.Add(new Force(force, colorForce, name));

            public void ClearForces() => _forces.Clear();
        
        public void OnDrawGizmos()
        {
            foreach (var force in _forces)
            {
                Gizmos.color = force.ColorForce;
                Vector3 start = transform.position;
                Vector3 end = start + force.Vector;
                
                Gizmos.DrawLine(start, end);
#if UNITY_EDITOR                
                UnityEditor.Handles.Label(end+Vector3.up*0.1f, force.Name);
#endif
            }
        }
    }

    [Serializable]
    public class Force
    {
        public Vector3 Vector;
        public Color ColorForce;
        public string Name;

        public Force(Vector3 vector, Color colorForce, string name)
        {
            Vector = vector;
            ColorForce = colorForce;
            Name = name;
        }
    }
}
