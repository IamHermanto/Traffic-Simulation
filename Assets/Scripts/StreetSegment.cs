using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public class StreetSegment
{
    [Header("Street Object")]
    public GameObject streetObject;
    
    [Header("Spatial Data")]
    public Vector3 startPoint;      // One end of the street
    public Vector3 endPoint;        // Other end of the street  
    public Vector3 centerPoint;     // Middle of the street
    public Vector3 direction;       // Normalized direction vector
    
    [Header("Dimensions")]
    public float length;            // Street length
    public float width;             // Street width
    public Bounds bounds;           // Collider bounds
    
    [Header("Network Data")]
    public List<StreetSegment> connectedStreets = new List<StreetSegment>();
    public bool isIntersection = false;
    
    [Header("Traffic Data")]
    public int connectionCount => connectedStreets.Count;
    
    public StreetSegment(GameObject obj)
    {
        streetObject = obj;
        AnalyzeStreetDimensions();
    }
    
    private void AnalyzeStreetDimensions()
    {
        if (streetObject == null) return;
        
        // Get the mesh collider bounds
        MeshCollider meshCollider = streetObject.GetComponent<MeshCollider>();
        if (meshCollider != null)
        {
            bounds = meshCollider.bounds;
            centerPoint = bounds.center;
            
            // Since streets run along Z-axis (forward/backward)
            // Length is the Z-dimension, Width is the X-dimension
            length = bounds.size.z;
            width = bounds.size.x;
            
            // Calculate start and end points along Z-axis
            Vector3 localForward = streetObject.transform.forward;
            float halfLength = length * 0.5f;
            
            startPoint = centerPoint - localForward * halfLength;
            endPoint = centerPoint + localForward * halfLength;
            direction = localForward;
        }
        else
        {
            Debug.LogWarning($"StreetSegment: {streetObject.name} missing MeshCollider!");
        }
    }
    
    // Utility Methods
    public bool IsConnectedTo(StreetSegment other)
    {
        return connectedStreets.Contains(other);
    }
    
    public void AddConnection(StreetSegment other)
    {
        if (!connectedStreets.Contains(other))
        {
            connectedStreets.Add(other);
        }
    }
    
    public void RemoveConnection(StreetSegment other)
    {
        connectedStreets.Remove(other);
    }
    
    public string GetConnectionNames()
    {
        List<string> names = new List<string>();
        foreach (StreetSegment street in connectedStreets)
        {
            if (street?.streetObject != null)
            {
                names.Add(street.streetObject.name);
            }
        }
        return string.Join(", ", names);
    }
    
    public override string ToString()
    {
        return $"Street: {streetObject?.name} | Connections: {connectionCount} | Intersection: {isIntersection}";
    }
}