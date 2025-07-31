using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class TrafficSystemAnalyzer : MonoBehaviour
{
    [Header("Street Detection Settings")]
    [Tooltip("Prefab name patterns to search for")]
    public string[] streetPrefabPatterns = {"Street", "Walkway", "Sideway", "Stoneway"};
    
    [Header("Connection Analysis")]
    [Tooltip("How close street endpoints need to be to connect")]
    public float connectionTolerance = 2f;
    
    [Tooltip("Minimum overlap to consider an intersection")]
    public float intersectionTolerance = 1f;
    
    [Header("Waypoint Generation (Future)")]
    [Tooltip("Distance between waypoints along streets")]
    public float waypointSpacing = 5f;
    
    [Tooltip("Offset from street edges for lane waypoints")]
    public float laneOffset = 3f;
    
    [Header("Debug Visualization")]
    public bool showDebugGizmos = true;
    public Color streetColor = Color.blue;
    public Color connectionColor = Color.green;
    public Color intersectionColor = Color.red;
    public float gizmoSize = 1f;
    
    [Header("Analysis Results")]
    [SerializeField] private List<StreetSegment> detectedStreets = new List<StreetSegment>();
    [SerializeField] private List<StreetSegment> intersections = new List<StreetSegment>();
    [SerializeField] private int totalStreetsFound = 0;
    [SerializeField] private int totalConnections = 0;
    [SerializeField] private int totalIntersections = 0;
    
    // Public Properties
    public List<StreetSegment> DetectedStreets => detectedStreets;
    public List<StreetSegment> Intersections => intersections;
    public bool HasAnalyzedStreets => detectedStreets.Count > 0;
    
    // MAIN ANALYSIS FUNCTION
    [ContextMenu("Analyze Street Network")]
    public void AnalyzeStreetNetwork()
    {
        Debug.Log("=== STARTING STREET NETWORK ANALYSIS ===");
        
        // Clear previous analysis
        ClearAnalysis();
        
        // Step 1: Find all street prefabs in scene
        FindAllStreetPrefabs();
        
        // Step 2: Analyze connections between streets
        AnalyzeStreetConnections();
        
        // Step 3: Identify intersections
        IdentifyIntersections();
        
        // Step 4: Log results
        LogAnalysisResults();
        
        Debug.Log("=== STREET ANALYSIS COMPLETE ===");
    }
    
    private void FindAllStreetPrefabs()
    {
        Debug.Log("Finding street prefabs...");
        
        // Get all GameObjects in scene
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        
        foreach (GameObject obj in allObjects)
        {
            // Check if object name matches any street pattern
            if (IsStreetPrefab(obj.name))
            {
                // Verify it has a mesh collider
                MeshCollider meshCollider = obj.GetComponent<MeshCollider>();
                if (meshCollider != null)
                {
                    StreetSegment street = new StreetSegment(obj);
                    detectedStreets.Add(street);
                    Debug.Log($"Found street: {obj.name} at {street.centerPoint}");
                }
                else
                {
                    Debug.LogWarning($"Street prefab {obj.name} missing MeshCollider!");
                }
            }
        }
        
        totalStreetsFound = detectedStreets.Count;
        Debug.Log($"Total streets found: {totalStreetsFound}");
    }
    
    private bool IsStreetPrefab(string objectName)
    {
        string name = objectName.ToLower();
        
        foreach (string pattern in streetPrefabPatterns)
        {
            if (name.StartsWith(pattern.ToLower()))
            {
                return true;
            }
        }
        
        return false;
    }
    
    private void AnalyzeStreetConnections()
    {
        Debug.Log("Analyzing street connections...");
        
        int connectionCount = 0;
        
        for (int i = 0; i < detectedStreets.Count; i++)
        {
            for (int j = i + 1; j < detectedStreets.Count; j++)
            {
                StreetSegment streetA = detectedStreets[i];
                StreetSegment streetB = detectedStreets[j];
                
                // Check if streets are connected
                if (AreStreetsConnected(streetA, streetB))
                {
                    streetA.AddConnection(streetB);
                    streetB.AddConnection(streetA);
                    connectionCount++;
                    
                    Debug.Log($"Connected: {streetA.streetObject.name} ↔ {streetB.streetObject.name}");
                }
            }
        }
        
        totalConnections = connectionCount;
        Debug.Log($"Total connections found: {connectionCount}");
    }
    
    private bool AreStreetsConnected(StreetSegment streetA, StreetSegment streetB)
    {
        // Method 1: Check if endpoints are close
        float tolerance = connectionTolerance;
        Vector3[] endpointsA = {streetA.startPoint, streetA.endPoint};
        Vector3[] endpointsB = {streetB.startPoint, streetB.endPoint};
        
        foreach (Vector3 pointA in endpointsA)
        {
            foreach (Vector3 pointB in endpointsB)
            {
                float distance = Vector3.Distance(pointA, pointB);
                if (distance <= tolerance)
                {
                    return true;
                }
            }
        }
        
        // Method 2: Check if bounds overlap (for intersections)
        return streetA.bounds.Intersects(streetB.bounds);
    }
    
    private void IdentifyIntersections()
    {
        Debug.Log("Identifying intersections...");
        
        foreach (StreetSegment street in detectedStreets)
        {
            // A street is an intersection if it connects to more than 2 other streets
            if (street.connectionCount > 2)
            {
                street.isIntersection = true;
                intersections.Add(street);
                Debug.Log($"Intersection: {street.streetObject.name} (connects to {street.connectionCount} streets)");
            }
        }
        
        totalIntersections = intersections.Count;
        Debug.Log($"Total intersections found: {totalIntersections}");
    }
    
    private void ClearAnalysis()
    {
        detectedStreets.Clear();
        intersections.Clear();
        totalStreetsFound = 0;
        totalConnections = 0;
        totalIntersections = 0;
    }
    
    private void LogAnalysisResults()
    {
        Debug.Log("=== STREET NETWORK ANALYSIS RESULTS ===");
        Debug.Log($"Streets Found: {totalStreetsFound}");
        Debug.Log($"Connections: {totalConnections}");
        Debug.Log($"Intersections: {totalIntersections}");
        
        // Log each street's connections
        foreach (StreetSegment street in detectedStreets)
        {
            Debug.Log($"{street.streetObject.name}: Connected to [{street.GetConnectionNames()}]");
        }
    }
    
    // PUBLIC UTILITY METHODS
    public StreetSegment GetStreetByName(string name)
    {
        return detectedStreets.FirstOrDefault(s => s.streetObject.name == name);
    }
    
    public List<StreetSegment> GetIntersectionStreets()
    {
        return detectedStreets.Where(s => s.isIntersection).ToList();
    }
    
    public List<StreetSegment> GetDeadEndStreets()
    {
        return detectedStreets.Where(s => s.connectionCount <= 1).ToList();
    }
    
    // DEBUG VISUALIZATION
    private void OnDrawGizmos()
    {
        if (!showDebugGizmos || detectedStreets == null || detectedStreets.Count == 0) 
            return;
        
        // Draw street segments
        Gizmos.color = streetColor;
        foreach (StreetSegment street in detectedStreets)
        {
            if (street?.streetObject != null)
            {
                // Draw street line from start to end
                Gizmos.DrawLine(street.startPoint, street.endPoint);
                
                // Draw street bounds
                Gizmos.DrawWireCube(street.bounds.center, street.bounds.size);
                
                // Draw center point
                Gizmos.DrawSphere(street.centerPoint, gizmoSize);
            }
        }
        
        // Draw connections
        Gizmos.color = connectionColor;
        HashSet<(StreetSegment, StreetSegment)> drawnConnections = new HashSet<(StreetSegment, StreetSegment)>();
        
        foreach (StreetSegment street in detectedStreets)
        {
            foreach (StreetSegment connected in street.connectedStreets)
            {
                // Avoid drawing the same connection twice
                var connection = street.GetHashCode() < connected.GetHashCode() ? 
                    (street, connected) : (connected, street);
                
                if (!drawnConnections.Contains(connection))
                {
                    Gizmos.DrawLine(street.centerPoint, connected.centerPoint);
                    drawnConnections.Add(connection);
                }
            }
        }
        
        // Highlight intersections
        Gizmos.color = intersectionColor;
        foreach (StreetSegment intersection in intersections)
        {
            if (intersection?.streetObject != null)
            {
                Gizmos.DrawSphere(intersection.centerPoint, gizmoSize * 2f);
            }
        }
    }
}