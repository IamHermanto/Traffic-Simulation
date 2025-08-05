using System.Collections.Generic;
using UnityEngine;

public enum SpawnDirection
{
    North,    // Forward (0, 0, 1)
    South,    // Backward (0, 0, -1)
    East,     // Right (1, 0, 0)
    West      // Left (-1, 0, 0)
}

public class Node : MonoBehaviour
{
    [Header("Node Settings")]
    public NodeType nodeType = NodeType.Normal;
    public float nodeSize = 0.5f;
    
    [Header("Connection Settings")]
    [Tooltip("Maximum distance to automatically connect to other nodes")]
    public float connectionDistance = 8f;
    [Tooltip("Force connect to these specific nodes regardless of distance")]
    public List<Node> manualConnections = new List<Node>();
    [Tooltip("Show connection lines in Scene view")]
    public bool showConnections = true;
    [Tooltip("Show neighbor count above node")]
    public bool showNeighborCount = true;
    
    [Header("Spawn Direction (For Start Nodes Only)")]
    [Tooltip("Direction cars will face when spawning at this node")]
    public SpawnDirection spawnDirection = SpawnDirection.North;
    
    [Header("Intersection Detection")]
    [Tooltip("Override automatic intersection detection")]
    public bool forceIntersection = false;
    [Tooltip("Minimum neighbors to be considered an intersection")]
    public int intersectionThreshold = 3;
    
    [Header("Debug Info")]
    [Tooltip("Enable detailed logging for this node")]
    public bool debugMode = false;
    
    // Pathfinding variables
    [HideInInspector] public int gCost;
    [HideInInspector] public int hCost;
    [HideInInspector] public Node parent;
    
    // Cache for performance
    private List<Node> cachedNeighbors = new List<Node>();
    private bool neighborsCached = false;
    private float lastNeighborRefresh = 0f;
    private float neighborRefreshInterval = 5f; // Refresh every 5 seconds
    
    public int fCost { get { return gCost + hCost; } }
    
    void Start()
    {
        RefreshNeighbors();
        
        if (debugMode)
        {
            Debug.Log($"Node {name} initialized with {GetNeighbors().Count} neighbors");
        }
    }
    
    void Update()
    {
        // Periodically refresh neighbor cache
        if (Time.time - lastNeighborRefresh > neighborRefreshInterval)
        {
            RefreshNeighbors();
        }
    }
    
    public List<Node> GetNeighbors()
    {
        // Return cached neighbors if available and recent
        if (neighborsCached && Time.time - lastNeighborRefresh < neighborRefreshInterval)
        {
            return new List<Node>(cachedNeighbors);
        }
        
        RefreshNeighbors();
        return new List<Node>(cachedNeighbors);
    }
    
    void RefreshNeighbors()
    {
        cachedNeighbors.Clear();
        
        // Add manual connections first
        foreach (Node manualNode in manualConnections)
        {
            if (manualNode != null && manualNode != this && !cachedNeighbors.Contains(manualNode))
            {
                cachedNeighbors.Add(manualNode);
            }
        }
        
        // Find automatic connections
        Node[] allNodes = FindObjectsOfType<Node>();
        foreach (Node node in allNodes)
        {
            if (node == this || cachedNeighbors.Contains(node)) continue;
            
            float distance = Vector3.Distance(transform.position, node.transform.position);
            if (distance <= connectionDistance)
            {
                // Simple validation without calling IsIntersectionNode() to avoid stack overflow
                if (IsValidConnection(node, distance))
                {
                    cachedNeighbors.Add(node);
                }
            }
        }
        
        neighborsCached = true;
        lastNeighborRefresh = Time.time;
        
        if (debugMode)
        {
            Debug.Log($"Node {name} refreshed: {cachedNeighbors.Count} neighbors found");
            foreach (Node neighbor in cachedNeighbors)
            {
                Debug.Log($"  → Connected to: {neighbor.name} (distance: {Vector3.Distance(transform.position, neighbor.transform.position):F1})");
            }
        }
    }
    
    bool IsValidConnection(Node otherNode, float distance)
    {
        // Basic validation without circular dependencies
        
        // Don't connect if too far vertically (different road levels)
        float heightDifference = Mathf.Abs(transform.position.y - otherNode.transform.position.y);
        if (heightDifference > 2f)
        {
            return false;
        }
        
        // Simple distance check - no intersection logic here to avoid recursion
        return distance <= connectionDistance;
    }
    
    public bool IsIntersectionNode()
    {
        if (forceIntersection) return true;
        
        // Use cached neighbors count to avoid recursion
        if (neighborsCached)
        {
            return cachedNeighbors.Count >= intersectionThreshold;
        }
        
        // If not cached, do a simple count without full neighbor refresh
        return GetBasicNeighborCount() >= intersectionThreshold;
    }
    
    private int GetBasicNeighborCount()
    {
        int count = manualConnections.Count;
        
        Node[] allNodes = FindObjectsOfType<Node>();
        foreach (Node node in allNodes)
        {
            if (node == this) continue;
            
            float distance = Vector3.Distance(transform.position, node.transform.position);
            if (distance <= connectionDistance)
            {
                float heightDifference = Mathf.Abs(transform.position.y - node.transform.position.y);
                if (heightDifference <= 2f)
                {
                    count++;
                }
            }
        }
        
        return count;
    }
    
    /// <summary>
    /// Gets the spawn direction for cars spawning at this node
    /// </summary>
    public Vector3 GetSpawnDirection()
    {
        Vector3 direction = ConvertSpawnDirectionToVector(spawnDirection);
        
        if (debugMode)
        {
            Debug.Log($"Node {gameObject.name}: GetSpawnDirection called. Enum: {spawnDirection}, Vector: {direction}");
        }
        
        return direction;
    }
    
    Vector3 ConvertSpawnDirectionToVector(SpawnDirection direction)
    {
        switch (direction)
        {
            case SpawnDirection.North:
                return Vector3.forward;    // (0, 0, 1)
            case SpawnDirection.South:
                return Vector3.back;       // (0, 0, -1)
            case SpawnDirection.East:
                return Vector3.right;      // (1, 0, 0)
            case SpawnDirection.West:
                return Vector3.left;       // (-1, 0, 0)
            default:
                return Vector3.forward;
        }
    }
    
    [ContextMenu("Force Refresh Neighbors")]
    public void ForceRefreshNeighbors()
    {
        neighborsCached = false;
        RefreshNeighbors();
        Debug.Log($"Node {name}: Forced neighbor refresh. Found {cachedNeighbors.Count} neighbors.");
    }
    
    [ContextMenu("Debug Node Info")]
    public void DebugNodeInfo()
    {
        Debug.Log("=== NODE DEBUG INFO ===");
        Debug.Log($"Node: {name}");
        Debug.Log($"Position: {transform.position}");
        Debug.Log($"Type: {nodeType}");
        Debug.Log($"Is Intersection: {IsIntersectionNode()}");
        Debug.Log($"Connection Distance: {connectionDistance}");
        Debug.Log($"Neighbors ({GetNeighbors().Count}):");
        
        foreach (Node neighbor in GetNeighbors())
        {
            float dist = Vector3.Distance(transform.position, neighbor.transform.position);
            Debug.Log($"  → {neighbor.name} (distance: {dist:F1}, type: {neighbor.nodeType})");
        }
        
        Debug.Log($"Manual Connections ({manualConnections.Count}):");
        foreach (Node manual in manualConnections)
        {
            if (manual != null)
            {
                float dist = Vector3.Distance(transform.position, manual.transform.position);
                Debug.Log($"  → {manual.name} (distance: {dist:F1}) [MANUAL]");
            }
        }
    }
    
    [ContextMenu("Find Nearest Intersection")]
    public void FindNearestIntersection()
    {
        Node[] allNodes = FindObjectsOfType<Node>();
        Node nearestIntersection = null;
        float nearestDistance = float.MaxValue;
        
        foreach (Node node in allNodes)
        {
            if (node != this && node.IsIntersectionNode())
            {
                float distance = Vector3.Distance(transform.position, node.transform.position);
                if (distance < nearestDistance)
                {
                    nearestDistance = distance;
                    nearestIntersection = node;
                }
            }
        }
        
        if (nearestIntersection != null)
        {
            Debug.Log($"Nearest intersection to {name}: {nearestIntersection.name} (distance: {nearestDistance:F1})");
        }
        else
        {
            Debug.Log($"No intersections found near {name}");
        }
    }
    
    void OnDrawGizmos()
    {
        // Draw node sphere with color coding
        Color nodeColor = GetNodeColor();
        Gizmos.color = nodeColor;
        Gizmos.DrawWireSphere(transform.position, nodeSize);
        
        // Fill sphere for intersections
        if (IsIntersectionNode())
        {
            Gizmos.color = new Color(nodeColor.r, nodeColor.g, nodeColor.b, 0.3f);
            Gizmos.DrawSphere(transform.position, nodeSize * 0.8f);
        }
        
        // Draw connections
        if (showConnections)
        {
            DrawConnections();
        }
        
        // Draw spawn direction for start nodes
        if (nodeType == NodeType.Start)
        {
            DrawSpawnDirection();
        }
        
        // Draw neighbor count
        if (showNeighborCount)
        {
            DrawNeighborCount();
        }
    }
    
    Color GetNodeColor()
    {
        switch (nodeType)
        {
            case NodeType.Normal:
                return IsIntersectionNode() ? Color.yellow : Color.white;
            case NodeType.Start:
                return Color.green;
            case NodeType.End:
                return Color.red;
            default:
                return Color.gray;
        }
    }
    
    void DrawConnections()
    {
        List<Node> neighbors = GetNeighbors();
        
        foreach (Node neighbor in neighbors)
        {
            if (neighbor == null) continue;
            
            // Different colors for different connection types
            if (manualConnections.Contains(neighbor))
            {
                Gizmos.color = Color.magenta; // Manual connections
            }
            else if (IsIntersectionNode() || neighbor.IsIntersectionNode())
            {
                Gizmos.color = new Color(1f, 0.5f, 0f); // Orange for intersection connections
            }
            else
            {
                Gizmos.color = Color.cyan; // Regular connections
            }
            
            // Draw line with slight offset to avoid z-fighting
            Vector3 start = transform.position + Vector3.up * 0.1f;
            Vector3 end = neighbor.transform.position + Vector3.up * 0.1f;
            Gizmos.DrawLine(start, end);
            
            // Draw arrow for direction
            Vector3 direction = (end - start).normalized;
            Vector3 arrowPoint = end - direction * 0.5f;
            Vector3 arrowSide1 = arrowPoint + Vector3.Cross(direction, Vector3.up) * 0.2f;
            Vector3 arrowSide2 = arrowPoint - Vector3.Cross(direction, Vector3.up) * 0.2f;
            
            Gizmos.DrawLine(end, arrowSide1);
            Gizmos.DrawLine(end, arrowSide2);
        }
    }
    
    void DrawSpawnDirection()
    {
        Vector3 direction = GetSpawnDirection();
        
        // Color code the arrow based on direction
        switch (spawnDirection)
        {
            case SpawnDirection.North:
                Gizmos.color = Color.blue;
                break;
            case SpawnDirection.South:
                Gizmos.color = Color.magenta;
                break;
            case SpawnDirection.East:
                Gizmos.color = Color.green;
                break;
            case SpawnDirection.West:
                Gizmos.color = Color.red;
                break;
        }
        
        // Draw spawn direction arrow
        Gizmos.DrawRay(transform.position, direction * 2f);
        
        // Draw arrow head
        Vector3 arrowHead1 = direction * 1.5f + Vector3.Cross(direction, Vector3.up) * 0.3f;
        Vector3 arrowHead2 = direction * 1.5f - Vector3.Cross(direction, Vector3.up) * 0.3f;
        Gizmos.DrawLine(transform.position + direction * 2f, transform.position + arrowHead1);
        Gizmos.DrawLine(transform.position + direction * 2f, transform.position + arrowHead2);
    }
    
    void DrawNeighborCount()
    {
        #if UNITY_EDITOR
        Vector3 labelPos = transform.position + Vector3.up * 1.5f;
        string label = $"{GetNeighbors().Count}";
        if (IsIntersectionNode())
        {
            label += " (I)"; // Mark intersections
        }
        UnityEditor.Handles.Label(labelPos, label);
        #endif
    }
    
    void OnValidate()
    {
        // Refresh neighbors when values change in inspector
        if (Application.isPlaying)
        {
            neighborsCached = false;
        }
    }
}