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
    
    [Header("Lane Identification")]
    [Tooltip("Visual indicator for which lane this node belongs to")]
    public bool showLaneInfo = true;
    
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
    private float neighborRefreshInterval = 5f;
    
    public int fCost { get { return gCost + hCost; } }
    
    void Start()
    {
        RefreshNeighbors();
        
        if (debugMode)
        {
            string lane = GetLane();
            Debug.Log($"Node {name} initialized with {GetNeighbors().Count} neighbors, Lane: {lane}");
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
        if (neighborsCached && Time.time - lastNeighborRefresh < neighborRefreshInterval)
        {
            return new List<Node>(cachedNeighbors);
        }
        
        RefreshNeighbors();
        return new List<Node>(cachedNeighbors);
    }
    
    /// <summary>
    /// Get which lane this node belongs to
    /// </summary>
    public string GetLane()
    {
        string nodeName = name.ToLower();
        
        // Check by name prefix
        if (nodeName.StartsWith("leftlane_") || nodeName.Contains("left"))
        {
            return "Left";
        }
        
        if (nodeName.StartsWith("rightlane_") || nodeName.Contains("right"))
        {
            return "Right";
        }
        
        // Check by parent tilemap name
        Transform parent = transform.parent;
        if (parent != null)
        {
            string parentName = parent.name.ToLower();
            if (parentName.Contains("left"))
            {
                return "Left";
            }
            if (parentName.Contains("right"))
            {
                return "Right";
            }
        }
        
        return "Center"; // Intersection or center nodes
    }
    
    /// <summary>
    /// Check if this node is in the same lane as another node
    /// </summary>
    public bool IsInSameLane(Node otherNode)
    {
        if (otherNode == null) return false;
        
        string thisLane = GetLane();
        string otherLane = otherNode.GetLane();
        
        return thisLane == otherLane && thisLane != "Center";
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
        }
    }
    
    bool IsValidConnection(Node otherNode, float distance)
    {
        // Don't connect if too far vertically (different road levels)
        float heightDifference = Mathf.Abs(transform.position.y - otherNode.transform.position.y);
        if (heightDifference > 2f)
        {
            return false;
        }
        
        return distance <= connectionDistance;
    }
    
    public bool IsIntersectionNode()
    {
        if (forceIntersection) return true;
        
        if (neighborsCached)
        {
            return cachedNeighbors.Count >= intersectionThreshold;
        }
        
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
                return Vector3.forward;
            case SpawnDirection.South:
                return Vector3.back;
            case SpawnDirection.East:
                return Vector3.right;
            case SpawnDirection.West:
                return Vector3.left;
            default:
                return Vector3.forward;
        }
    }
    
    [ContextMenu("Debug Lane Info")]
    public void DebugLaneInfo()
    {
        Debug.Log("=== LANE DEBUG INFO ===");
        Debug.Log($"Node: {name}");
        Debug.Log($"Lane: {GetLane()}");
        Debug.Log($"Position: {transform.position}");
        Debug.Log($"Parent: {(transform.parent != null ? transform.parent.name : "None")}");
        
        Debug.Log($"Same-lane neighbors:");
        foreach (Node neighbor in GetNeighbors())
        {
            if (IsInSameLane(neighbor))
            {
                Debug.Log($"  - {neighbor.name} ({neighbor.GetLane()})");
            }
        }
        
        Debug.Log($"Cross-lane connections:");
        foreach (Node neighbor in GetNeighbors())
        {
            if (!IsInSameLane(neighbor) && neighbor.GetLane() != "Center")
            {
                Debug.Log($"  - {neighbor.name} ({neighbor.GetLane()})");
            }
        }
    }
    
    [ContextMenu("Force Refresh Neighbors")]
    public void ForceRefreshNeighbors()
    {
        neighborsCached = false;
        RefreshNeighbors();
        Debug.Log($"Node {name}: Forced neighbor refresh. Found {cachedNeighbors.Count} neighbors.");
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
        
        // Draw lane indicator
        if (showLaneInfo)
        {
            DrawLaneIndicator();
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
    
    void DrawLaneIndicator()
    {
        string lane = GetLane();
        Vector3 pos = transform.position + Vector3.up * 1.2f;
        
        switch (lane)
        {
            case "Left":
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(pos, Vector3.left * 1f);
                break;
                
            case "Right":
                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(pos, Vector3.right * 1f);
                break;
                
            case "Center":
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(pos, 0.3f);
                break;
        }
    }
    
    void DrawConnections()
    {
        List<Node> neighbors = GetNeighbors();
        
        foreach (Node neighbor in neighbors)
        {
            if (neighbor == null) continue;
            
            // Color connections based on whether they're same-lane or cross-lane
            if (IsInSameLane(neighbor))
            {
                Gizmos.color = Color.green; // Same lane connections
            }
            else if (manualConnections.Contains(neighbor))
            {
                Gizmos.color = Color.magenta; // Manual connections
            }
            else if (GetLane() == "Center" || neighbor.GetLane() == "Center")
            {
                Gizmos.color = Color.yellow; // Intersection connections
            }
            else
            {
                Gizmos.color = Color.red; // Cross-lane connections
            }
            
            Vector3 start = transform.position + Vector3.up * 0.1f;
            Vector3 end = neighbor.transform.position + Vector3.up * 0.1f;
            Gizmos.DrawLine(start, end);
        }
    }
    
    void DrawSpawnDirection()
    {
        Vector3 direction = GetSpawnDirection();
        
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
        
        Gizmos.DrawRay(transform.position, direction * 2f);
    }
    
    void DrawNeighborCount()
    {
        #if UNITY_EDITOR
        Vector3 labelPos = transform.position + Vector3.up * 1.5f;
        
        string lane = GetLane();
        string label = $"{GetNeighbors().Count}";
        if (IsIntersectionNode())
        {
            label += " (I)";
        }
        label += $" [{lane}]";
        
        UnityEditor.Handles.Label(labelPos, label);
        #endif
    }
    
    void OnValidate()
    {
        if (Application.isPlaying)
        {
            neighborsCached = false;
        }
    }
}