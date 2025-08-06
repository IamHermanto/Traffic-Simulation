using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class LaneRestriction
{
    public string fromLane;  // Which lane you're coming from
    public string toLane;    // Which lane you can go to
    public bool canTurnLeft;
    public bool canTurnRight;
    public bool canGoStraight;
}

public class ImprovedNode : MonoBehaviour
{
    [Header("Node Settings")]
    public NodeType nodeType = NodeType.Normal;
    public float nodeSize = 0.5f;
    
    [Header("Lane Information")]
    public string laneName = "Center"; // Left, Right, or Center
    public LaneType laneType = LaneType.AllDirections; // What movements are allowed
    
    [Header("Lane Restrictions")]
    [Tooltip("Define which lanes can connect through this node")]
    public List<LaneRestriction> laneRestrictions = new List<LaneRestriction>();
    
    [Header("Connection Settings")]
    public float connectionDistance = 8f;
    public List<Node> manualConnections = new List<Node>();
    public bool showConnections = true;
    
    [Header("Traffic Control")]
    public bool isIntersection = false;
    public bool hasTrafficLight = false;
    public float trafficLightCycleDuration = 30f;
    public bool isStopSign = false;
    
    [Header("Speed Limits")]
    public float speedLimit = 50f; // km/h
    public bool isSchoolZone = false;
    public bool isResidentialArea = false;
    
    // Cached connections for performance
    private Dictionary<string, List<Node>> neighborsByLane;
    private float lastCacheUpdate = 0f;
    private float cacheUpdateInterval = 5f;
    
    // Pathfinding variables
    [HideInInspector] public int gCost;
    [HideInInspector] public int hCost;
    [HideInInspector] public Node parent;
    
    public int fCost { get { return gCost + hCost; } }
    
    void Start()
    {
        InitializeLaneInfo();
        RefreshConnections();
    }
    
    void InitializeLaneInfo()
    {
        // Auto-detect lane from name if not set
        if (string.IsNullOrEmpty(laneName) || laneName == "Center")
        {
            string nodeName = name.ToLower();
            
            if (nodeName.Contains("left"))
                laneName = "Left";
            else if (nodeName.Contains("right"))
                laneName = "Right";
            else
                laneName = "Center";
        }
        
        // Set up default lane restrictions if none exist
        if (laneRestrictions.Count == 0)
        {
            SetupDefaultLaneRestrictions();
        }
    }
    
    void SetupDefaultLaneRestrictions()
    {
        // Default: all lanes can go all directions
        if (isIntersection)
        {
            // At intersections, set up realistic lane restrictions
            laneRestrictions.Add(new LaneRestriction
            {
                fromLane = "Left",
                toLane = "Left",
                canGoStraight = true,
                canTurnLeft = true,
                canTurnRight = false
            });
            
            laneRestrictions.Add(new LaneRestriction
            {
                fromLane = "Right",
                toLane = "Right",
                canGoStraight = true,
                canTurnLeft = false,
                canTurnRight = true
            });
        }
        else
        {
            // On regular road segments, stay in lane
            laneRestrictions.Add(new LaneRestriction
            {
                fromLane = laneName,
                toLane = laneName,
                canGoStraight = true,
                canTurnLeft = laneType.AllowsLeftTurn(),
                canTurnRight = laneType.AllowsRightTurn()
            });
        }
    }
    
    public List<Node> GetNeighbors(string fromLane = null)
    {
        if (Time.time - lastCacheUpdate > cacheUpdateInterval)
        {
            RefreshConnections();
        }
        
        if (string.IsNullOrEmpty(fromLane))
        {
            // Return all neighbors
            List<Node> allNeighbors = new List<Node>();
            foreach (var kvp in neighborsByLane)
            {
                allNeighbors.AddRange(kvp.Value);
            }
            return allNeighbors;
        }
        
        // Return neighbors accessible from specific lane
        if (neighborsByLane.ContainsKey(fromLane))
        {
            return new List<Node>(neighborsByLane[fromLane]);
        }
        
        return new List<Node>();
    }
    
    public bool CanTransitionTo(Node targetNode, string fromLane)
    {
        // Check if transition is allowed based on lane restrictions
        foreach (LaneRestriction restriction in laneRestrictions)
        {
            if (restriction.fromLane == fromLane)
            {
                // Check if target node's lane matches allowed transition
                ImprovedNode improvedTarget = targetNode as ImprovedNode;
                if (improvedTarget != null)
                {
                    if (restriction.toLane == improvedTarget.laneName)
                    {
                        // Check direction restrictions
                        Vector3 toTarget = (targetNode.transform.position - transform.position).normalized;
                        float angle = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
                        
                        if (Mathf.Abs(angle) < 30f && restriction.canGoStraight)
                            return true;
                        if (angle < -30f && restriction.canTurnLeft)
                            return true;
                        if (angle > 30f && restriction.canTurnRight)
                            return true;
                    }
                }
            }
        }
        
        // Default to true if no restrictions apply
        return laneRestrictions.Count == 0;
    }
    
    void RefreshConnections()
    {
        neighborsByLane = new Dictionary<string, List<Node>>();
        
        // Initialize dictionary
        neighborsByLane["Left"] = new List<Node>();
        neighborsByLane["Right"] = new List<Node>();
        neighborsByLane["Center"] = new List<Node>();
        
        // Add manual connections first
        foreach (Node manualNode in manualConnections)
        {
            if (manualNode != null && manualNode != this)
            {
                ImprovedNode improvedNode = manualNode as ImprovedNode;
                string targetLane = improvedNode != null ? improvedNode.laneName : "Center";
                
                if (!neighborsByLane.ContainsKey(targetLane))
                    neighborsByLane[targetLane] = new List<Node>();
                    
                neighborsByLane[targetLane].Add(manualNode);
            }
        }
        
        // Find automatic connections
        Node[] allNodes = FindObjectsOfType<Node>();
        foreach (Node node in allNodes)
        {
            if (node == this) continue;
            
            float distance = Vector3.Distance(transform.position, node.transform.position);
            if (distance <= connectionDistance)
            {
                ImprovedNode improvedNode = node as ImprovedNode;
                string targetLane = improvedNode != null ? improvedNode.laneName : "Center";
                
                if (!neighborsByLane.ContainsKey(targetLane))
                    neighborsByLane[targetLane] = new List<Node>();
                    
                if (!neighborsByLane[targetLane].Contains(node))
                {
                    neighborsByLane[targetLane].Add(node);
                }
            }
        }
        
        lastCacheUpdate = Time.time;
    }
    
    public float GetSpeedLimitMPS()
    {
        float speedLimitMPS = speedLimit / 3.6f; // Convert km/h to m/s
        
        if (isSchoolZone)
            speedLimitMPS *= 0.5f;
        if (isResidentialArea)
            speedLimitMPS *= 0.7f;
            
        return speedLimitMPS;
    }
    
    public bool RequiresStop()
    {
        return isStopSign || (hasTrafficLight && !IsGreenLight());
    }
    
    bool IsGreenLight()
    {
        if (!hasTrafficLight) return true;
        
        // Simple traffic light simulation
        float cycleTime = Time.time % trafficLightCycleDuration;
        return cycleTime < trafficLightCycleDuration * 0.4f; // Green for 40% of cycle
    }
    
    void OnDrawGizmos()
    {
        // Draw node with color coding
        Color nodeColor = GetNodeColor();
        Gizmos.color = nodeColor;
        Gizmos.DrawWireSphere(transform.position, nodeSize);
        
        // Draw lane indicator
        DrawLaneIndicator();
        
        // Draw connections if enabled
        if (showConnections)
        {
            DrawConnections();
        }
        
        // Draw traffic control indicators
        if (hasTrafficLight)
        {
            Gizmos.color = IsGreenLight() ? Color.green : Color.red;
            Gizmos.DrawCube(transform.position + Vector3.up * 3f, Vector3.one * 0.5f);
        }
        
        if (isStopSign)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(transform.position + Vector3.up * 2f, Vector3.one * 0.3f);
        }
        
        // Draw speed limit indicator
        #if UNITY_EDITOR
        if (isSchoolZone || isResidentialArea)
        {
            UnityEditor.Handles.Label(transform.position + Vector3.up * 1.5f, 
                $"Speed: {speedLimit} km/h" + (isSchoolZone ? " (School)" : " (Residential)"));
        }
        #endif
    }
    
    Color GetNodeColor()
    {
        switch (nodeType)
        {
            case NodeType.Normal:
                if (isIntersection) return Color.yellow;
                switch (laneName)
                {
                    case "Left": return new Color(0.5f, 0.5f, 1f);
                    case "Right": return new Color(1f, 0.5f, 0.5f);
                    default: return Color.white;
                }
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
        Vector3 pos = transform.position + Vector3.up * 1f;
        
        switch (laneType)
        {
            case LaneType.StraightOnly:
                Gizmos.color = Color.green;
                Gizmos.DrawRay(pos, transform.forward * 1f);
                break;
                
            case LaneType.LeftTurn:
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(pos, -transform.right * 0.7f + transform.forward * 0.7f);
                break;
                
            case LaneType.RightTurn:
                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(pos, transform.right * 0.7f + transform.forward * 0.7f);
                break;
                
            case LaneType.AllDirections:
                Gizmos.color = Color.white;
                Gizmos.DrawWireSphere(pos, 0.3f);
                break;
        }
    }
    
    void DrawConnections()
    {
        if (neighborsByLane == null) return;
        
        foreach (var kvp in neighborsByLane)
        {
            string lane = kvp.Key;
            List<Node> neighbors = kvp.Value;
            
            // Different colors for different lane connections
            switch (lane)
            {
                case "Left":
                    Gizmos.color = new Color(0.5f, 0.5f, 1f, 0.5f);
                    break;
                case "Right":
                    Gizmos.color = new Color(1f, 0.5f, 0.5f, 0.5f);
                    break;
                default:
                    Gizmos.color = new Color(1f, 1f, 1f, 0.3f);
                    break;
            }
            
            foreach (Node neighbor in neighbors)
            {
                if (neighbor != null)
                {
                    Vector3 start = transform.position + Vector3.up * 0.1f;
                    Vector3 end = neighbor.transform.position + Vector3.up * 0.1f;
                    Gizmos.DrawLine(start, end);
                }
            }
        }
    }
}