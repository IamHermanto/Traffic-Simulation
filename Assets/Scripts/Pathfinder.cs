using System.Collections.Generic;
using UnityEngine;

public class PathFinder : MonoBehaviour
{
    [Header("Lane Discipline Settings")]
    public static int laneChangePenalty = 200; // MUCH higher penalty for changing lanes
    public static int sameLaneBonus = 50; // Higher bonus for staying in same lane
    public static bool debugLaneSelection = true;
    
    public static List<Node> FindPath(Node startNode, Node endNode)
    {
        // Determine what lane we're starting in
        string startLane = GetNodeLane(startNode);
        string endLane = GetNodeLane(endNode);
        
        if (debugLaneSelection)
        {
            Debug.Log($"Pathfinding: {startLane} → {endLane}");
        }
        
        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        
        openSet.Add(startNode);
        
        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost || 
                    openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost)
                {
                    currentNode = openSet[i];
                }
            }
            
            openSet.Remove(currentNode);
            closedSet.Add(currentNode);
            
            if (currentNode == endNode)
            {
                List<Node> path = RetracePath(startNode, endNode);
                
                if (debugLaneSelection)
                {
                    LogPathLaneUsage(path);
                }
                
                return path;
            }
            
            foreach (Node neighbor in currentNode.GetNeighbors())
            {
                if (closedSet.Contains(neighbor)) continue;
                
                // Calculate lane-aware cost
                int laneAwareCost = GetLaneAwareCost(currentNode, neighbor, startLane, endLane);
                int newCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor) + laneAwareCost;
                
                if (newCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, endNode);
                    neighbor.parent = currentNode;
                    
                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                }
            }
        }
        
        return new List<Node>();
    }
    
    /// <summary>
    /// Determine which lane a node belongs to based on its name or parent
    /// </summary>
    static string GetNodeLane(Node node)
    {
        if (node == null) return "Unknown";
        
        string nodeName = node.name.ToLower();
        
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
        Transform parent = node.transform.parent;
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
        
        return "Center"; // Default for intersection or center nodes
    }
    
    /// <summary>
    /// Calculate additional cost based on lane discipline
    /// </summary>
    static int GetLaneAwareCost(Node currentNode, Node neighborNode, string startLane, string endLane)
    {
        string currentLane = GetNodeLane(currentNode);
        string neighborLane = GetNodeLane(neighborNode);
        
        // Bonus for staying in the same lane
        if (currentLane == neighborLane && currentLane != "Center")
        {
            return -sameLaneBonus; // Negative cost = bonus
        }
        
        // Penalty for changing lanes (unless it's necessary to reach destination)
        if (currentLane != neighborLane && currentLane != "Center" && neighborLane != "Center")
        {
            // If we need to eventually reach the other lane, reduce penalty as we get closer
            if (neighborLane == endLane && currentLane != endLane)
            {
                return laneChangePenalty / 2; // Reduced penalty for necessary lane change
            }
            else
            {
                return laneChangePenalty; // Full penalty for unnecessary lane change
            }
        }
        
        // No additional cost for using center/intersection nodes
        return 0;
    }
    
    /// <summary>
    /// Log which lanes the path uses for debugging
    /// </summary>
    static void LogPathLaneUsage(List<Node> path)
    {
        if (path == null || path.Count == 0) return;
        
        Dictionary<string, int> laneUsage = new Dictionary<string, int>();
        string pathString = "Path: ";
        
        foreach (Node node in path)
        {
            string lane = GetNodeLane(node);
            
            if (!laneUsage.ContainsKey(lane))
            {
                laneUsage[lane] = 0;
            }
            laneUsage[lane]++;
            
            pathString += $"{node.name}({lane}) → ";
        }
        
        Debug.Log(pathString.TrimEnd('→', ' '));
        
        string usageString = "Lane usage: ";
        foreach (var kvp in laneUsage)
        {
            usageString += $"{kvp.Key}: {kvp.Value} nodes, ";
        }
        Debug.Log(usageString.TrimEnd(',', ' '));
    }
    
    static List<Node> RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        return path;
    }
    
    static int GetDistance(Node nodeA, Node nodeB)
    {
        return Mathf.RoundToInt(Vector3.Distance(nodeA.transform.position, nodeB.transform.position) * 10);
    }
    
    /// <summary>
    /// Public method to check if two nodes are in the same lane
    /// </summary>
    public static bool AreNodesInSameLane(Node nodeA, Node nodeB)
    {
        string laneA = GetNodeLane(nodeA);
        string laneB = GetNodeLane(nodeB);
        
        return laneA == laneB && laneA != "Center";
    }
    
    /// <summary>
    /// Find the best start node for a car based on destination lane
    /// </summary>
    public static Node FindBestStartNodeForDestination(Node[] startNodes, Node endNode)
    {
        if (startNodes == null || startNodes.Length == 0 || endNode == null)
            return null;
        
        string endLane = GetNodeLane(endNode);
        
        // First preference: start node in the same lane as destination
        foreach (Node startNode in startNodes)
        {
            if (GetNodeLane(startNode) == endLane)
            {
                if (debugLaneSelection)
                    Debug.Log($"Best start node: {startNode.name} (same lane as destination: {endLane})");
                return startNode;
            }
        }
        
        // Second preference: any start node
        if (debugLaneSelection)
            Debug.Log($"Using fallback start node: {startNodes[0].name} (different lane from destination: {endLane})");
        
        return startNodes[0];
    }
}