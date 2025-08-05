using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

public class NodeNetworkValidator : EditorWindow
{
    [MenuItem("Tools/City Organizer/Validate Node Network")]
    public static void ShowWindow()
    {
        GetWindow<NodeNetworkValidator>("Node Network Validator");
    }

    private Vector2 scrollPosition;
    private bool showConnections = true;
    private bool showIsolatedNodes = true;
    private bool showIntersections = true;
    private float connectionDistance = 8f;

    private void OnGUI()
    {
        GUILayout.Label("Node Network Validator", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        // Settings
        GUILayout.Label("Display Options:", EditorStyles.boldLabel);
        showConnections = EditorGUILayout.Toggle("Show Connections", showConnections);
        showIsolatedNodes = EditorGUILayout.Toggle("Show Isolated Nodes", showIsolatedNodes);
        showIntersections = EditorGUILayout.Toggle("Show Intersections", showIntersections);
        
        GUILayout.Space(5);
        connectionDistance = EditorGUILayout.FloatField("Connection Distance", connectionDistance);
        
        GUILayout.Space(10);
        
        // Action buttons
        if (GUILayout.Button("Validate Network", GUILayout.Height(30)))
        {
            ValidateNetwork();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Auto-Fix Intersection Connections", GUILayout.Height(25)))
        {
            AutoFixIntersectionConnections();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Set All Connection Distances", GUILayout.Height(25)))
        {
            SetAllConnectionDistances();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Find Path Issues", GUILayout.Height(25)))
        {
            FindPathIssues();
        }
        
        GUILayout.Space(10);
        
        // Quick stats
        Node[] allNodes = FindObjectsOfType<Node>();
        if (allNodes.Length > 0)
        {
            GUILayout.Label("Network Stats:", EditorStyles.boldLabel);
            GUILayout.Label($"Total Nodes: {allNodes.Length}");
            
            int startNodes = allNodes.Count(n => n.nodeType == NodeType.Start);
            int endNodes = allNodes.Count(n => n.nodeType == NodeType.End);
            int intersections = allNodes.Count(n => n.IsIntersectionNode());
            
            GUILayout.Label($"Start Nodes: {startNodes}");
            GUILayout.Label($"End Nodes: {endNodes}");
            GUILayout.Label($"Intersections: {intersections}");
        }
    }
    
    void ValidateNetwork()
    {
        Node[] allNodes = FindObjectsOfType<Node>();
        
        if (allNodes.Length == 0)
        {
            Debug.LogWarning("No nodes found in scene!");
            return;
        }
        
        Debug.Log("=== NODE NETWORK VALIDATION ===");
        
        List<Node> isolatedNodes = new List<Node>();
        List<Node> intersectionNodes = new List<Node>();
        int totalConnections = 0;
        
        foreach (Node node in allNodes)
        {
            List<Node> neighbors = node.GetNeighbors();
            totalConnections += neighbors.Count;
            
            if (neighbors.Count == 0)
            {
                isolatedNodes.Add(node);
            }
            
            if (node.IsIntersectionNode())
            {
                intersectionNodes.Add(node);
            }
            
            if (showConnections)
            {
                Debug.Log($"Node '{node.name}': {neighbors.Count} connections - {string.Join(", ", neighbors.Select(n => n.name))}");
            }
        }
        
        // Report isolated nodes
        if (isolatedNodes.Count > 0 && showIsolatedNodes)
        {
            Debug.LogWarning($"Found {isolatedNodes.Count} isolated nodes (no connections):");
            foreach (Node isolated in isolatedNodes)
            {
                Debug.LogWarning($"  - {isolated.name} at {isolated.transform.position}");
            }
        }
        
        // Report intersections
        if (showIntersections)
        {
            Debug.Log($"Found {intersectionNodes.Count} intersection nodes:");
            foreach (Node intersection in intersectionNodes)
            {
                Debug.Log($"  - {intersection.name}: {intersection.GetNeighbors().Count} connections");
            }
        }
        
        Debug.Log($"Network Summary: {allNodes.Length} nodes, {totalConnections/2} connections, {isolatedNodes.Count} isolated");
        
        if (isolatedNodes.Count == 0)
        {
            Debug.Log("✓ Network validation passed - no isolated nodes found!");
        }
    }
    
    void AutoFixIntersectionConnections()
    {
        Node[] allNodes = FindObjectsOfType<Node>();
        int fixedConnections = 0;
        
        Debug.Log("=== AUTO-FIXING INTERSECTION CONNECTIONS ===");
        
        foreach (Node node in allNodes)
        {
            if (node.IsIntersectionNode())
            {
                // For intersection nodes, connect to nearby nodes more aggressively
                foreach (Node otherNode in allNodes)
                {
                    if (otherNode == node) continue;
                    
                    float distance = Vector3.Distance(node.transform.position, otherNode.transform.position);
                    
                    // Connect if within extended range and not already connected
                    if (distance <= connectionDistance * 1.5f && !node.GetNeighbors().Contains(otherNode))
                    {
                        // Add manual connection
                        if (!node.manualConnections.Contains(otherNode))
                        {
                            node.manualConnections.Add(otherNode);
                            fixedConnections++;
                            
                            Debug.Log($"Added connection: {node.name} → {otherNode.name} (distance: {distance:F1})");
                        }
                        
                        // Make it bidirectional
                        if (!otherNode.manualConnections.Contains(node))
                        {
                            otherNode.manualConnections.Add(node);
                        }
                    }
                }
                
                // Force refresh neighbors
                node.ForceRefreshNeighbors();
            }
        }
        
        Debug.Log($"Auto-fix complete: Added {fixedConnections} new connections");
        EditorUtility.SetDirty(this);
    }
    
    void SetAllConnectionDistances()
    {
        Node[] allNodes = FindObjectsOfType<Node>();
        
        foreach (Node node in allNodes)
        {
            node.connectionDistance = connectionDistance;
            node.ForceRefreshNeighbors();
            EditorUtility.SetDirty(node);
        }
        
        Debug.Log($"Set connection distance to {connectionDistance} for all {allNodes.Length} nodes");
    }
    
    void FindPathIssues()
    {
        Node[] allNodes = FindObjectsOfType<Node>();
        Node[] startNodes = allNodes.Where(n => n.nodeType == NodeType.Start).ToArray();
        Node[] endNodes = allNodes.Where(n => n.nodeType == NodeType.End).ToArray();
        
        if (startNodes.Length == 0 || endNodes.Length == 0)
        {
            Debug.LogWarning("Need both start and end nodes to test pathfinding!");
            return;
        }
        
        Debug.Log("=== PATHFINDING VALIDATION ===");
        
        int successfulPaths = 0;
        int failedPaths = 0;
        
        foreach (Node startNode in startNodes)
        {
            foreach (Node endNode in endNodes)
            {
                if (startNode == endNode) continue;
                
                List<Node> path = PathFinder.FindPath(startNode, endNode);
                
                if (path != null && path.Count > 0)
                {
                    successfulPaths++;
                    Debug.Log($"✓ Path found: {startNode.name} → {endNode.name} ({path.Count} nodes)");
                }
                else
                {
                    failedPaths++;
                    Debug.LogWarning($"✗ No path: {startNode.name} → {endNode.name}");
                }
            }
        }
        
        Debug.Log($"Pathfinding Summary: {successfulPaths} successful, {failedPaths} failed");
        
        if (failedPaths == 0)
        {
            Debug.Log("✓ All paths working correctly!");
        }
        else
        {
            Debug.LogWarning($"⚠ {failedPaths} path(s) failed - check node connections!");
        }
    }
}