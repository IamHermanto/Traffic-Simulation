using UnityEngine;

public class NodeCreator : MonoBehaviour
{
    [Header("Lane Tilemaps")]
    [Tooltip("Tilemap containing left lane road tiles")]
    public Transform leftLaneTilemap;
    
    [Tooltip("Tilemap containing right lane road tiles")]
    public Transform rightLaneTilemap;
    
    [Header("Node Settings")]
    [Tooltip("Connection distance for generated nodes")]
    public float connectionDistance = 8f;
    
    [Header("Naming Convention")]
    [Tooltip("Prefix for left lane node names")]
    public string leftLanePrefix = "LeftLane_";
    
    [Tooltip("Prefix for right lane node names")]
    public string rightLanePrefix = "RightLane_";
    
    [ContextMenu("Create Nodes on Both Lanes")]
    void CreateNodes()
    {
        int totalNodesCreated = 0;
        
        // Create nodes on left lane
        if (leftLaneTilemap != null)
        {
            int leftNodes = CreateNodesOnTilemap(leftLaneTilemap, leftLanePrefix, "Left");
            totalNodesCreated += leftNodes;
        }
        else
        {
            Debug.LogWarning("Left lane tilemap reference is null!");
        }
        
        // Create nodes on right lane
        if (rightLaneTilemap != null)
        {
            int rightNodes = CreateNodesOnTilemap(rightLaneTilemap, rightLanePrefix, "Right");
            totalNodesCreated += rightNodes;
        }
        else
        {
            Debug.LogWarning("Right lane tilemap reference is null!");
        }
        
        Debug.Log($"=== NODE CREATION COMPLETE ===");
        Debug.Log($"Total nodes created: {totalNodesCreated}");
        Debug.Log($"Left lane nodes: {GetNodeCount(leftLaneTilemap)}");
        Debug.Log($"Right lane nodes: {GetNodeCount(rightLaneTilemap)}");
    }
    
    [ContextMenu("Create Left Lane Nodes Only")]
    void CreateLeftLaneNodes()
    {
        if (leftLaneTilemap == null)
        {
            Debug.LogError("Left lane tilemap reference is null!");
            return;
        }
        
        int nodesCreated = CreateNodesOnTilemap(leftLaneTilemap, leftLanePrefix, "Left");
        Debug.Log($"Created {nodesCreated} left lane nodes");
    }
    
    [ContextMenu("Create Right Lane Nodes Only")]
    void CreateRightLaneNodes()
    {
        if (rightLaneTilemap == null)
        {
            Debug.LogError("Right lane tilemap reference is null!");
            return;
        }
        
        int nodesCreated = CreateNodesOnTilemap(rightLaneTilemap, rightLanePrefix, "Right");
        Debug.Log($"Created {nodesCreated} right lane nodes");
    }
    
    int CreateNodesOnTilemap(Transform tilemap, string namePrefix, string laneDescription)
    {
        int nodesCreated = 0;
        
        Debug.Log($"Creating nodes on {laneDescription} lane tilemap: {tilemap.name}");
        
        foreach (Transform child in tilemap)
        {
            Node existingNode = child.GetComponent<Node>();
            
            if (existingNode == null)
            {
                // Create new node
                Node newNode = child.gameObject.AddComponent<Node>();
                
                // Configure the node (no lane type restrictions for now)
                newNode.connectionDistance = connectionDistance;
                newNode.showLaneInfo = true;
                newNode.showConnections = true;
                
                // Set appropriate name
                if (!child.name.StartsWith(namePrefix))
                {
                    child.name = namePrefix + child.name;
                }
                
                nodesCreated++;
                
                Debug.Log($"Created {laneDescription} node: {child.name} (Lane: {newNode.GetLane()})");
            }
            else
            {
                // Update existing node
                existingNode.connectionDistance = connectionDistance;
                
                // Update name if needed
                if (!child.name.StartsWith(namePrefix))
                {
                    child.name = namePrefix + child.name;
                }
                
                Debug.Log($"Updated existing {laneDescription} node: {child.name} (Lane: {existingNode.GetLane()})");
            }
        }
        
        return nodesCreated;
    }
    
    int GetNodeCount(Transform tilemap)
    {
        if (tilemap == null) return 0;
        
        int count = 0;
        foreach (Transform child in tilemap)
        {
            if (child.GetComponent<Node>() != null)
            {
                count++;
            }
        }
        return count;
    }
    
    [ContextMenu("Clear All Nodes")]
    void ClearAllNodes()
    {
        int removedCount = 0;
        
        // Clear left lane nodes
        if (leftLaneTilemap != null)
        {
            removedCount += ClearNodesFromTilemap(leftLaneTilemap, "Left");
        }
        
        // Clear right lane nodes  
        if (rightLaneTilemap != null)
        {
            removedCount += ClearNodesFromTilemap(rightLaneTilemap, "Right");
        }
        
        Debug.Log($"Removed {removedCount} nodes total");
    }
    
    int ClearNodesFromTilemap(Transform tilemap, string laneDescription)
    {
        int removedCount = 0;
        
        foreach (Transform child in tilemap)
        {
            Node node = child.GetComponent<Node>();
            if (node != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(node);
                }
                else
                {
                    DestroyImmediate(node);
                }
                removedCount++;
            }
        }
        
        Debug.Log($"Removed {removedCount} {laneDescription} lane nodes");
        return removedCount;
    }
    
    [ContextMenu("Debug Lane Setup")]
    void DebugLaneSetup()
    {
        Debug.Log("=== LANE SETUP DEBUG ===");
        
        if (leftLaneTilemap != null)
        {
            Debug.Log($"Left Lane Tilemap: {leftLaneTilemap.name}");
            Debug.Log($"  - Children: {leftLaneTilemap.childCount}");
            Debug.Log($"  - Nodes: {GetNodeCount(leftLaneTilemap)}");
            Debug.Log($"  - Lane Identification: By name prefix '{leftLanePrefix}'");
        }
        else
        {
            Debug.LogWarning("Left Lane Tilemap: NOT SET");
        }
        
        if (rightLaneTilemap != null)
        {
            Debug.Log($"Right Lane Tilemap: {rightLaneTilemap.name}");
            Debug.Log($"  - Children: {rightLaneTilemap.childCount}");  
            Debug.Log($"  - Nodes: {GetNodeCount(rightLaneTilemap)}");
            Debug.Log($"  - Lane Identification: By name prefix '{rightLanePrefix}'");
        }
        else
        {
            Debug.LogWarning("Right Lane Tilemap: NOT SET");
        }
        
        Debug.Log($"Connection Distance: {connectionDistance}");
    }
    
    [ContextMenu("Validate Lane Connections")]
    void ValidateLaneConnections()
    {
        Debug.Log("=== VALIDATING LANE CONNECTIONS ===");
        
        // Check left lane nodes
        if (leftLaneTilemap != null)
        {
            ValidateConnectionsForTilemap(leftLaneTilemap, "Left");
        }
        
        // Check right lane nodes
        if (rightLaneTilemap != null)
        {
            ValidateConnectionsForTilemap(rightLaneTilemap, "Right");
        }
    }
    
    void ValidateConnectionsForTilemap(Transform tilemap, string laneDescription)
    {
        int isolatedNodes = 0;
        int connectedNodes = 0;
        
        foreach (Transform child in tilemap)
        {
            Node node = child.GetComponent<Node>();
            if (node != null)
            {
                node.ForceRefreshNeighbors();
                int neighborCount = node.GetNeighbors().Count;
                
                if (neighborCount == 0)
                {
                    isolatedNodes++;
                    Debug.LogWarning($"Isolated {laneDescription} node: {child.name}");
                }
                else
                {
                    connectedNodes++;
                }
            }
        }
        
        Debug.Log($"{laneDescription} Lane Validation:");
        Debug.Log($"  - Connected nodes: {connectedNodes}");
        Debug.Log($"  - Isolated nodes: {isolatedNodes}");
    }
}