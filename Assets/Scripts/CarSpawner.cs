using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class CarSpawner : MonoBehaviour
{
    public GameObject carPrefab;
    
    [Header("Spawn Settings")]
    public float spawnHeight = 0.5f;
    public bool spawnOnStart = true;
    public float spawnInterval = 5f;
    public bool continuousSpawning = true;
    public int maxCarsInScene = 10;
    
    [Header("Lane-Aware Spawning")]
    [Tooltip("Prefer to spawn cars on the correct lane for their destination")]
    public bool useLaneAwareSpawning = true;
    [Tooltip("Only allow same-lane trips (no cross-lane destinations)")]
    public bool strictSameLaneOnly = true;
    [Tooltip("Debug lane selection logic")]
    public bool debugLaneSpawning = true;
    
    [Header("Simulation Settings")]
    public bool randomizeDestinations = true;
    
    // Cache for performance
    private List<Node> cachedStartNodes = new List<Node>();
    private List<Node> cachedEndNodes = new List<Node>();
    private Dictionary<string, List<Node>> startNodesByLane = new Dictionary<string, List<Node>>();
    private Dictionary<string, List<Node>> endNodesByLane = new Dictionary<string, List<Node>>();
    
    private float cacheRefreshInterval = 10f;
    private float lastCacheRefresh = 0f;
    
    void Start()
    {
        Debug.Log("Lane-Aware CarSpawner Start() - Setting up intelligent lane spawning");
        
        RefreshNodeCache();
        
        if (spawnOnStart)
        {
            SpawnCarsOnAllStartNodes();
        }
        
        if (continuousSpawning)
        {
            InvokeRepeating(nameof(SpawnRandomCar), spawnInterval, spawnInterval);
        }
    }
    
    void Update()
    {
        if (Time.time - lastCacheRefresh > cacheRefreshInterval)
        {
            RefreshNodeCache();
        }
    }
    
    void RefreshNodeCache()
    {
        cachedStartNodes.Clear();
        cachedEndNodes.Clear();
        startNodesByLane.Clear();
        endNodesByLane.Clear();
        
        Node[] allNodes = FindObjectsOfType<Node>();
        
        foreach (Node node in allNodes)
        {
            if (node.nodeType == NodeType.Start) 
            {
                cachedStartNodes.Add(node);
                
                // Group by lane
                string lane = GetNodeLane(node);
                if (!startNodesByLane.ContainsKey(lane))
                    startNodesByLane[lane] = new List<Node>();
                startNodesByLane[lane].Add(node);
            }
            
            if (node.nodeType == NodeType.End) 
            {
                cachedEndNodes.Add(node);
                
                // Group by lane
                string lane = GetNodeLane(node);
                if (!endNodesByLane.ContainsKey(lane))
                    endNodesByLane[lane] = new List<Node>();
                endNodesByLane[lane].Add(node);
            }
        }
        
        lastCacheRefresh = Time.time;
        
        if (debugLaneSpawning)
        {
            Debug.Log($"=== NODE CACHE REFRESHED ===");
            Debug.Log($"Total start nodes: {cachedStartNodes.Count}");
            Debug.Log($"Total end nodes: {cachedEndNodes.Count}");
            
            foreach (var kvp in startNodesByLane)
            {
                Debug.Log($"Start nodes in {kvp.Key} lane: {kvp.Value.Count}");
            }
            
            foreach (var kvp in endNodesByLane)
            {
                Debug.Log($"End nodes in {kvp.Key} lane: {kvp.Value.Count}");
            }
        }
    }
    
    void SpawnRandomCar()
    {
        // Check car limit
        RealisticCar[] existingCars = FindObjectsOfType<RealisticCar>();
        if (existingCars.Length >= maxCarsInScene)
        {
            if (debugLaneSpawning)
                Debug.Log($"Max cars ({maxCarsInScene}) reached, skipping spawn");
            return;
        }
        
        if (cachedStartNodes.Count == 0 || cachedEndNodes.Count == 0)
        {
            Debug.LogWarning("No start or end nodes available for spawning!");
            return;
        }
        
        Node startNode = null;
        Node endNode = null;
        
        if (strictSameLaneOnly)
        {
            // STRICT: Only same-lane trips
            if (!TryFindSameLaneTrip(out startNode, out endNode))
            {
                if (debugLaneSpawning)
                    Debug.Log("No same-lane trips available, skipping spawn");
                return;
            }
        }
        else if (useLaneAwareSpawning)
        {
            // FLEXIBLE: Prefer same-lane, allow cross-lane as fallback
            endNode = cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
            startNode = FindBestStartNodeForDestination(endNode);
        }
        else
        {
            // RANDOM: Pick random start and end
            startNode = cachedStartNodes[Random.Range(0, cachedStartNodes.Count)];
            endNode = cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
        }
        
        // Make sure start and end are different
        if (startNode == endNode && cachedEndNodes.Count > 1)
        {
            do {
                endNode = cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
            } while (endNode == startNode);
        }
        
        SpawnCarAtNode(startNode, endNode);
    }
    
    /// <summary>
    /// Try to find a same-lane trip (strict mode)
    /// </summary>
    bool TryFindSameLaneTrip(out Node startNode, out Node endNode)
    {
        startNode = null;
        endNode = null;
        
        // Get available lanes that have both start and end nodes
        List<string> availableLanes = new List<string>();
        
        foreach (var lane in startNodesByLane.Keys)
        {
            if (endNodesByLane.ContainsKey(lane) && 
                startNodesByLane[lane].Count > 0 && 
                endNodesByLane[lane].Count > 0)
            {
                availableLanes.Add(lane);
            }
        }
        
        if (availableLanes.Count == 0)
        {
            if (debugLaneSpawning)
                Debug.LogWarning("No lanes have both start and end nodes available!");
            return false;
        }
        
        // Pick a random lane
        string chosenLane = availableLanes[Random.Range(0, availableLanes.Count)];
        
        // Pick random start and end from that lane
        startNode = startNodesByLane[chosenLane][Random.Range(0, startNodesByLane[chosenLane].Count)];
        endNode = endNodesByLane[chosenLane][Random.Range(0, endNodesByLane[chosenLane].Count)];
        
        // Make sure they're different
        if (startNode == endNode && endNodesByLane[chosenLane].Count > 1)
        {
            do {
                endNode = endNodesByLane[chosenLane][Random.Range(0, endNodesByLane[chosenLane].Count)];
            } while (endNode == startNode);
        }
        
        if (debugLaneSpawning)
            Debug.Log($"✓ STRICT same-lane trip: {startNode.name}({chosenLane}) → {endNode.name}({chosenLane})");
        
        return true;
    }
    
    /// <summary>
    /// Find the best start node for a given destination (lane-aware)
    /// </summary>
    Node FindBestStartNodeForDestination(Node destination)
    {
        string destinationLane = GetNodeLane(destination);
        
        // First try: start nodes in the same lane as destination
        if (startNodesByLane.ContainsKey(destinationLane) && startNodesByLane[destinationLane].Count > 0)
        {
            Node sameLineStart = startNodesByLane[destinationLane][Random.Range(0, startNodesByLane[destinationLane].Count)];
            
            if (debugLaneSpawning)
                Debug.Log($"✓ Found same-lane start: {sameLineStart.name} ({destinationLane}) → {destination.name} ({destinationLane})");
            
            return sameLineStart;
        }
        
        // Fallback: any available start node
        Node fallbackStart = cachedStartNodes[Random.Range(0, cachedStartNodes.Count)];
        
        if (debugLaneSpawning)
            Debug.Log($"⚠ Using cross-lane spawn: {fallbackStart.name} ({GetNodeLane(fallbackStart)}) → {destination.name} ({destinationLane})");
        
        return fallbackStart;
    }
    
    void SpawnCarsOnAllStartNodes()
    {
        if (cachedEndNodes.Count == 0)
        {
            Debug.LogWarning("No end nodes found! Cannot spawn cars.");
            return;
        }
        
        int spawnedCount = 0;
        
        if (strictSameLaneOnly)
        {
            // STRICT: Only spawn on lanes that have both start and end nodes
            foreach (var laneKvp in startNodesByLane)
            {
                string lane = laneKvp.Key;
                List<Node> laneStartNodes = laneKvp.Value;
                
                // Check if this lane has end nodes
                if (!endNodesByLane.ContainsKey(lane) || endNodesByLane[lane].Count == 0)
                {
                    if (debugLaneSpawning)
                        Debug.LogWarning($"Skipping {lane} lane - no end nodes available");
                    continue;
                }
                
                // Spawn cars on each start node in this lane
                foreach (Node startNode in laneStartNodes)
                {
                    // Pick random end node from same lane
                    List<Node> laneEndNodes = endNodesByLane[lane];
                    Node targetEndNode = laneEndNodes[Random.Range(0, laneEndNodes.Count)];
                    
                    // Make sure start and end are different
                    if (startNode == targetEndNode && laneEndNodes.Count > 1)
                    {
                        do {
                            targetEndNode = laneEndNodes[Random.Range(0, laneEndNodes.Count)];
                        } while (targetEndNode == startNode);
                    }
                    
                    SpawnCarAtNode(startNode, targetEndNode);
                    spawnedCount++;
                }
            }
        }
        else
        {
            // FLEXIBLE: Original behavior
            foreach (Node startNode in cachedStartNodes)
            {
                Node targetEndNode;
                
                if (useLaneAwareSpawning)
                {
                    // Find best destination for this start node
                    targetEndNode = FindBestDestinationForStart(startNode);
                }
                else
                {
                    // Random destination
                    targetEndNode = cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
                }
                
                SpawnCarAtNode(startNode, targetEndNode);
                spawnedCount++;
            }
        }
        
        if (debugLaneSpawning)
        {
            Debug.Log($"Spawned {spawnedCount} cars with strict same-lane policy");
        }
    }
    
    /// <summary>
    /// Find the best destination for a given start node (prefer same lane)
    /// </summary>
    Node FindBestDestinationForStart(Node startNode)
    {
        string startLane = GetNodeLane(startNode);
        
        // First try: end nodes in the same lane
        if (endNodesByLane.ContainsKey(startLane) && endNodesByLane[startLane].Count > 0)
        {
            return endNodesByLane[startLane][Random.Range(0, endNodesByLane[startLane].Count)];
        }
        
        // Fallback: any end node
        return cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
    }
    
    void SpawnCarAtNode(Node startNode, Node endNode)
    {
        if (startNode == null || endNode == null) 
        {
            Debug.LogWarning("SpawnCarAtNode: Invalid start or end node");
            return;
        }
        
        // Get spawn direction and position
        Vector3 spawnDirection = startNode.GetSpawnDirection();
        Vector3 spawnPosition = startNode.transform.position + Vector3.up * spawnHeight;
        Quaternion spawnRotation = spawnDirection.magnitude > 0.1f ? 
            Quaternion.LookRotation(spawnDirection) : Quaternion.identity;
        
        // Spawn car
        GameObject car = Instantiate(carPrefab, spawnPosition, spawnRotation);
        
        // Find path using lane-aware pathfinding
        List<Node> path = PathFinder.FindPath(startNode, endNode);
        
        if (path != null && path.Count > 0)
        {
            RealisticCar carComponent = car.GetComponent<RealisticCar>();
            if (carComponent != null)
            {
                carComponent.SetPathWithoutAlignment(path);
                
                if (debugLaneSpawning)
                {
                    string startLane = GetNodeLane(startNode);
                    string endLane = GetNodeLane(endNode);
                    string laneMatch = startLane == endLane ? "✓ SAME LANE" : "⚠ CROSS LANE";
                    
                    Debug.Log($"🚗 Spawned car: {startNode.name}({startLane}) → {endNode.name}({endLane}) {laneMatch} (Path: {path.Count} nodes)");
                }
            }
            else
            {
                Debug.LogError($"Car prefab missing RealisticCar component!");
                Destroy(car);
            }
        }
        else
        {
            string startLane = GetNodeLane(startNode);
            string endLane = GetNodeLane(endNode);
            Debug.LogWarning($"✗ No path found: {startNode.name}({startLane}) → {endNode.name}({endLane})");
            Destroy(car);
        }
    }
    
    /// <summary>
    /// Determine which lane a node belongs to
    /// </summary>
    string GetNodeLane(Node node)
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
        
        return "Center";
    }
    
    [ContextMenu("Debug Lane Distribution")]
    void DebugLaneDistribution()
    {
        RefreshNodeCache();
        
        Debug.Log("=== LANE DISTRIBUTION DEBUG ===");
        
        foreach (var kvp in startNodesByLane)
        {
            Debug.Log($"{kvp.Key} Lane Start Nodes ({kvp.Value.Count}):");
            foreach (Node node in kvp.Value)
            {
                Debug.Log($"  - {node.name} at {node.transform.position}");
            }
        }
        
        foreach (var kvp in endNodesByLane)
        {
            Debug.Log($"{kvp.Key} Lane End Nodes ({kvp.Value.Count}):");
            foreach (Node node in kvp.Value)
            {
                Debug.Log($"  - {node.name} at {node.transform.position}");
            }
        }
    }
    
    // Context menu methods for testing
    [ContextMenu("Spawn Single Random Car")]
    void SpawnSingleRandomCar()
    {
        RefreshNodeCache();
        SpawnRandomCar();
    }
    
    [ContextMenu("Clear All Cars")]
    void ClearAllCars()
    {
        RealisticCar[] allCars = FindObjectsOfType<RealisticCar>();
        foreach (RealisticCar car in allCars)
        {
            if (Application.isPlaying)
            {
                Destroy(car.gameObject);
            }
            else
            {
                DestroyImmediate(car.gameObject);
            }
        }
        Debug.Log($"Cleared {allCars.Length} cars");
    }
}