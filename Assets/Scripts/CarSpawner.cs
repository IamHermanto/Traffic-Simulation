using System.Collections.Generic;
using UnityEngine;

public class CarSpawner : MonoBehaviour
{
    public GameObject carPrefab;
    
    [Header("Spawn Settings")]
    public float spawnHeight = 0.5f;
    public bool spawnOnStart = true;
    public float spawnInterval = 5f;
    public bool continuousSpawning = true;
    public int maxCarsInScene = 10; // Limit cars for performance
    
    [Header("Simulation Settings")]
    public bool randomizeDestinations = true;
    public bool debugSpawning = true;
    
    // Cache for performance
    private List<Node> cachedStartNodes = new List<Node>();
    private List<Node> cachedEndNodes = new List<Node>();
    private float cacheRefreshInterval = 10f; // Refresh cache every 10 seconds
    private float lastCacheRefresh = 0f;
    
    void Start()
    {
        Debug.Log("Smart CarSpawner Start() - Setting up intersection simulation");
        
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
        // Refresh node cache periodically
        if (Time.time - lastCacheRefresh > cacheRefreshInterval)
        {
            RefreshNodeCache();
        }
    }
    
    void RefreshNodeCache()
    {
        cachedStartNodes.Clear();
        cachedEndNodes.Clear();
        
        Node[] allNodes = FindObjectsOfType<Node>();
        foreach (Node node in allNodes)
        {
            if (node.nodeType == NodeType.Start) 
                cachedStartNodes.Add(node);
            if (node.nodeType == NodeType.End) 
                cachedEndNodes.Add(node);
        }
        
        lastCacheRefresh = Time.time;
        
        if (debugSpawning)
        {
            Debug.Log($"Node cache refreshed: {cachedStartNodes.Count} start nodes, {cachedEndNodes.Count} end nodes");
        }
    }
    
    void SpawnRandomCar()
    {
        // Check if we have too many cars
        RealisticCar[] existingCars = FindObjectsOfType<RealisticCar>();
        if (existingCars.Length >= maxCarsInScene)
        {
            if (debugSpawning)
                Debug.Log($"Max cars ({maxCarsInScene}) reached, skipping spawn");
            return;
        }
        
        if (cachedStartNodes.Count == 0 || cachedEndNodes.Count == 0)
        {
            Debug.LogWarning("No start or end nodes available for spawning!");
            return;
        }
        
        // Pick random start and end nodes
        Node randomStartNode = cachedStartNodes[Random.Range(0, cachedStartNodes.Count)];
        Node randomEndNode = cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
        
        // Don't spawn if start and end are the same (in case a node is both start and end)
        if (randomStartNode == randomEndNode && cachedEndNodes.Count > 1)
        {
            // Pick a different end node
            do {
                randomEndNode = cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
            } while (randomEndNode == randomStartNode);
        }
        
        SpawnCarAtNode(randomStartNode, randomEndNode);
    }
    
    void SpawnCarsOnAllStartNodes()
    {
        if (cachedEndNodes.Count == 0)
        {
            Debug.LogWarning("No end nodes found! Cannot spawn cars.");
            return;
        }
        
        int spawnedCount = 0;
        foreach (Node startNode in cachedStartNodes)
        {
            Node targetEndNode;
            
            if (randomizeDestinations)
            {
                // Random destination
                targetEndNode = cachedEndNodes[Random.Range(0, cachedEndNodes.Count)];
            }
            else
            {
                // Use first end node for all cars (old behavior)
                targetEndNode = cachedEndNodes[0];
            }
            
            SpawnCarAtNode(startNode, targetEndNode);
            spawnedCount++;
        }
        
        if (debugSpawning)
        {
            Debug.Log($"Spawned {spawnedCount} cars on {cachedStartNodes.Count} start nodes");
        }
    }
    
    void SpawnCarAtNode(Node startNode, Node endNode)
    {
        if (startNode == null || endNode == null) 
        {
            Debug.LogWarning("SpawnCarAtNode: Invalid start or end node");
            return;
        }
        
        // Get spawn direction
        Vector3 spawnDirection = Vector3.forward;
        if (startNode.nodeType == NodeType.Start)
        {
            spawnDirection = startNode.GetSpawnDirection();
            if (debugSpawning)
                Debug.Log($"Spawn direction for {startNode.name}: {spawnDirection}");
        }
        
        // Calculate spawn position and rotation
        Vector3 spawnPosition = startNode.transform.position + Vector3.up * spawnHeight;
        Quaternion spawnRotation = spawnDirection.magnitude > 0.1f ? 
            Quaternion.LookRotation(spawnDirection) : Quaternion.identity;
        
        // Spawn car
        GameObject car = Instantiate(carPrefab, spawnPosition, spawnRotation);
        
        if (debugSpawning)
        {
            Debug.Log($"Car spawned at {spawnPosition} facing {spawnDirection}");
        }
        
        // Find path and set it
        List<Node> path = PathFinder.FindPath(startNode, endNode);
        
        if (path != null && path.Count > 0)
        {
            RealisticCar carComponent = car.GetComponent<RealisticCar>();
            if (carComponent != null)
            {
                carComponent.SetPathWithoutAlignment(path);
                
                if (debugSpawning)
                {
                    Debug.Log($"✓ Spawned car: {startNode.name} → {endNode.name} (Path: {path.Count} nodes)");
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
            Debug.LogWarning($"✗ No path found: {startNode.name} → {endNode.name}");
            Destroy(car);
        }
    }
    
    [ContextMenu("Spawn Single Random Car")]
    void SpawnSingleRandomCar()
    {
        RefreshNodeCache();
        SpawnRandomCar();
    }
    
    [ContextMenu("Spawn Cars on All Start Nodes")]
    void ManualSpawnAll()
    {
        RefreshNodeCache();
        SpawnCarsOnAllStartNodes();
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
    
    [ContextMenu("Debug Node Info")]
    void DebugNodeInfo()
    {
        RefreshNodeCache();
        Debug.Log("=== NODE DEBUG INFO ===");
        Debug.Log($"Start Nodes ({cachedStartNodes.Count}):");
        foreach (Node node in cachedStartNodes)
        {
            Debug.Log($"  - {node.name} at {node.transform.position} facing {node.GetSpawnDirection()}");
        }
        
        Debug.Log($"End Nodes ({cachedEndNodes.Count}):");
        foreach (Node node in cachedEndNodes)
        {
            Debug.Log($"  - {node.name} at {node.transform.position}");
        }
    }
    
    // Public method to get current car count
    public int GetCurrentCarCount()
    {
        return FindObjectsOfType<RealisticCar>().Length;
    }
    
    // Public method to adjust spawn rate based on traffic density
    public void AdjustSpawnRate(float newInterval)
    {
        spawnInterval = Mathf.Max(1f, newInterval); // Minimum 1 second
        
        if (continuousSpawning)
        {
            CancelInvoke(nameof(SpawnRandomCar));
            InvokeRepeating(nameof(SpawnRandomCar), spawnInterval, spawnInterval);
        }
        
        Debug.Log($"Spawn interval adjusted to {spawnInterval} seconds");
    }
}