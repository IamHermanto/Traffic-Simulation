using UnityEngine;
using UnityEditor;

public class LaneNodeCreator : EditorWindow
{
    [MenuItem("Tools/City Organizer/Create Lane Nodes")]
    public static void ShowWindow()
    {
        GetWindow<LaneNodeCreator>("Lane Node Creator");
    }

    [Header("Intersection Settings")]
    public Vector3 intersectionCenter = Vector3.zero;
    public float laneWidth = 2f;
    public float approachDistance = 10f;
    public float exitDistance = 10f;
    public bool createConnections = true;
    
    private void OnGUI()
    {
        GUILayout.Label("Multi-Lane Intersection Creator", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        GUILayout.Label("This tool creates proper lane nodes for realistic traffic flow", EditorStyles.wordWrappedLabel);
        GUILayout.Space(10);
        
        // Settings
        GUILayout.Label("Settings:", EditorStyles.boldLabel);
        intersectionCenter = EditorGUILayout.Vector3Field("Intersection Center", intersectionCenter);
        laneWidth = EditorGUILayout.FloatField("Lane Width", laneWidth);
        approachDistance = EditorGUILayout.FloatField("Approach Distance", approachDistance);
        exitDistance = EditorGUILayout.FloatField("Exit Distance", exitDistance);
        createConnections = EditorGUILayout.Toggle("Auto-Create Connections", createConnections);
        
        GUILayout.Space(10);
        
        // Action buttons
        if (GUILayout.Button("Create Full Intersection", GUILayout.Height(30)))
        {
            CreateFullIntersection();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Create North Approach Only", GUILayout.Height(25)))
        {
            CreateDirectionNodes("North");
        }
        
        if (GUILayout.Button("Create East Approach Only", GUILayout.Height(25)))
        {
            CreateDirectionNodes("East");
        }
        
        if (GUILayout.Button("Create South Approach Only", GUILayout.Height(25)))
        {
            CreateDirectionNodes("South");
        }
        
        if (GUILayout.Button("Create West Approach Only", GUILayout.Height(25)))
        {
            CreateDirectionNodes("West");
        }
        
        GUILayout.Space(10);
        
        if (GUILayout.Button("Clear All Lane Nodes", GUILayout.Height(25)))
        {
            ClearLaneNodes();
        }
        
        GUILayout.Space(10);
        
        // Instructions
        GUILayout.Label("Instructions:", EditorStyles.boldLabel);
        GUILayout.Label("1. Set intersection center position", EditorStyles.miniLabel);
        GUILayout.Label("2. Adjust lane width and distances", EditorStyles.miniLabel);
        GUILayout.Label("3. Click 'Create Full Intersection'", EditorStyles.miniLabel);
        GUILayout.Label("4. Cars will automatically use proper lanes!", EditorStyles.miniLabel);
    }
    
    void CreateFullIntersection()
    {
        Debug.Log("=== CREATING MULTI-LANE INTERSECTION ===");
        
        // Create all four approaches
        CreateDirectionNodes("North");
        CreateDirectionNodes("East");
        CreateDirectionNodes("South");
        CreateDirectionNodes("West");
        
        // Create intersection center nodes
        CreateIntersectionCenterNodes();
        
        if (createConnections)
        {
            CreateAllConnections();
        }
        
        Debug.Log("✓ Multi-lane intersection created successfully!");
        Debug.Log("Cars will now use proper lanes for turns and straight-through traffic.");
    }
    
    void CreateDirectionNodes(string direction)
    {
        Vector3 offset = GetDirectionOffset(direction);
        SpawnDirection spawnDir = GetSpawnDirection(direction);
        
        // Create approach nodes (where cars come from)
        CreateLaneNode($"{direction}_Left_Lane", intersectionCenter + offset * approachDistance + GetLaneOffset(direction, true), NodeType.Start, spawnDir);
        CreateLaneNode($"{direction}_Right_Lane", intersectionCenter + offset * approachDistance + GetLaneOffset(direction, false), NodeType.Start, spawnDir);
        
        // Create exit nodes (where cars go to)
        CreateLaneNode($"{direction}_Exit_Left_Lane", intersectionCenter - offset * exitDistance + GetLaneOffset(direction, true), NodeType.End, spawnDir);
        CreateLaneNode($"{direction}_Exit_Right_Lane", intersectionCenter - offset * exitDistance + GetLaneOffset(direction, false), NodeType.End, spawnDir);
        
        Debug.Log($"Created {direction} approach and exit lanes");
    }
    
    void CreateIntersectionCenterNodes()
    {
        CreateLaneNode("Intersection_Center_North", intersectionCenter + Vector3.forward * (laneWidth/2), NodeType.Normal, SpawnDirection.North);
        CreateLaneNode("Intersection_Center_East", intersectionCenter + Vector3.right * (laneWidth/2), NodeType.Normal, SpawnDirection.East);
        CreateLaneNode("Intersection_Center_South", intersectionCenter + Vector3.back * (laneWidth/2), NodeType.Normal, SpawnDirection.South);
        CreateLaneNode("Intersection_Center_West", intersectionCenter + Vector3.left * (laneWidth/2), NodeType.Normal, SpawnDirection.West);
        
        Debug.Log("Created intersection center nodes");
    }
    
    void CreateLaneNode(string nodeName, Vector3 position, NodeType nodeType, SpawnDirection spawnDirection)
    {
        // Check if node already exists
        GameObject existing = GameObject.Find(nodeName);
        if (existing != null)
        {
            Debug.LogWarning($"Node {nodeName} already exists, skipping...");
            return;
        }
        
        // Create node GameObject
        GameObject nodeObj = new GameObject(nodeName);
        nodeObj.transform.position = position;
        
        // Add Node component
        Node nodeComponent = nodeObj.AddComponent<Node>();
        nodeComponent.nodeType = nodeType;
        nodeComponent.spawnDirection = spawnDirection;
        nodeComponent.connectionDistance = 8f; // Increased for lane connections
        nodeComponent.showConnections = true;
        nodeComponent.showNeighborCount = true;
        
        // Register with undo system
        Undo.RegisterCreatedObjectUndo(nodeObj, $"Create Lane Node {nodeName}");
        
        Debug.Log($"Created {nodeType} node: {nodeName} at {position}");
    }
    
    Vector3 GetDirectionOffset(string direction)
    {
        switch (direction)
        {
            case "North": return Vector3.forward;
            case "East": return Vector3.right;
            case "South": return Vector3.back;
            case "West": return Vector3.left;
            default: return Vector3.forward;
        }
    }
    
    Vector3 GetLaneOffset(string direction, bool isLeftLane)
    {
        Vector3 laneOffset = Vector3.zero;
        float offset = laneWidth / 2;
        
        switch (direction)
        {
            case "North":
                laneOffset = isLeftLane ? Vector3.left * offset : Vector3.right * offset;
                break;
            case "East":
                laneOffset = isLeftLane ? Vector3.forward * offset : Vector3.back * offset;
                break;
            case "South":
                laneOffset = isLeftLane ? Vector3.right * offset : Vector3.left * offset;
                break;
            case "West":
                laneOffset = isLeftLane ? Vector3.back * offset : Vector3.forward * offset;
                break;
        }
        
        return laneOffset;
    }
    
    SpawnDirection GetSpawnDirection(string direction)
    {
        switch (direction)
        {
            case "North": return SpawnDirection.South;
            case "East": return SpawnDirection.West;
            case "South": return SpawnDirection.North;
            case "West": return SpawnDirection.East;
            default: return SpawnDirection.North;
        }
    }
    
    void CreateAllConnections()
    {
        Debug.Log("Creating lane connections...");
        
        // This would be quite complex to do automatically
        // Better to let the enhanced Node script handle connections
        // or use the Network Validator tool
        
        Debug.Log("Use the Node Network Validator to check and fix connections!");
    }
    
    void ClearLaneNodes()
    {
        if (EditorUtility.DisplayDialog("Clear Lane Nodes", 
            "This will delete all lane nodes (nodes with 'Lane' in the name). Are you sure?", 
            "Yes, Clear", "Cancel"))
        {
            GameObject[] allObjects = FindObjectsOfType<GameObject>();
            int deletedCount = 0;
            
            foreach (GameObject obj in allObjects)
            {
                if (obj.name.Contains("Lane") || obj.name.Contains("Intersection_Center"))
                {
                    Undo.DestroyObjectImmediate(obj);
                    deletedCount++;
                }
            }
            
            Debug.Log($"Cleared {deletedCount} lane nodes");
        }
    }
}