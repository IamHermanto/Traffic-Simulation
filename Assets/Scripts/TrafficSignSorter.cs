using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

public class TrafficSignSorter : EditorWindow
{
    [MenuItem("Tools/City Organizer/Sort Traffic Signs")]
    public static void ShowWindow()
    {
        GetWindow<TrafficSignSorter>("Traffic Sign Sorter");
    }

    private void OnGUI()
    {
        GUILayout.Label("Traffic Sign Organizer", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        GUILayout.Label("This tool will automatically sort traffic sign prefabs in your scene.", EditorStyles.wordWrappedLabel);
        GUILayout.Label("Detects: parking sign, road traffic sign, slow sign, stop sign,", EditorStyles.miniLabel);
        GUILayout.Label("traffic sign X, traffic sign speed X", EditorStyles.miniLabel);
        GUILayout.Space(10);
        
        if (GUILayout.Button("Sort Traffic Sign Prefabs", GUILayout.Height(30)))
        {
            SortTrafficSignPrefabs();
        }
        
        GUILayout.Space(10);
        
        if (GUILayout.Button("Preview Traffic Sign Objects", GUILayout.Height(25)))
        {
            PreviewTrafficSignObjects();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Clear Traffic Sign Organization", GUILayout.Height(25)))
        {
            ClearTrafficSignOrganization();
        }
    }
    
    private void SortTrafficSignPrefabs()
    {
        // Find or create the Traffic Signs parent object
        GameObject trafficSignsParent = GameObject.Find("Traffic Signs");
        if (trafficSignsParent == null)
        {
            trafficSignsParent = new GameObject("Traffic Signs");
            Undo.RegisterCreatedObjectUndo(trafficSignsParent, "Create Traffic Signs Parent");
        }
        
        // Get all GameObjects in the scene
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        List<GameObject> trafficSignObjects = new List<GameObject>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsTrafficSignPrefab(obj.name))
            {
                trafficSignObjects.Add(obj);
            }
        }
        
        // Move traffic sign objects under the Traffic Signs parent
        int movedCount = 0;
        foreach (GameObject trafficSign in trafficSignObjects)
        {
            // Only move if it's not already a child of Traffic Signs parent
            if (trafficSign.transform.parent != trafficSignsParent.transform)
            {
                Undo.SetTransformParent(trafficSign.transform, trafficSignsParent.transform, "Sort Traffic Sign Prefabs");
                movedCount++;
            }
        }
        
        Debug.Log($"Traffic Sign Sorter: Moved {movedCount} traffic sign prefabs under 'Traffic Signs' parent. Total traffic sign objects: {trafficSignObjects.Count}");
        
        // Mark scene as dirty to save changes
        EditorUtility.SetDirty(trafficSignsParent);
    }
    
    private void PreviewTrafficSignObjects()
    {
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        List<string> basicSigns = new List<string>();
        List<string> numberedSigns = new List<string>();
        List<string> speedSigns = new List<string>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsTrafficSignPrefab(obj.name))
            {
                string name = obj.name.ToLower();
                
                if (name.Contains("speed"))
                {
                    speedSigns.Add(obj.name);
                }
                else if (name.StartsWith("traffic sign") && !name.Contains("speed"))
                {
                    numberedSigns.Add(obj.name);
                }
                else
                {
                    basicSigns.Add(obj.name);
                }
            }
        }
        
        Debug.Log("=== TRAFFIC SIGN PREFABS FOUND ===");
        
        Debug.Log("--- BASIC SIGNS ---");
        foreach (string name in basicSigns)
            Debug.Log($"Basic Sign: {name}");
        
        Debug.Log("--- NUMBERED TRAFFIC SIGNS ---");
        foreach (string name in numberedSigns)
            Debug.Log($"Numbered Sign: {name}");
        
        Debug.Log("--- SPEED LIMIT SIGNS ---");
        foreach (string name in speedSigns)
            Debug.Log($"Speed Sign: {name}");
        
        Debug.Log($"Total traffic sign prefabs found: {basicSigns.Count + numberedSigns.Count + speedSigns.Count}");
        Debug.Log($"(Basic: {basicSigns.Count}, Numbered: {numberedSigns.Count}, Speed: {speedSigns.Count})");
    }
    
    private void ClearTrafficSignOrganization()
    {
        GameObject trafficSignsParent = GameObject.Find("Traffic Signs");
        if (trafficSignsParent != null)
        {
            if (EditorUtility.DisplayDialog("Clear Traffic Sign Organization", 
                "This will move all traffic sign objects back to the root and delete the Traffic Signs organization structure. Are you sure?", 
                "Yes, Clear", "Cancel"))
            {
                // Move all children back to root
                Transform[] children = trafficSignsParent.GetComponentsInChildren<Transform>();
                foreach (Transform child in children)
                {
                    if (child != trafficSignsParent.transform && IsTrafficSignPrefab(child.name))
                    {
                        Undo.SetTransformParent(child, null, "Clear Traffic Sign Organization");
                    }
                }
                
                // Delete the Traffic Signs parent object
                Undo.DestroyObjectImmediate(trafficSignsParent);
                Debug.Log("Traffic sign organization cleared successfully.");
            }
        }
        else
        {
            Debug.Log("No Traffic Sign organization found to clear.");
        }
    }
    
    private bool IsTrafficSignPrefab(string objectName)
    {
        // Convert to lowercase for case-insensitive comparison
        string name = objectName.ToLower();
        
        // Basic signs
        string[] basicSigns = {
            "parking sign",
            "road traffic sign", 
            "slow sign",
            "stop sign"
        };
        
        foreach (string basicSign in basicSigns)
        {
            if (name == basicSign)
            {
                return true;
            }
        }
        
        // Pattern: "traffic sign X" (where X is a number)
        if (name.StartsWith("traffic sign ") && !name.Contains("speed"))
        {
            return true;
        }
        
        // Pattern: "traffic sign speed X" (where X is a speed number)
        if (name.StartsWith("traffic sign speed"))
        {
            return true;
        }
        
        // Additional catch-all for any sign containing "sign"
        if (name.Contains("sign") && (name.Contains("traffic") || name.Contains("parking") || name.Contains("stop") || name.Contains("slow")))
        {
            return true;
        }
        
        return false;
    }
}

// Alternative: Automatic sorting script that can be run from the menu
public class AutoTrafficSignSorter
{
    [MenuItem("Tools/City Organizer/Auto Sort All Traffic Signs")]
    public static void AutoSortTrafficSigns()
    {
        // Create parent object
        GameObject trafficSignsParent = CreateOrFindParent("Traffic Signs");
        
        // Get all objects and sort them
        GameObject[] allObjects = Object.FindObjectsOfType<GameObject>();
        int sortedCount = 0;
        
        foreach (GameObject obj in allObjects)
        {
            if (IsTrafficSignObject(obj.name) && obj.transform.parent == null)
            {
                Undo.SetTransformParent(obj.transform, trafficSignsParent.transform, "Auto Sort Traffic Signs");
                sortedCount++;
            }
        }
        
        Debug.Log($"Auto Traffic Sign Sorter: Organized {sortedCount} traffic sign objects");
        EditorUtility.SetDirty(trafficSignsParent);
    }
    
    private static GameObject CreateOrFindParent(string parentName)
    {
        GameObject parent = GameObject.Find(parentName);
        if (parent == null)
        {
            parent = new GameObject(parentName);
            Undo.RegisterCreatedObjectUndo(parent, $"Create {parentName} Parent");
        }
        return parent;
    }
    
    private static bool IsTrafficSignObject(string objectName)
    {
        string name = objectName.ToLower();
        
        // Check for specific basic signs
        if (name == "parking sign" || name == "road traffic sign" || 
            name == "slow sign" || name == "stop sign")
        {
            return true;
        }
        
        // Check for traffic sign patterns
        if (name.StartsWith("traffic sign"))
        {
            return true;
        }
        
        // Additional catch-all
        if (name.Contains("sign") && (name.Contains("traffic") || name.Contains("parking") || 
            name.Contains("stop") || name.Contains("slow")))
        {
            return true;
        }
        
        return false;
    }
}