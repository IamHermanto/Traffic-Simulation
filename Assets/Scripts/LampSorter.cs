using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

public class LampSorter : EditorWindow
{
    [MenuItem("Tools/City Organizer/Sort Lamps")]
    public static void ShowWindow()
    {
        GetWindow<LampSorter>("Lamp Sorter");
    }

    private void OnGUI()
    {
        GUILayout.Label("Lamp Organizer", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        GUILayout.Label("This tool will automatically sort lamp prefabs in your scene.", EditorStyles.wordWrappedLabel);
        GUILayout.Label("Detects: Lamp_X_prefab and street lamp X prefab", EditorStyles.miniLabel);
        GUILayout.Space(10);
        
        if (GUILayout.Button("Sort Lamp Prefabs", GUILayout.Height(30)))
        {
            SortLampPrefabs();
        }
        
        GUILayout.Space(10);
        
        if (GUILayout.Button("Preview Lamp Objects", GUILayout.Height(25)))
        {
            PreviewLampObjects();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Clear Lamp Organization", GUILayout.Height(25)))
        {
            ClearLampOrganization();
        }
    }
    
    private void SortLampPrefabs()
    {
        // Find or create the Lamps parent object
        GameObject lampsParent = GameObject.Find("Lamps");
        if (lampsParent == null)
        {
            lampsParent = new GameObject("Lamps");
            Undo.RegisterCreatedObjectUndo(lampsParent, "Create Lamps Parent");
        }
        
        // Get all GameObjects in the scene
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        List<GameObject> lampObjects = new List<GameObject>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsLampPrefab(obj.name))
            {
                lampObjects.Add(obj);
            }
        }
        
        // Move lamp objects under the Lamps parent
        int movedCount = 0;
        foreach (GameObject lamp in lampObjects)
        {
            // Only move if it's not already a child of Lamps parent
            if (lamp.transform.parent != lampsParent.transform)
            {
                Undo.SetTransformParent(lamp.transform, lampsParent.transform, "Sort Lamp Prefabs");
                movedCount++;
            }
        }
        
        Debug.Log($"Lamp Sorter: Moved {movedCount} lamp prefabs under 'Lamps' parent. Total lamp objects: {lampObjects.Count}");
        
        // Mark scene as dirty to save changes
        EditorUtility.SetDirty(lampsParent);
    }
    
    private void PreviewLampObjects()
    {
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        List<string> regularLamps = new List<string>();
        List<string> streetLamps = new List<string>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsLampPrefab(obj.name))
            {
                string name = obj.name.ToLower();
                if (name.StartsWith("street lamp"))
                {
                    streetLamps.Add(obj.name);
                }
                else
                {
                    regularLamps.Add(obj.name);
                }
            }
        }
        
        Debug.Log("=== LAMP PREFABS FOUND ===");
        
        Debug.Log("--- REGULAR LAMPS ---");
        foreach (string name in regularLamps)
            Debug.Log($"Regular Lamp: {name}");
        
        Debug.Log("--- STREET LAMPS ---");
        foreach (string name in streetLamps)
            Debug.Log($"Street Lamp: {name}");
        
        Debug.Log($"Total lamp prefabs found: {regularLamps.Count + streetLamps.Count} (Regular: {regularLamps.Count}, Street: {streetLamps.Count})");
    }
    
    private void ClearLampOrganization()
    {
        GameObject lampsParent = GameObject.Find("Lamps");
        if (lampsParent != null)
        {
            if (EditorUtility.DisplayDialog("Clear Lamp Organization", 
                "This will move all lamp objects back to the root and delete the Lamps organization structure. Are you sure?", 
                "Yes, Clear", "Cancel"))
            {
                // Move all children back to root
                Transform[] children = lampsParent.GetComponentsInChildren<Transform>();
                foreach (Transform child in children)
                {
                    if (child != lampsParent.transform && IsLampPrefab(child.name))
                    {
                        Undo.SetTransformParent(child, null, "Clear Lamp Organization");
                    }
                }
                
                // Delete the Lamps parent object
                Undo.DestroyObjectImmediate(lampsParent);
                Debug.Log("Lamp organization cleared successfully.");
            }
        }
        else
        {
            Debug.Log("No Lamp organization found to clear.");
        }
    }
    
    private bool IsLampPrefab(string objectName)
    {
        // Convert to lowercase for case-insensitive comparison
        string name = objectName.ToLower();
        
        // Pattern 1: "Lamp_X_prefab" (e.g., Lamp_1_prefab, Lamp_2_prefab)
        if (name.StartsWith("lamp_") && name.EndsWith("_prefab"))
        {
            return true;
        }
        
        // Pattern 2: "street lamp X prefab" (e.g., street lamp 1 prefab, street lamp 2 prefab)
        if (name.StartsWith("street lamp") && name.EndsWith("prefab"))
        {
            return true;
        }
        
        // Additional pattern: just "lamp" followed by number (in case there are variations)
        if (name.StartsWith("lamp") && name.Contains("prefab"))
        {
            return true;
        }
        
        return false;
    }
}

// Alternative: Automatic sorting script that can be run from the menu
public class AutoLampSorter
{
    [MenuItem("Tools/City Organizer/Auto Sort All Lamps")]
    public static void AutoSortLamps()
    {
        // Create parent object
        GameObject lampsParent = CreateOrFindParent("Lamps");
        
        // Get all objects and sort them
        GameObject[] allObjects = Object.FindObjectsOfType<GameObject>();
        int sortedCount = 0;
        
        foreach (GameObject obj in allObjects)
        {
            if (IsLampObject(obj.name) && obj.transform.parent == null)
            {
                Undo.SetTransformParent(obj.transform, lampsParent.transform, "Auto Sort Lamps");
                sortedCount++;
            }
        }
        
        Debug.Log($"Auto Lamp Sorter: Organized {sortedCount} lamp objects");
        EditorUtility.SetDirty(lampsParent);
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
    
    private static bool IsLampObject(string objectName)
    {
        string name = objectName.ToLower();
        
        return (name.StartsWith("lamp_") && name.EndsWith("_prefab")) ||
               (name.StartsWith("street lamp") && name.EndsWith("prefab")) ||
               (name.StartsWith("lamp") && name.Contains("prefab"));
    }
}