using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

public class FloorSorter : EditorWindow
{
    [MenuItem("Tools/City Organizer/Sort Floor Elements")]
    public static void ShowWindow()
    {
        GetWindow<FloorSorter>("Floor Sorter");
    }

    private void OnGUI()
    {
        GUILayout.Label("Floor Elements Organizer", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        GUILayout.Label("This tool will sort floor prefabs into subcategories:", EditorStyles.wordWrappedLabel);
        GUILayout.Label("• Floor → Walkway", EditorStyles.miniLabel);
        GUILayout.Label("• Floor → Streets", EditorStyles.miniLabel);
        GUILayout.Label("• Floor → Sideway", EditorStyles.miniLabel);
        GUILayout.Label("• Floor → Stoneway", EditorStyles.miniLabel);
        GUILayout.Label("• Floor → Sidewalk", EditorStyles.miniLabel);
        GUILayout.Space(10);
        
        if (GUILayout.Button("Sort All Floor Prefabs", GUILayout.Height(30)))
        {
            SortFloorPrefabs();
        }
        
        GUILayout.Space(10);
        
        if (GUILayout.Button("Preview Floor Objects", GUILayout.Height(25)))
        {
            PreviewFloorObjects();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Clear Floor Organization", GUILayout.Height(25)))
        {
            ClearFloorOrganization();
        }
    }
    
    private void SortFloorPrefabs()
    {
        // Create main Floor parent
        GameObject floorParent = CreateOrFindParent("Floor");
        
        // Create subcategory parents
        GameObject walkwayParent = CreateOrFindParent("Walkway", floorParent);
        GameObject streetsParent = CreateOrFindParent("Streets", floorParent);
        GameObject sidewayParent = CreateOrFindParent("Sideway", floorParent);
        GameObject stonewayParent = CreateOrFindParent("Stoneway", floorParent);
        GameObject sidewalkParent = CreateOrFindParent("Sidewalk", floorParent);
        GameObject sidewalkParent = CreateOrFindParent("Sidewalk", floorParent);
        
        // Get all GameObjects in the scene
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        
        int walkwayCount = 0, streetCount = 0, sidewayCount = 0, stonewayCount = 0, sidewalkCount = 0;
        
        foreach (GameObject obj in allObjects)
        {
            FloorCategory category = GetFloorCategory(obj.name);
            GameObject targetParent = null;
            
            switch (category)
            {
                case FloorCategory.Walkway:
                    targetParent = walkwayParent;
                    walkwayCount++;
                    break;
                case FloorCategory.Street:
                    targetParent = streetsParent;
                    streetCount++;
                    break;
                case FloorCategory.Sideway:
                    targetParent = sidewayParent;
                    sidewayCount++;
                    break;
                case FloorCategory.Stoneway:
                    targetParent = stonewayParent;
                    stonewayCount++;
                    break;
                case FloorCategory.Sidewalk:
                    targetParent = sidewalkParent;
                    sidewalkCount++;
                    break;
                case FloorCategory.None:
                    continue; // Skip non-floor objects
            }
            
            // Move object under appropriate parent if it's not already there
            if (targetParent != null && obj.transform.parent != targetParent.transform)
            {
                Undo.SetTransformParent(obj.transform, targetParent.transform, "Sort Floor Prefabs");
            }
        }
        
        Debug.Log("=== FLOOR SORTING COMPLETE ===");
        Debug.Log($"Walkway objects: {walkwayCount}");
        Debug.Log($"Street objects: {streetCount}");
        Debug.Log($"Sideway objects: {sidewayCount}");
        Debug.Log($"Stoneway objects: {stonewayCount}");
        Debug.Log($"Sidewalk objects: {sidewalkCount}");
        Debug.Log($"Total floor objects sorted: {walkwayCount + streetCount + sidewayCount + stonewayCount + sidewalkCount}");
        
        // Mark scene as dirty
        EditorUtility.SetDirty(floorParent);
    }
    
    private void PreviewFloorObjects()
    {
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        
        List<string> walkwayObjects = new List<string>();
        List<string> streetObjects = new List<string>();
        List<string> sidewayObjects = new List<string>();
        List<string> stonewayObjects = new List<string>();
        List<string> sidewalkObjects = new List<string>();
        
        foreach (GameObject obj in allObjects)
        {
            FloorCategory category = GetFloorCategory(obj.name);
            
            switch (category)
            {
                case FloorCategory.Walkway:
                    walkwayObjects.Add(obj.name);
                    break;
                case FloorCategory.Street:
                    streetObjects.Add(obj.name);
                    break;
                case FloorCategory.Sideway:
                    sidewayObjects.Add(obj.name);
                    break;
                case FloorCategory.Stoneway:
                    stonewayObjects.Add(obj.name);
                    break;
                case FloorCategory.Sidewalk:
                    sidewalkObjects.Add(obj.name);
                    break;
            }
        }
        
        Debug.Log("=== FLOOR OBJECTS PREVIEW ===");
        
        Debug.Log("--- WALKWAY OBJECTS ---");
        foreach (string name in walkwayObjects)
            Debug.Log($"Walkway: {name}");
        
        Debug.Log("--- STREET OBJECTS ---");
        foreach (string name in streetObjects)
            Debug.Log($"Street: {name}");
        
        Debug.Log("--- SIDEWAY OBJECTS ---");
        foreach (string name in sidewayObjects)
            Debug.Log($"Sideway: {name}");
        
        Debug.Log("--- STONEWAY OBJECTS ---");
        foreach (string name in stonewayObjects)
            Debug.Log($"Stoneway: {name}");
        
        Debug.Log("--- SIDEWALK OBJECTS ---");
        foreach (string name in sidewalkObjects)
            Debug.Log($"Sidewalk: {name}");
        
        Debug.Log($"Total: {walkwayObjects.Count + streetObjects.Count + sidewayObjects.Count + stonewayObjects.Count + sidewalkObjects.Count} floor objects found");
    }
    
    private void ClearFloorOrganization()
    {
        GameObject floorParent = GameObject.Find("Floor");
        if (floorParent != null)
        {
            if (EditorUtility.DisplayDialog("Clear Floor Organization", 
                "This will move all floor objects back to the root and delete the Floor organization structure. Are you sure?", 
                "Yes, Clear", "Cancel"))
            {
                // Move all children back to root
                Transform[] children = floorParent.GetComponentsInChildren<Transform>();
                foreach (Transform child in children)
                {
                    if (child != floorParent.transform && child.parent != null)
                    {
                        // Only move objects that are actual prefabs, not the organization folders
                        if (GetFloorCategory(child.name) != FloorCategory.None)
                        {
                            Undo.SetTransformParent(child, null, "Clear Floor Organization");
                        }
                    }
                }
                
                // Delete the Floor parent object
                Undo.DestroyObjectImmediate(floorParent);
                Debug.Log("Floor organization cleared successfully.");
            }
        }
        else
        {
            Debug.Log("No Floor organization found to clear.");
        }
    }
    
    private GameObject CreateOrFindParent(string parentName, GameObject parent = null)
    {
        GameObject existingParent = null;
        
        if (parent == null)
        {
            existingParent = GameObject.Find(parentName);
        }
        else
        {
            Transform childTransform = parent.transform.Find(parentName);
            if (childTransform != null)
                existingParent = childTransform.gameObject;
        }
        
        if (existingParent == null)
        {
            existingParent = new GameObject(parentName);
            if (parent != null)
            {
                existingParent.transform.SetParent(parent.transform);
            }
            Undo.RegisterCreatedObjectUndo(existingParent, $"Create {parentName} Parent");
        }
        
        return existingParent;
    }
    
    private FloorCategory GetFloorCategory(string objectName)
    {
        string name = objectName.ToLower();
        
        // Check for Walkway (Walkway X Prefab)
        if (name.StartsWith("walkway"))
        {
            return FloorCategory.Walkway;
        }
        
        // Check for Street (Street X Prefab, Street X YM prefab, etc.)
        if (name.StartsWith("street"))
        {
            return FloorCategory.Street;
        }
        
        // Check for Sideway (Sideway X prefab)
        if (name.StartsWith("sideway"))
        {
            return FloorCategory.Sideway;
        }
        
        // Check for Stoneway (stoneway X)
        if (name.StartsWith("stoneway"))
        {
            return FloorCategory.Stoneway;
        }
        
        // Check for Sidewalk (Sidewalk_XX (Y))
        if (name.StartsWith("sidewalk"))
        {
            return FloorCategory.Sidewalk;
        }
        
        return FloorCategory.None;
    }
    
    private enum FloorCategory
    {
        None,
        Walkway,
        Street,
        Sideway,
        Stoneway,
        Sidewalk
    }
}

// Quick menu option for instant sorting
public class AutoFloorSorter
{
    [MenuItem("Tools/City Organizer/Auto Sort All Floor Elements")]
    public static void AutoSortFloorElements()
    {
        // Create hierarchical structure
        GameObject floorParent = CreateOrFindParent("Floor");
        GameObject walkwayParent = CreateOrFindParent("Walkway", floorParent);
        GameObject streetsParent = CreateOrFindParent("Streets", floorParent);
        GameObject sidewayParent = CreateOrFindParent("Sideway", floorParent);
        GameObject stonewayParent = CreateOrFindParent("Stoneway", floorParent);
        
        GameObject[] allObjects = Object.FindObjectsOfType<GameObject>();
        int sortedCount = 0;
        
        foreach (GameObject obj in allObjects)
        {
            string name = obj.name.ToLower();
            GameObject targetParent = null;
            
            if (name.StartsWith("walkway"))
                targetParent = walkwayParent;
            else if (name.StartsWith("street"))
                targetParent = streetsParent;
            else if (name.StartsWith("sideway"))
                targetParent = sidewayParent;
            else if (name.StartsWith("stoneway"))
                targetParent = stonewayParent;
            else if (name.StartsWith("sidewalk"))
                targetParent = sidewalkParent;
            
            if (targetParent != null && obj.transform.parent != targetParent.transform)
            {
                Undo.SetTransformParent(obj.transform, targetParent.transform, "Auto Sort Floor Elements");
                sortedCount++;
            }
        }
        
        Debug.Log($"Auto Floor Sorter: Organized {sortedCount} floor elements into subcategories");
        EditorUtility.SetDirty(floorParent);
    }
    
    private static GameObject CreateOrFindParent(string parentName, GameObject parent = null)
    {
        GameObject existingParent = null;
        
        if (parent == null)
        {
            existingParent = GameObject.Find(parentName);
        }
        else
        {
            Transform childTransform = parent.transform.Find(parentName);
            if (childTransform != null)
                existingParent = childTransform.gameObject;
        }
        
        if (existingParent == null)
        {
            existingParent = new GameObject(parentName);
            if (parent != null)
            {
                existingParent.transform.SetParent(parent.transform);
            }
            Undo.RegisterCreatedObjectUndo(existingParent, $"Create {parentName} Parent");
        }
        
        return existingParent;
    }
}