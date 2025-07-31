using UnityEngine;
using System.Collections.Generic;

public class SafeStreetDetector : MonoBehaviour
{
    [Header("Detection Settings")]
    public string[] streetNamePatterns = {"Street", "Walkway", "Sideway", "Stoneway"};
    
    [Header("Found Streets")]
    public List<GameObject> foundStreets = new List<GameObject>();
    
    [Header("Debug")]
    public bool showLogs = true;
    
    [ContextMenu("Step 1: Find Streets Only")]
    public void FindStreetsOnly()
    {
        foundStreets.Clear();
        
        try
        {
            if (showLogs) Debug.Log("=== SAFE STREET DETECTION ===");
            
            GameObject[] allObjects = FindObjectsOfType<GameObject>();
            if (showLogs) Debug.Log($"Scanning {allObjects.Length} total objects...");
            
            int checkedCount = 0;
            foreach (GameObject obj in allObjects)
            {
                checkedCount++;
                
                if (obj == null) 
                {
                    if (showLogs) Debug.LogWarning($"Null object found at index {checkedCount}");
                    continue;
                }
                
                if (IsStreetObject(obj))
                {
                    foundStreets.Add(obj);
                    if (showLogs) Debug.Log($"✅ Found street: {obj.name}");
                }
                
                // Safety check - don't process too many at once
                if (checkedCount % 100 == 0)
                {
                    if (showLogs) Debug.Log($"Processed {checkedCount} objects...");
                }
            }
            
            if (showLogs) Debug.Log($"=== DETECTION COMPLETE ===");
            if (showLogs) Debug.Log($"Found {foundStreets.Count} street objects");
            
            // List all found streets
            foreach (GameObject street in foundStreets)
            {
                if (showLogs) Debug.Log($"Street: {street.name} at position {street.transform.position}");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error during street detection: {e.Message}");
            Debug.LogError($"Stack trace: {e.StackTrace}");
        }
    }
    
    [ContextMenu("Step 2: Check Street Components")]
    public void CheckStreetComponents()
    {
        if (foundStreets.Count == 0)
        {
            Debug.LogWarning("No streets found! Run 'Find Streets Only' first.");
            return;
        }
        
        try
        {
            if (showLogs) Debug.Log("=== CHECKING STREET COMPONENTS ===");
            
            foreach (GameObject street in foundStreets)
            {
                if (street == null)
                {
                    Debug.LogWarning("Found null street object!");
                    continue;
                }
                
                // Check MeshCollider
                MeshCollider meshCollider = street.GetComponent<MeshCollider>();
                if (meshCollider == null)
                {
                    Debug.LogWarning($"❌ {street.name}: Missing MeshCollider");
                }
                else
                {
                    if (showLogs) Debug.Log($"✅ {street.name}: Has MeshCollider, bounds: {meshCollider.bounds.size}");
                }
                
                // Check MeshRenderer
                MeshRenderer meshRenderer = street.GetComponent<MeshRenderer>();
                if (meshRenderer == null)
                {
                    Debug.LogWarning($"❌ {street.name}: Missing MeshRenderer");
                }
                else
                {
                    if (showLogs) Debug.Log($"✅ {street.name}: Has MeshRenderer");
                }
                
                // Check Transform
                if (showLogs) Debug.Log($"📍 {street.name}: Position {street.transform.position}, Rotation {street.transform.eulerAngles}");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error during component check: {e.Message}");
        }
    }
    
    [ContextMenu("Step 3: Test Simple Connection")]
    public void TestSimpleConnection()
    {
        if (foundStreets.Count < 2)
        {
            Debug.LogWarning("Need at least 2 streets to test connections!");
            return;
        }
        
        try
        {
            if (showLogs) Debug.Log("=== TESTING SIMPLE CONNECTIONS ===");
            
            for (int i = 0; i < Mathf.Min(5, foundStreets.Count); i++)
            {
                for (int j = i + 1; j < Mathf.Min(5, foundStreets.Count); j++)
                {
                    GameObject streetA = foundStreets[i];
                    GameObject streetB = foundStreets[j];
                    
                    if (streetA == null || streetB == null) continue;
                    
                    float distance = Vector3.Distance(streetA.transform.position, streetB.transform.position);
                    if (showLogs) Debug.Log($"Distance between {streetA.name} and {streetB.name}: {distance:F2}");
                    
                    if (distance < 50f) // Reasonable connection distance
                    {
                        if (showLogs) Debug.Log($"🔗 Potential connection: {streetA.name} ↔ {streetB.name}");
                    }
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error during connection test: {e.Message}");
        }
    }
    
    private bool IsStreetObject(string objectName)
    {
        if (string.IsNullOrEmpty(objectName)) return false;
        
        string name = objectName.ToLower();
        
        foreach (string pattern in streetNamePatterns)
        {
            if (name.StartsWith(pattern.ToLower()))
            {
                return true;
            }
        }
        
        return false;
    }
    
    private bool IsStreetObject(GameObject obj)
    {
        return IsStreetObject(obj.name);
    }
    
    [ContextMenu("Clear Results")]
    public void ClearResults()
    {
        foundStreets.Clear();
        Debug.Log("Cleared all results.");
    }
}