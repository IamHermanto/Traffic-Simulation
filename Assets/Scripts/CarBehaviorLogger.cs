using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class CarBehaviorLogger : MonoBehaviour
{
    [Header("🔍 FILE LOGGING SETTINGS")]
    [Tooltip("Save logs to file (in addition to console)")]
    public bool saveToFile = true;
    
    [Tooltip("Log file name (saved in persistent data path)")]
    public string logFileName = "car_behavior_log.txt";
    
    [Tooltip("Maximum file size in MB before creating new file")]
    public int maxFileSizeMB = 10;
    
    [Tooltip("Clear log file on game start")]
    public bool clearLogOnStart = true;
    
    [Tooltip("Add timestamps to file logs")]
    public bool includeTimestamps = true;
    
    private static CarBehaviorLogger instance;
    private string logFilePath;
    private List<string> logBuffer = new List<string>();
    private float lastFlushTime = 0f;
    private float flushInterval = 2f; // Write to file every 2 seconds
    
    void Awake()
    {
        // Singleton pattern
        if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(gameObject);
            InitializeLogging();
        }
        else
        {
            Destroy(gameObject);
        }
    }
    
    void InitializeLogging()
    {
        if (!saveToFile) return;
        
        // Create log file path
        logFilePath = Path.Combine(Application.persistentDataPath, logFileName);
        
        Debug.Log($"📁 Car logs will be saved to: {logFilePath}");
        
        if (clearLogOnStart && File.Exists(logFilePath))
        {
            File.Delete(logFilePath);
        }
        
        // Write header
        string header = $"=== CAR BEHAVIOR LOG SESSION STARTED ===\n";
        header += $"Time: {System.DateTime.Now}\n";
        header += $"Scene: {UnityEngine.SceneManagement.SceneManager.GetActiveScene().name}\n";
        header += $"===========================================\n\n";
        
        File.AppendAllText(logFilePath, header);
    }
    
    void Update()
    {
        if (saveToFile && Time.time - lastFlushTime > flushInterval)
        {
            FlushLogBuffer();
            lastFlushTime = Time.time;
        }
    }
    
    /// <summary>
    /// Log a car behavior event (called by cars)
    /// </summary>
    public static void Log(string carID, string category, string details, Vector3 position)
    {
        if (instance == null) return;
        
        string timestamp = Time.time.ToString("F1");
        string positionStr = $"({position.x:F0}, {position.z:F0})";
        
        // Console log (always)
        string consoleLog = $"[{timestamp}s] {category} | {carID} @{positionStr} | {details}";
        Debug.Log(consoleLog);
        
        // File log (if enabled)
        if (instance.saveToFile)
        {
            string fileLog = consoleLog;
            
            if (instance.includeTimestamps)
            {
                string realTime = System.DateTime.Now.ToString("HH:mm:ss.fff");
                fileLog = $"[{realTime}] {consoleLog}";
            }
            
            instance.logBuffer.Add(fileLog);
        }
    }
    
    void FlushLogBuffer()
    {
        if (!saveToFile || logBuffer.Count == 0) return;
        
        try
        {
            // Check file size
            if (File.Exists(logFilePath))
            {
                FileInfo fileInfo = new FileInfo(logFilePath);
                if (fileInfo.Length > maxFileSizeMB * 1024 * 1024)
                {
                    // Create new log file
                    string backupPath = logFilePath.Replace(".txt", $"_backup_{System.DateTime.Now:yyyyMMdd_HHmmss}.txt");
                    File.Move(logFilePath, backupPath);
                    Debug.Log($"📁 Log file rotated. Backup saved to: {backupPath}");
                }
            }
            
            // Write buffered logs
            string logsToWrite = string.Join("\n", logBuffer) + "\n";
            File.AppendAllText(logFilePath, logsToWrite);
            
            logBuffer.Clear();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to write to log file: {e.Message}");
        }
    }
    
    void OnApplicationPause(bool pauseStatus)
    {
        if (!pauseStatus) FlushLogBuffer(); // Flush when unpausing
    }
    
    void OnApplicationFocus(bool hasFocus)
    {
        if (!hasFocus) FlushLogBuffer(); // Flush when losing focus
    }
    
    void OnApplicationQuit()
    {
        FlushLogBuffer();
        
        if (saveToFile)
        {
            string footer = $"\n=== SESSION ENDED at {System.DateTime.Now} ===\n\n";
            File.AppendAllText(logFilePath, footer);
        }
    }
    
    [ContextMenu("Open Log File Location")]
    void OpenLogFileLocation()
    {
        if (saveToFile)
        {
            string folderPath = Application.persistentDataPath;
            
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            System.Diagnostics.Process.Start("explorer.exe", folderPath.Replace('/', '\\'));
#elif UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
            System.Diagnostics.Process.Start("open", folderPath);
#else
            Debug.Log($"Log files are located at: {folderPath}");
#endif
        }
    }
    
    [ContextMenu("Clear Log File")]
    void ClearLogFile()
    {
        if (saveToFile && File.Exists(logFilePath))
        {
            File.Delete(logFilePath);
            logBuffer.Clear();
            Debug.Log("🗑️ Log file cleared");
        }
    }
    
    [ContextMenu("Show Log File Path")]
    void ShowLogFilePath()
    {
        Debug.Log($"📁 Log file path: {logFilePath}");
    }
}