using System.IO;
using UnityEditor;
using UnityEngine;

namespace OPTINAV.Editor
{
    /// <summary>
    /// Utility to control Play Mode programmatically for automated M3 test validation.
    /// Watches Temp/play_command.txt for "play" or "stop" commands.
    /// </summary>
    [InitializeOnLoad]
    public static class OPTINAVPlayModeControl
    {
        private const string CommandFilePath = "Temp/play_command.txt";

        static OPTINAVPlayModeControl()
        {
            EditorApplication.update += CheckPlayCommand;
        }

        [MenuItem("OPTINAV/Enter Play Mode")]
        public static void EnterPlayMode()
        {
            EditorApplication.isPlaying = true;
        }

        [MenuItem("OPTINAV/Stop Play Mode")]
        public static void StopPlayMode()
        {
            EditorApplication.isPlaying = false;
        }

        private static void CheckPlayCommand()
        {
            if (File.Exists(CommandFilePath))
            {
                try
                {
                    string cmd = File.ReadAllText(CommandFilePath).Trim().ToLower();
                    File.Delete(CommandFilePath);

                    if (cmd == "refresh")
                    {
                        Debug.Log("[OPTINAVPlayModeControl] Refreshing AssetDatabase...");
                        AssetDatabase.Refresh();
                    }
                    else if (cmd == "play" && !EditorApplication.isPlaying)
                    {
                        Debug.Log("[OPTINAVPlayModeControl] Starting Play Mode...");
                        EditorApplication.isPlaying = true;
                    }
                    else if (cmd == "stop" && EditorApplication.isPlaying)
                    {
                        Debug.Log("[OPTINAVPlayModeControl] Stopping Play Mode...");
                        EditorApplication.isPlaying = false;
                    }
                }
                catch
                {
                    // Ignore file access collision
                }
            }
        }
    }
}
