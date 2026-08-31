using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class IH01Build
{
    public static void Build()
    {
        var projectRoot = Directory.GetParent(Application.dataPath).FullName;
        PlayerSettings.companyName = "Haoming";
        PlayerSettings.productName = "IPE Quest Hand Teleop";
        PlayerSettings.bundleVersion = "0.1.0";
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android, "com.haoming.ipe.handteleop");
        PlayerSettings.Android.bundleVersionCode = 1;
        ConfigureIpeSplash();

        var output = Path.Combine(projectRoot, "Builds", "ih01-quest-hand-teleop.apk");
        Directory.CreateDirectory(Path.GetDirectoryName(output));
        var options = new BuildPlayerOptions
        {
            scenes = new[] { "Assets/Scenes/Scene.unity" },
            locationPathName = output,
            target = BuildTarget.Android,
            options = BuildOptions.None,
        };
        var report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
            throw new System.Exception($"IH01 Quest build failed: {report.summary.result}");
        Debug.Log($"IH01 Quest build: {output} ({report.summary.totalSize} bytes)");
    }

    [MenuItem("IH01/Configure IPE Splash Screen")]
    public static void ConfigureIpeSplash()
    {
        const string logoPath = "Assets/Resources/IPEGroupLimitedLogo.png";
        var importer = AssetImporter.GetAtPath(logoPath) as TextureImporter;
        if (importer == null)
            throw new FileNotFoundException("IPE splash image is missing", logoPath);

        if (importer.textureType != TextureImporterType.Sprite ||
            importer.spriteImportMode != SpriteImportMode.Single || importer.mipmapEnabled)
        {
            importer.textureType = TextureImporterType.Sprite;
            importer.spriteImportMode = SpriteImportMode.Single;
            importer.mipmapEnabled = false;
            importer.npotScale = TextureImporterNPOTScale.None;
            importer.SaveAndReimport();
        }

        var logoSprite = AssetDatabase.LoadAssetAtPath<Sprite>(logoPath);
        var logoTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(logoPath);
        if (logoSprite == null || logoTexture == null)
            throw new System.InvalidOperationException("IPE splash image did not import as a Sprite/Texture2D");

        // A custom Unity splash logo has a two-second minimum. Keep the native
        // loading surface black and let IPEStartupScreen render the exact
        // one-second branded page after the XR camera becomes available.
        PlayerSettings.SplashScreen.show = false;
        PlayerSettings.SplashScreen.showUnityLogo = false;
        PlayerSettings.SplashScreen.backgroundColor = Color.black;
        PlayerSettings.SplashScreen.animationMode = PlayerSettings.SplashScreen.AnimationMode.Static;
        PlayerSettings.SplashScreen.logos = System.Array.Empty<PlayerSettings.SplashScreenLogo>();
        PlayerSettings.virtualRealitySplashScreen = null;
        EditorUtility.SetDirty(logoTexture);
        AssetDatabase.SaveAssets();
        Debug.Log("Configured black native loading surface and one-second IPE startup screen.");
    }
}
