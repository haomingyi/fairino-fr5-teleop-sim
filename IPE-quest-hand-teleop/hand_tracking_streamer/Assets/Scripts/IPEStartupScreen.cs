using System.Collections;
using UnityEngine;
using UnityEngine.UI;

// Author: haoming
// A camera-space startup card is used because Unity's native custom splash
// enforces a two-second minimum. This path is stereo-safe on Quest/OpenXR and
// gives the IPE brand page an exact one-second visible interval.
public static class IPEStartupScreen
{
    private const float VisibleSeconds = 1.0f;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Show()
    {
        var host = new GameObject("IPE Startup Screen");
        Object.DontDestroyOnLoad(host);
        host.AddComponent<StartupController>();
    }

    private sealed class StartupController : MonoBehaviour
    {
        private IEnumerator Start()
        {
            Camera xrCamera = null;
            while (xrCamera == null)
            {
                xrCamera = Camera.main;
                yield return null;
            }

            var canvas = gameObject.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceCamera;
            canvas.worldCamera = xrCamera;
            canvas.planeDistance = Mathf.Max(xrCamera.nearClipPlane + 0.05f, 0.35f);
            canvas.sortingOrder = short.MaxValue;

            var scaler = gameObject.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.screenMatchMode = CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
            scaler.matchWidthOrHeight = 0.5f;

            var background = CreateImage("Black Background", transform, Color.black);
            Fill(background.rectTransform);

            var logoSprite = Resources.Load<Sprite>("IPEGroupLimitedLogo");
            if (logoSprite != null)
            {
                var logo = CreateImage("IPE Group Limited", background.transform, Color.white);
                logo.sprite = logoSprite;
                logo.preserveAspect = true;
                logo.rectTransform.anchorMin = new Vector2(0.16f, 0.22f);
                logo.rectTransform.anchorMax = new Vector2(0.84f, 0.78f);
                logo.rectTransform.offsetMin = Vector2.zero;
                logo.rectTransform.offsetMax = Vector2.zero;
            }
            else
            {
                Debug.LogError("IPE startup image was not found in Resources.");
            }

            yield return new WaitForSecondsRealtime(VisibleSeconds);
            Destroy(gameObject);
        }

        private static Image CreateImage(string objectName, Transform parent, Color color)
        {
            var child = new GameObject(objectName, typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
            child.transform.SetParent(parent, false);
            var image = child.GetComponent<Image>();
            image.color = color;
            image.raycastTarget = true;
            return image;
        }

        private static void Fill(RectTransform rect)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;
        }
    }
}
