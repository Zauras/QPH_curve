using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Transforms;
using Unity.Collections;
using System.Collections.Generic;



namespace Master
{
    [UpdateBefore(typeof(CalcPHQcurve))]
    public class LineRendererSystem : ComponentSystem
    {
        struct Chunks
        {
            public readonly int Length;
            public ComponentArray<PathMarker> paths;
            public ComponentArray<LineRenderer> lrs;
        }
        [Inject] private Chunks _chunks;
        
        protected override void OnStartRunning()
        {
            UpdateInjectedComponentGroups();
            SetupLineRenderer(_chunks.lrs[0],
                              BootStrap.Settings.lineWide,
                              BootStrap.Settings.lineColor,
                              BootStrap.Settings.lineRendererMaterial);
        }

        private static void SetupLineRenderer(LineRenderer lineRenderer, float widness, Color color, Material material)
        {
            lineRenderer.material = new Material(material);
            // A simple 2 color gradient with a fixed alpha of 1.0f.
            float alpha = 1.0f;
            Gradient gradient = new Gradient();
            gradient.SetKeys(
                new GradientColorKey[] { new GradientColorKey(color, 0.0f), new GradientColorKey(color, 1.0f) },
                new GradientAlphaKey[] { new GradientAlphaKey(alpha, 0.0f), new GradientAlphaKey(alpha, 1.0f) }
            );
            lineRenderer.colorGradient = gradient;

            lineRenderer.widthMultiplier = widness;
            //lineRenderer.SetPosition(0, new Vector3(0, 0, 0));
        }

        public static void SetPolygonPoints(LineRenderer lineRenderer, List<float3> pointList)
        {         
            lineRenderer.positionCount = pointList.Count;
            for (int i = 0; i < pointList.Count; i++)
            {
                lineRenderer.SetPosition(i, pointList[i]);
            }
        }

        protected override void OnUpdate() { }
    }
}
