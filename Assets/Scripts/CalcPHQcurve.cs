using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.IntegralTransforms;
using MathNet.Numerics.Integration;
//using Unity.Mathematics.math;
using Unity.Collections;
using UnityEngine.UI;
using Unity.Transforms;
using System.Threading;

namespace Master {
    using DenseMatrix = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
    using H = Master.LibQuaternionAritmetics;
    using m = Unity.Mathematics.math;

    public class CalcPHQcurve : ComponentSystem
    {
        struct Chunks
        {
            public readonly int Length;
           //public EntityArray entities;
            public ComponentArray<Transform> T;
            public ComponentArray<PathMarker> paths;
            public ComponentArray<LineRenderer> lrs;

        }
        [Inject] private Chunks _chunks;

        struct PosAndRotArrs
        {
            public PosAndRotArrs(float3[] posArr, quaternion[] rotArr) : this() {
                this.posArr = posArr;
                this.rotArr = rotArr;
            }
            public float3[] posArr;
            public quaternion[] rotArr;
        }

        private PosAndRotArrs GetPointsPosAndRot(Transform transform)
        {
            float3[] cpsPos = new float3[2];
            quaternion[] cpsRot = new quaternion[2];
            byte index = 0;
            foreach (Transform childTrans in transform)
            {
                if (index >= 2) break;
                if (childTrans.tag == "ControlPoint")
                {
                    cpsPos[index] = childTrans.position;
                    cpsRot[index] = childTrans.rotation;
                    index++;
                }
            }
            return new PosAndRotArrs(cpsPos, cpsRot);
        }

        bool done = false;

        protected override void OnUpdate()
        {           
            if (!done) {
                //Debug.Log("OnUpdate");
                EntityManager em = World.Active.GetOrCreateManager<EntityManager>();
                for (int pth = 0; pth < _chunks.Length; pth++) // Paths
                {
                    Transform pathTransform = _chunks.T[pth];
                    List<float3> pathPoints = new List<float3>();

                    foreach (Transform curveTrans in pathTransform) // Curves
                    {
                        if (curveTrans.tag == "Trajectory")
                        {
                            PosAndRotArrs cpsData = GetPointsPosAndRot(curveTrans);
                            quaternion[] cpsRot = cpsData.rotArr;
                            float3[] cpsPos = cpsData.posArr;

                            // dummy test data:
                            quaternion q0 = (new quaternion( 0f, 5f, 0f, -12f));
                            quaternion q1 = (new quaternion( 0f, 0f, -4f, -3f));
                            float3 p0 = new float3(0f, 0f, 0f);
                            float3 p1 = new float3(.6f, .8f, .5f);

                            Transform p0T = curveTrans.GetChild(0);
                            Transform p1T = curveTrans.GetChild(1);
                            // changing transform to mock data
                            p0T.position = p0;
                            p0T.rotation = q0;
                            p1T.position = p1;
                            p1T.rotation = q1;

                            // quaternion[] RFrames = GetRFrames(cpsRot[0], cpsRot[1]); // how should be
                            quaternion[] RFrames = GetRFrames(q0, q1); // correct with mock data
                            // Debug.Log(RFrames[0]);
                            // Debug.Log(RFrames[1]);
                            // Debug.Log(RFrames[2]);

                            // GetPHCUrve(cpsPos[0], cpsRot[0], cpsPos[1], cpsRot[1], ref pathPoints);
                            GetPHCUrve(p0, q0, p1, q1, ref pathPoints);
                        }
                    }
                    LineRendererSystem.SetPolygonPoints(_chunks.lrs[pth], pathPoints);
                    done = true;
                }
            }
        } 

        private quaternion[] GetRFrames(quaternion q0, quaternion q1)
        {
            Debug.Log("===========R-FRAMES begin ===========");
            //two frames of p0 & p1:
            q0 = (new quaternion(12f, 0f, 5f, 0f)); // mock data like maple
            q1 = (new quaternion(-5f, 0f, 12f, 0f)); // mock data like maple

            quaternion f0x = m.mul(m.mul(q0, H.ii), m.inverse(q0));
            //Debug.Log(f0x);
            quaternion f1x = m.mul(m.mul(q1, H.ii), m.inverse(q1));
            //Debug.Log(f1x);

            quaternion f0y = m.mul(m.mul(q0, H.jj), m.inverse(q0));
            quaternion f1y = m.mul(m.mul(q1, H.jj), m.inverse(q1));

            quaternion f0z = m.mul(m.mul(q0, H.kk), m.inverse(q0));
            quaternion f1z = m.mul(m.mul(q1, H.kk), m.inverse(q1));

            var F1 = m.normalizesafe(q0);
            var F3 = m.normalizesafe(q1);
            var F2 = m.cross(new float3(F3.value.x, F3.value.y, F3.value.z), new float3(F1.value.x, F1.value.y, F1.value.z));
            var F2H = new quaternion(F2.x, F2.y, F2.z, 0f);
            //Debug.Log(F1);
            //Debug.Log(F2H);
            //Debug.Log(F3);

            //Matrix<double> x

            // Test Mathnet.Numerics
            //var ii = Vector<double>.Build.Dense(new double[] { 1, 0, 0, 0 });
            // var F1v = Vector<double>.Build.Dense(new double[] { F1.value.w, F1.value.x, F1.value.y, F1.value.z });
            // var qq = Vector<double>.Build.Dense(4, 1.0);

            float4 F1f = new float4(F1.value.x, F1.value.y, F1.value.z, F1.value.w);
            float4 F2f = new float4(F2.x, F2.y, F2.z, 0f);
            float4 F3f = new float4(F3.value.x, F3.value.y, F3.value.z, F3.value.w);
            float4 qq = new float4(1f, 1f, 1f, 1f);

            var qq1 = H.Mult(qq, H.iif);
            var f1 = H.Mult(F1f, qq);
            var m1 = qq1.w - f1.w;
            //var M1 = qq*ii - ;
            // Debug.Log(f1);
            //var M1 = 

            //Debug.Log("############# first MATRIX ############");
            //float4 qqii = H.Mult(qq, H.iif);
            float4 row1 = H.ClampToZero(H.Mult(new float4(0f, 0f, 0f, 1f), H.iif) - H.Mult(F1f, new float4(0, 0f, 0f, 1f)));
            float4 row2 = H.ClampToZero(H.Mult(new float4(1f, 0f, 0f, 0f), H.iif) - H.Mult(F1f, new float4(1f, 0f, 0f, 0f)));
            float4 row3 = H.ClampToZero(H.Mult(new float4(0f, 1f, 0f, 0f), H.iif) - H.Mult(F1f, new float4(0f, 1f, 0f, 0f)));
            float4 row4 = H.ClampToZero(H.Mult(new float4(0f, 0f, 1f, 0f), H.iif) - H.Mult(F1f, new float4(0f, 0f, 1f, 0f)));

            var M1 = Matrix<float>.Build.DenseOfArray(new float[,] {
                { row1.x, row1.y, row1.z, row1.w },
                { row2.x, row2.y, row2.z, row2.w },
                { row3.x, row3.y, row3.z, row3.w },
                { row4.x, row4.y, row4.z, row4.w },
            });
            //Debug.Log("Rank: "+ M1.Rank());
            //Debug.Log("Solutions Count: "+ M1.Kernel().Length);
            //Debug.Log(M1);

            //Debug.Log("############# second MATRIX ############");
            float4 row5 = H.ClampToZero(H.Mult(new float4(0f, 0f, 0f, 1f), H.jjf) - H.Mult(F2f, new float4(0f, 0f, 0f, 1f)));
            float4 row6 = H.ClampToZero(H.Mult(new float4(1f, 0f, 0f, 0f), H.jjf) - H.Mult(F2f, new float4(1f, 0f, 0f, 0f)));
            float4 row7 = H.ClampToZero(H.Mult(new float4(0f, 1f, 0f, 0f), H.jjf) - H.Mult(F2f, new float4(0f, 1f, 0f, 0f)));
            float4 row8 = H.ClampToZero(H.Mult(new float4(0f, 0f, 1f, 0f), H.jjf) - H.Mult(F2f, new float4(0f, 0f, 1f, 0f)));

            var M2 = Matrix<float>.Build.DenseOfArray(new float[,] {
                { row5.x, row5.y, row5.z, row5.w },
                { row6.x, row6.y, row6.z, row6.w },
                { row7.x, row7.y, row7.z, row7.w },
                { row8.x, row8.y, row8.z, row8.w },
            });
            //Debug.Log("Rank: " + M2.Rank());
            //Debug.Log("Solutions Count: " + M2.Kernel().Length);
            //Debug.Log(M2);

            //Debug.Log("############# third MATRIX ############");
            float4 row9 = H.ClampToZero(H.Mult(new float4(0f, 0f, 0f, 1f), H.jjf) - H.Mult(F3f, new float4(0f, 0f, 0f, 1f)));
            float4 row10 = H.ClampToZero(H.Mult(new float4(1f, 0f, 0f, 0f), H.jjf) - H.Mult(F3f, new float4(1f, 0f, 0f, 0f)));
            float4 row11 = H.ClampToZero(H.Mult(new float4(0f, 1f, 0f, 0f), H.jjf) - H.Mult(F3f, new float4(0f, 1f, 0f, 0f)));
            float4 row12 = H.ClampToZero(H.Mult(new float4(0f, 0f, 1f, 0f), H.jjf) - H.Mult(F3f, new float4(0f, 0f, 1f, 0f)));

            var M3 = Matrix<float>.Build.DenseOfArray(new float[,] {
                { row9.x, row9.y, row9.z, row9.w },
                { row10.x, row10.y, row10.z, row10.w },
                { row11.x, row11.y, row11.z, row11.w },
                { row12.x, row12.y, row12.z, row12.w },
            });
           // Debug.Log("Rank: " + M3.Rank());
            //Debug.Log("Solutions Count: " + M3.Kernel().Length);
            //Debug.Log(M3);
            /*
            Debug.Log("############# solution MATRIX ############");
            var BigM = Matrix<double>.Build.DenseOfArray(new double[,] {
                { row1.x, row1.y, row1.z, row1.w },
                { row2.x, row2.y, row2.z, row2.w },
                { row3.x, row3.y, row3.z, row3.w },
                { row4.x, row4.y, row4.z, row4.w },
                { row5.x, row5.y, row5.z, row5.w },
                { row6.x, row6.y, row6.z, row6.w },
                { row7.x, row7.y, row7.z, row7.w },
                { row8.x, row8.y, row8.z, row8.w },
                { row9.x, row9.y, row9.z, row9.w },
                { row10.x, row10.y, row10.z, row10.w },
                { row11.x, row11.y, row11.z, row11.w },
                { row12.x, row12.y, row12.z, row12.w },
            });
            Debug.Log("Rank: " + BigM.Rank());
            Debug.Log("Solutions Count: " + BigM.Kernel().Length);
            var solution = BigM.Solve(Vector<double>.Build.Dense(new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }));
            Debug.Log(solution);
            Debug.Log(BigM);
            

            Debug.Log("############# solution As Maple MATRIX ############");
            var MapleBigM = Matrix<double>.Build.DenseOfArray(new double[,] {
                { row1.w, row1.x, row1.y, row1.z },
                { row2.w, row2.x, row2.y, row2.z },
                { row3.w, row3.x, row3.y, row3.z }, 
                { row4.w, row4.x, row4.y, row4.z },
                { row5.w, row5.x, row5.y, row5.z },
                { row6.w, row6.x, row6.y, row6.z },
                { row7.w, row7.x, row7.y, row7.z },
                { row8.w, row8.x, row8.y, row8.z },
                { row9.w, row9.x, row9.y, row9.z },
                { row10.w, row10.x, row10.y, row10.z },
                { row11.w, row11.x, row11.y, row11.z },
                { row12.w, row12.x, row12.y, row12.z },
            });
            Debug.Log("Rank: " + MapleBigM.Rank());
            Debug.Log("Solutions Count: " + MapleBigM.Kernel().Length);
            var solved = MapleBigM.Solve(Vector<double>.Build.Dense(new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }));
            Debug.Log(solved);
            Debug.Log(MapleBigM);
            */


            Debug.Log("############# solution MATRIX ############");
            var MM = Matrix<float>.Build.DenseOfArray(new float[,] {
                { row1.x, row1.y, row1.z, row1.w },
                { row2.x, row2.y, row2.z, row2.w },
                { row6.x, row6.y, row6.z, row6.w },
                { row8.x, row8.y, row8.z, row8.w },
            });
            Debug.Log("Rank: " + MM.Rank());
            Debug.Log("Solutions Count: " + MM.Kernel().Length);
            var sol = MM.Kernel()[0].ToArray();
            Debug.Log("Solution: " + sol[0]+", "+sol[1]+", "+sol[2]+", "+sol[3]);
            Debug.Log(MM);


            Debug.Log("############# dummy MATRIX ############");
            var Mtest = Matrix<float>.Build.DenseOfArray(new float[,] {
                { 0f     , 1f / 13f   , 0f     , -5f / 13f     },
                { -1f / 13f    , 0f    , -5f / 13f     , 0f     },
                { 0f     , 0f    , 0f     , 2f     },
                { 0f     , -2f   , 0f     , 0f     },
            });
            var rank = Mtest.Rank();
            sol = Mtest.Kernel()[0].ToArray();
            Debug.Log("Solution: " + sol[0] + ", " + sol[1] + ", " + sol[2] + ", " + sol[3]);

            //var vec = Vector<double>.Build.Dense(new double[] { 0f, 0f, 0f, 0f });
            //var ats = MM.Solve();

            Debug.Log("===========R-FRAMES end ===========");
            return new quaternion[] { F1, F2H, F3 }; ;
        }

        private void GetPHCUrve(float3 point0, quaternion q0H, float3 point1, quaternion q1H, ref List<float3> pathPoints)
        {
            float lmd0 = 1f;
            float lmd1 = 1f;

            float4 p0 = new float4(point0, 1);
            float4 p1 = new float4(point1, 1);

            q0H = m.normalizesafe(q0H);
            q1H = m.normalizesafe(q1H);
            Debug.Log(q0H);
            Debug.Log(q1H);

            float4 q0 = new float4(q0H.value.x, q0H.value.y, q0H.value.z, q0H.value.w);
            float4 q1 = new float4(q1H.value.x, q1H.value.y, q1H.value.z, q1H.value.w);
            Debug.Log("Turi buti teigiamas: " + (q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w));

            // EQP => Q gavimas... ir q0 = Q

            // hardcoded
            q0 = new float4(0f, 0.196116f, 0f, -0.98058f);

            float4 bb = 3f * (lmd0*q0 + lmd1*q1) / 2f;
            Debug.Log("bb:= " + bb);

            float4 cc = 1.5f * (m.pow(lmd0, 2f) * H.StarDub(q0)
                      + m.pow(lmd1, 2f) * H.StarDub(q1)
                      + (lmd0 * lmd1 * H.StarOpr(q0, q1)) / 3f
                      - 5f * (p1 - p0)); 
            Debug.Log("cc:= " + cc);

            // Isdalintas Y skaiciavimas:
            float4 bc = 0.25f * H.StarDub(bb) - cc;
            Debug.Log("bc:= " + bc);
            var nbc = m.length(bc); // norm of bc
            Debug.Log("nbc:= " + nbc);
            float4 virsus = H.iif + bc / nbc;
            Debug.Log("virsus:= " + virsus);
            float4 Y = m.sqrt(nbc) * virsus / m.length(virsus);
            Debug.Log("Y:= " + Y);

            float phi = (float) m.PI / 2.0f;
            Debug.Log("phi:= " + phi);
            //float4 qphi = new float4(m.sin(phi), 0f, 0f, m.cos(phi));
            float4 qphi = new float4(m.sin(phi), 0f, 0f, 0f);
            Debug.Log("qphi:= " + qphi);

            float4 A0 = lmd0 * q0;
            float4 A1 = -0.5f * bb + H.Mult(Y, qphi);
            var o = H.Mult(Y, qphi);
            Debug.Log("A1pat1:= " + (-0.5f * bb));
            Debug.Log("A1pat2:= " + o);
            float4 A2 = lmd1 * q1;
            Debug.Log("A0: " + A0);
            Debug.Log("A1: " + A1);
            Debug.Log("A2: " + A2);


            Debug.Log(H.StarDub(H.Bezier(0.5f, A0, A1, A2)));
 

            // Bezier stuff
            float3[] curvePoints = new float3[50];
            float t = 0f;
            float tStep = 1f / curvePoints.Length;

            for (int i = 0; i <= curvePoints.Length; i++)
            {
                // var aa = H.Bezier(t, A0, A1, A2);
                //Debug.Log(aa);
                //var point = H.StarDub(aa);

                var point = H.Bezier(t, A0, A1, A2);


                //curvePoints[i] = new float3(point.x, point.y, point.z);
                pathPoints.Add(new float3(point.x, point.y, point.z));
                t += tStep;
            }
            //return curvePoints;
        }

    }
}
