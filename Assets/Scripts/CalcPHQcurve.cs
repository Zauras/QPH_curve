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
using MathNet.Numerics;
using System;


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
                            quaternion v0 = new quaternion( 3f, 4f, 1f, 0f);
                            quaternion v1 = new quaternion( 2f, 2f, 0f, 0f);
                            float3 p0 = new float3(0f, 0f, 0f);
                            float3 p1 = new float3(.6f, .8f, .5f);

                            float3 postumis = new float3(1, 2, -2);
                            p0 += postumis;
                            p1 += postumis;

                            Transform p0T = curveTrans.GetChild(0);
                            Transform p1T = curveTrans.GetChild(1);
                            // changing transform to mock data
                            p0T.position = p0;
                            p1T.position = p1;

                            p0T.rotation = v0;
                            p1T.rotation = v1;

                            // quaternion[] RFrames = GetRFrames(cpsRot[0], cpsRot[1]); // how should be
                            
                            // USE THIS ONE
                            float4[] RFrames = GetRFrames(v0, v1);
                            p0T.rotation = H.Float4ToQuat(RFrames[0]);
                            p1T.rotation = H.Float4ToQuat(RFrames[1]);
                            //quaternion[] RFrames = GetRFramesWithMatrix(q0, q1); // correct with mock data
                            //quaternion[] RFramesX = GetRFramesOld(q0, q1); // correct with mock data


                            // Debug.Log(RFrames[0]);s
                            // Debug.Log(RFrames[1]);
                            // Debug.Log(RFrames[2]);



                            // GetPHCUrve(cpsPos[0], cpsRot[0], cpsPos[1], cpsRot[1], ref pathPoints);

                            // SITAS
                            GetPHCUrve(p0, RFrames[0], p1, RFrames[1], ref pathPoints);
                        }
                    }
                    LineRendererSystem.SetPolygonPoints(_chunks.lrs[pth], pathPoints);
                    done = true;
                }
            }
        }

        private float4[] GetRFrames(quaternion v0, quaternion v1)
        {
            Debug.Log("===========R-FRAMES begin ===========");
            //two frames of p0 & p1:

            quaternion F01h = m.normalizesafe(v0);
            float4 F01 = H.QuatToFloat4(F01h);
            //quaternion F02h = m.normalizesafe(new quaternion(-F01h.value.y, F01h.value.x, 0f, 0f));
            //quaternion F03h = H.Mult(F01h, F02h);
            //Debug.Log(F01 + " | " + F02 + " | " + F03);

            quaternion F11h = m.normalizesafe(v1);
            float4 F11 = H.QuatToFloat4(F11h);
            //quaternion F12 = m.normalizesafe(new quaternion(-F11.value.y, F11.value.x, 0f, 0f));
            //quaternion F13 = H.Mult(F11, F12);
            //Debug.Log(F11 + " | " + F12 + " | " + F13);

            float4 one = new float4(0f, 0f, 0f, 1f);

            float4 oq1 = one - H.Mult(F01, H.iif);
            float4 oq2 = F01 + H.iif;
            //Debug.Log(oq1 + " | " + oq2);
            float io1 = m.sqrt(H.QuatLength(oq1));
            float io2 = m.sqrt(H.QuatLength(oq2));
            // Debug.Log(io1 + " | " + io2);

            float4 oq3 = one - H.Mult(F11, H.iif);
            float4 oq4 = F11 + H.iif;
            //Debug.Log(oq3 + " | " + oq4);

            float io3 = m.sqrt(H.QuatLength(oq3));
            float io4 = m.sqrt(H.QuatLength(oq4));
            //Debug.Log(io3 + " | " + io4);

            float4 nq1 = oq1 / io1;
            float4 nq2 = oq2 / io2;
            float4 nq3 = oq3 / io3;
            float4 nq4 = oq4 / io4;
            //Debug.Log(nq1 + " | " + nq2);
            //Debug.Log(nq3 + " | " + nq4);

            float t1 = -Mathf.PI / 12f;
            float t2 = -Mathf.PI / 8f;
            float4 q0 = GetQuatOnSphere(nq1, nq2, t1);
            float4 q1 = GetQuatOnSphere(nq3, nq4, t2);
            //Debug.Log(q0 + " | " + q1);
            Debug.Log("===========R-FRAMES ends ===========");
            return new float4[] { q0, q1 };
        }

        private float4 GetQuatOnSphere(float4 nqBeg, float4 nqEnd, float t)
        {
            return m.cos(t) * nqBeg + m.sin(t) * nqEnd;
        }

        private quaternion[] GetRFramesWithMatrix(quaternion q0, quaternion q1)
        {
            Debug.Log("===========R-FRAMES begin ===========");
            //two frames of p0 & p1:

            quaternion F01h = m.normalizesafe(q0);
            quaternion F02h = m.normalizesafe(new quaternion(-F01h.value.y, F01h.value.x, 0f, 0f));
            quaternion F03h = H.Mult(F01h, F02h);
            //Debug.Log(F01 + " | " + F02 + " | " + F03);

            quaternion F11 = m.normalizesafe(q1);
            quaternion F12 = m.normalizesafe(new quaternion(-F11.value.y, F11.value.x, 0f, 0f));
            quaternion F13 = H.Mult(F11, F12);
            //Debug.Log(F11 + " | " + F12 + " | " + F13);

            //transform to float4 wxyz:
            float4 F01 = H.TransformToQWYZ(H.QuatToFloat4(F01h));
            float4 F02 = H.TransformToQWYZ(H.QuatToFloat4(F02h));
            float4 F03 = H.TransformToQWYZ(H.QuatToFloat4(F03h));

            //Debug.Log(H.Matrix4of4ToMatrixBuild(H.MulPQ(H.QuatToFloat4(F01))));

            float4Matrix m1 = H.SubMatrix(H.MulQP(H.iif), H.MulPQ(F01));
            Matrix<float> M1 = H.BuildMatrix(m1);
            Debug.Log(H.iifm);
            Debug.Log(H.BuildMatrix(H.MulQP(H.iifm)));
            Debug.Log(H.BuildMatrix(H.MulPQ(F01)));
            Debug.Log(M1);

            float4Matrix m2 = H.SubMatrix(H.MulQP(H.jjf), H.MulPQ(F02));
            Matrix<float> M2 = H.BuildMatrix(m2);
            Debug.Log(H.jjfm);
            Debug.Log(H.BuildMatrix(H.MulQP(H.jjfm)));
            Debug.Log(H.BuildMatrix(H.MulPQ(F02)));
            //Debug.Log(M2);

            float4Matrix m3 = H.SubMatrix(H.MulQP(H.kkf), H.MulPQ(F03));
            Matrix<float> M3 = H.BuildMatrix(m3);
            Debug.Log(H.kkfm);
            Debug.Log(H.BuildMatrix(H.MulQP(H.kkfm)));
            Debug.Log(H.BuildMatrix(H.MulPQ(F03)));
            //Debug.Log(M3);

           // Debug.Log("############# converted FULL MATRIX ############");
            var BigM = Matrix<double>.Build.DenseOfArray(new double[,] {
                { m1.r0.x, m1.r0.y, m1.r0.z, m1.r0.w },
                { m1.r1.x, m1.r1.y, m1.r1.z, m1.r1.w },
                { m1.r2.x, m1.r2.y, m1.r2.z, m1.r2.w },
                { m1.r3.x, m1.r3.y, m1.r3.z, m1.r3.w },

                { m2.r0.x, m2.r0.y, m2.r0.z, m2.r0.w },
                { m2.r1.x, m2.r1.y, m2.r1.z, m2.r1.w },
                { m2.r2.x, m2.r2.y, m2.r2.z, m2.r2.w },
                { m2.r3.x, m2.r3.y, m2.r3.z, m2.r3.w },

                { m3.r0.x, m3.r0.y, m3.r0.z, m3.r0.w },
                { m3.r1.x, m3.r1.y, m3.r1.z, m3.r1.w },
                { m3.r2.x, m3.r2.y, m3.r2.z, m3.r2.w },
                { m3.r3.x, m3.r3.y, m3.r3.z, m3.r3.w },
            });
            //Debug.Log("Rank: " + BigM.Rank());
            //Debug.Log("Solutions Count: " + BigM.Kernel().Length);
            // Debug.Log("Solutions Count: " + BigM.Kernel()[0]);
            //var solution = BigM.Solve(Vector<double>.Build.Dense(new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }));
            //Debug.Log(solution);
             Debug.Log(BigM);

            Debug.Log("############# converted FULL MATRIX ############");
            var MM = Matrix<float>.Build.DenseOfArray(new float[,] {
                { m1.r0.x, m1.r0.y, m1.r0.z, m1.r0.w },
                { m2.r0.x, m2.r0.y, m2.r0.z, m2.r0.w },
                { m3.r0.x, m3.r0.y, m3.r0.z, m3.r0.w },
            });
            Debug.Log("Rank: " + MM.Rank());
            Debug.Log("Solutions Count: " + MM.Kernel().Length);
            if (MM.Kernel().Length > 0) Debug.Log("Solutions: " + MM.Kernel()[0][0]);
           // var solution = MM.Solve(Vector<float>.Build.Dense(new float[] { 0, 0, 0, 0 }));
            //Debug.Log(solution);
            Debug.Log(MM);

            Debug.Log("===========R-FRAMES ends ===========");
            return null;
        }

        private quaternion[] GetRFramesOld(quaternion q0, quaternion q1)
        {
            Debug.Log("===========R-FRAMES begin ===========");
            //two frames of p0 & p1:
           // q0 = (new quaternion(12f, 0f, 5f, 0f)); // mock data like maple
            //q1 = (new quaternion(-5f, 0f, 12f, 0f)); // mock data like maple

           // q0 = (new quaternion(0f, -5f, 0f, -12f)); // mock data like maple
            //q1 = (new quaternion(0f, 0f, -4f, -3f)); // mock data like maple

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
            float4 row1 = H.Round(H.Mult(new float4(0f, 0f, 0f, 1f), H.iif) - H.Mult(F1f, new float4(0, 0f, 0f, 1f)));
            float4 row2 = H.Round(H.Mult(new float4(1f, 0f, 0f, 0f), H.iif) - H.Mult(F1f, new float4(1f, 0f, 0f, 0f)));
            float4 row3 = H.Round(H.Mult(new float4(0f, 1f, 0f, 0f), H.iif) - H.Mult(F1f, new float4(0f, 1f, 0f, 0f)));
            float4 row4 = H.Round(H.Mult(new float4(0f, 0f, 1f, 0f), H.iif) - H.Mult(F1f, new float4(0f, 0f, 1f, 0f)));

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
            float4 row5 = H.Round(H.Mult(new float4(0f, 0f, 0f, 1f), H.jjf) - H.Mult(F2f, new float4(0f, 0f, 0f, 1f)));
            float4 row6 = H.Round(H.Mult(new float4(1f, 0f, 0f, 0f), H.jjf) - H.Mult(F2f, new float4(1f, 0f, 0f, 0f)));
            float4 row7 = H.Round(H.Mult(new float4(0f, 1f, 0f, 0f), H.jjf) - H.Mult(F2f, new float4(0f, 1f, 0f, 0f)));
            float4 row8 = H.Round(H.Mult(new float4(0f, 0f, 1f, 0f), H.jjf) - H.Mult(F2f, new float4(0f, 0f, 1f, 0f)));

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
            float4 row9 = H.Round(H.Mult(new float4(0f, 0f, 0f, 1f), H.kkf) - H.Mult(F3f, new float4(0f, 0f, 0f, 1f)));
            float4 row10 = H.Round(H.Mult(new float4(1f, 0f, 0f, 0f), H.kkf) - H.Mult(F3f, new float4(1f, 0f, 0f, 0f)));
            float4 row11 = H.Round(H.Mult(new float4(0f, 1f, 0f, 0f), H.kkf) - H.Mult(F3f, new float4(0f, 1f, 0f, 0f)));
            float4 row12 = H.Round(H.Mult(new float4(0f, 0f, 1f, 0f), H.kkf) - H.Mult(F3f, new float4(0f, 0f, 1f, 0f)));

            var M3 = Matrix<float>.Build.DenseOfArray(new float[,] {
                { row9.x, row9.y, row9.z, row9.w },
                { row10.x, row10.y, row10.z, row10.w },
                { row11.x, row11.y, row11.z, row11.w },
                { row12.x, row12.y, row12.z, row12.w },
            });
           // Debug.Log("Rank: " + M3.Rank());
            //Debug.Log("Solutions Count: " + M3.Kernel().Length);
            //Debug.Log(M3);
            
            Debug.Log("############# converted FULL MATRIX ############");
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
            //Debug.Log("Rank: " + BigM.Rank());
            //Debug.Log("Solutions Count: " + BigM.Kernel().Length);
            //var solution = BigM.Solve(Vector<double>.Build.Dense(new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }));
            //Debug.Log(solution);
            //Debug.Log(BigM);
            

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
            //Debug.Log("Rank: " + MapleBigM.Rank());
            //Debug.Log("Solutions Count: " + MapleBigM.Kernel().Length);
            //var solved = MapleBigM.Solve(Vector<double>.Build.Dense(new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }));
            //Debug.Log(solved);
            //Debug.Log(MapleBigM);
            


            Debug.Log("############# solution MATRIX ############");
            var MM = Matrix<float>.Build.DenseOfArray(new float[,] {
                { row5.w, row5.x, row5.y, row5.z },
                { row6.w, row6.x, row6.y, row6.z },
                { row11.w, row11.x, row11.y, row11.z },
                { row12.w, row12.x, row12.y, row12.z },
            });
            Debug.Log("Rank: " + MM.Rank());
            Debug.Log("Solutions Count: " + MM.Kernel().Length);
            //var sol = MM.Kernel()[0].ToArray();
            //Debug.Log("Solution: " + sol[0]+", "+sol[1]+", "+sol[2]+", "+sol[3]);
            if (MapleBigM.Kernel().Length > 0) Debug.Log(MM.Kernel()[0]);
            Debug.Log(MM);

            Debug.Log("===========R-FRAMES end ===========");
            return new quaternion[] { F1, F2H, F3 }; ;
        }

        private void GetPHCUrve(float3 point0, float4 q0, float3 point1, float4 q1, ref List<float3> pathPoints)
        {
            //float lmd0 = lmd1 = 1.2f;
            //float4 postumis = new float4(1f, 2f, -2f, 0f);
            float4 p0 = new float4(point0, 0f);
            float4 p1 = new float4(point1, 0f);
            float lmd0 = m.sqrt(H.QuatLength(p0 - p1));
            float lmd1 = lmd0;

            Debug.Log("Turi buti teigiamas: " + (q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w));

            // EQP => Q gavimas... ir q0 = Q

            // HARDCODED !!!!!!!!
            //q0 = new float4(0f, 0.1961161f, 0f, -0.9805806f);

            //q0 = new float4(-0.1961161f, 0f, 0.9805806f, 0f);

            float4 bb = 3f * (lmd0*q0 + lmd1*q1) / 2f;
            //Debug.Log("bb:= " + bb);

            float4 cc = 1.5f * (m.pow(lmd0, 2f) * H.StarDub(q0)
                      + m.pow(lmd1, 2f) * H.StarDub(q1)
                      + (lmd0 * lmd1 * H.StarOpr(q0, q1)) / 3f
                      - 5f * (p1 - p0)); 
            //Debug.Log("cc:= " + cc);

            // Isdalintas Y skaiciavimas:
            float4 bc = 0.25f * H.StarDub(bb) - cc;
            //Debug.Log("bc:= " + bc);
            var nbc = m.length(bc); // norm of bc
            //Debug.Log("nbc:= " + nbc);
            float4 virsus = H.iif + bc / nbc;
           // Debug.Log("virsus:= " + virsus);
            float4 Y = m.sqrt(nbc) * virsus / m.length(virsus);
            //Debug.Log("Y:= " + Y);

            float phi = -Mathf.PI / 2.0f;
           // Debug.Log("phi:= " + phi);
            //float4 qphi = new float4(m.sin(phi), 0f, 0f, m.cos(phi));
            float4 qphi = H.Round(new float4(m.sin(phi), 0f, 0f, m.cos(phi)));
           //Debug.Log("qphi:= " + qphi);

            float4 A0 = lmd0 * q0;
            float4 A1 = -0.5f * bb + H.Mult(Y, qphi);
            var o = H.Mult(Y, qphi);
           // Debug.Log("A1pat1:= " + (-0.5f * bb));
           // Debug.Log("A1pat2:= " + o);
            float4 A2 = lmd1 * q1;
           // Debug.Log("A0: " + A0);
           // Debug.Log("A1: " + A1);
           // Debug.Log("A2: " + A2);


            // bezier control points:
            float4 cp0 = H.StarDub(A0);
            float4 cp1 = H.StarOpr(A1, A0);
            float4 cp2 = 1f / 3f * (2 * H.StarDub(A1) + H.StarOpr(A2, A0));
            float4 cp3 = H.StarOpr(A1, A2);
            float4 cp4 = H.StarDub(A2);

            // integrated bezier control points:
            float4 icp0 = p0;
            float4 icp1 = icp0 + 0.2f * cp0;
            float4 icp2 = icp1 + 0.2f * cp1;
            float4 icp3 = icp2 + 0.2f * cp2;
            float4 icp4 = icp3 + 0.2f * cp3;
            float4 icp5 = icp4 + 0.2f * cp4;

            Debug.Log(cp0 + "; " + cp1 + "; " + cp2 + "; " + cp3 + "; " + cp4);
            Debug.Log(icp0 + "; " + icp1 + "; " + icp2 + "; " + icp3 + "; " + icp4 + "; " + icp5);


             float4[] bezPoints = { cp0, cp1, cp2, cp3, cp4 };
             float4[] intBezPoints = { icp0, icp1, icp2, icp3, icp4, icp5 };

            // Bezier stuff
            float3[] curvePoints = new float3[10];
            float t = 0f;
            float tStep = 1f / curvePoints.Length;

            for (int i = 0; i <= curvePoints.Length; i++)
            {
                // var aa = H.Bezier(t, A0, A1, A2);
                //Debug.Log(aa);
                //var point = H.StarDub(aa);

                var point = H.Bezier(t, bezPoints, intBezPoints);
                //var point = H.StarDub(H.Bezier(t, bezPoints, intBezPoints));

                //var point = H.Integral(bezPoints, t, t + tStep);


                //curvePoints[i] = new float3(point.x, point.y, point.z);
                pathPoints.Add(new float3(point.x, point.y, point.z));
                t += tStep;
            }
            //return curvePoints;
        }

    }
}
