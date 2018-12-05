using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Master
{
	public sealed class LibQuaternionAritmetics
	{
		public static LibQuaternionAritmetics H;

        public static quaternion ii = new quaternion(1f, 0f, 0f, 0f); // x, y, z, w
        public static quaternion jj = new quaternion(0f, 1f, 0f, 0f);
        public static quaternion kk = new quaternion(0f, 0f, 1f, 0f);

        public static float4 iif = new float4(1f, 0f, 0f, 0f); // x, y, z, w
        public static float4 jjf = new float4(0f, 1f, 0f, 0f);
        public static float4 kkf = new float4(0f, 0f, 1f, 0f);

        // like in Maple
        public static float4 iifm = new float4(0f, 1f, 0f, 0f); // w, x, y, z
        public static float4 jjfm = new float4(0f, 0f, 1f, 0f);
        public static float4 kkfm = new float4(0f, 0f, 0f, 1f);


        public static float4 Bezier(float t, float4 A0, float4 A1, float4 A2)
        {
            float delta = 1f - t;

            float4 integral = math.pow(delta, 5f) / 5f * A0 +
                           math.pow(delta, 4f) * A0 +

                           4f * math.pow(delta, 4f) * t * A1 + // cia laipsnis 3 arba 4
                           6f * math.pow(delta, 2f) * math.pow(t, 2f) * A1 +
                           4f * math.pow(delta, 3f) * A0 +
                           math.pow(t, 4f) * A2 +
                           math.pow(delta, 5f) * A0 +
                           5f * math.pow(delta, 4f) * t * A1;

            float4 bezierPoint = math.pow(delta, 2f) * A0
                                + 2f * delta * t * A1
                                + math.pow(t, 2f) * A2;

            
            var point = integral / 5f;
            var hod = StarDub(point);

            return hod;
        }
        /*
         * https://github.com/regebro/svg.path/issues/15
        A polynomial function is one that can be written in the form:

f(x) = a_n x^n + a_n−1 x^(n−1) + . . . + a_2 x^2 + a_1 x + a_0
Where a_0 through a_n are constants.

The cubic Bezier curve function looks like this:

f(x) = A(1 - x) ^ 3 + 3B(1 - x)^2 x + 3C(1 - x) x^2 + Dx^3
Where A, B, C and D are the control points and, for the purpose of evaluating an instance of the Bezier curve, are constants.Multiplying everything out and collecting terms yields the expanded polynomial form:

f(x) = (-A + 3B -3C + D)x^3 + (3A - 6B + 3C)x^2 + (-3A + 3B)x + A
If we say:

a = -A + 3B - 3C + D
b = 3A - 6B + 3C
c = -3A + 3B
d = A
Then we have the expanded polynomal:

f(x) = ax^3 + bx^2 + cx + d
Whos indefinite integral is:

a/4 x^4 + b/3 x^3 + c/2 x^2 + dx + E
Where E is a new constant introduced by integration.

The indefinite integral of the quadratic Bezier curve is:

(-A + 3B - 3C + D)/4 x^4 + (A - 2B + C) x^3 + 3/2 (B - A) x^2 + Ax + E
So it is possible to integrate a cubic Bezier but this gives you the area under the curve not the length of the curve.See: http://math.stackexchange.com/questions/922098/arc-length-of-general-polynomial
*/

        public static float4 ClampToZero(float4 h)
        {
            float x, y, z, w;
            float max = 0.000001f, min = -0.000001f;
            if (h.x < max && h.x > min) x = 0f; else x = h.x;
            if (h.y < max && h.y > min) y = 0f; else y = h.y;
            if (h.z < max && h.z > min) z = 0f; else z = h.z;
            if (h.w < max && h.w > min) w = 0f; else w = h.w;
            return new float4(x, y, z, w);
        }

        public static float4 GetDelta(float4 iPoint, float4 jPoint)
        {
            return iPoint - jPoint;
        }

        public static float4 StarOpr(float4 a, float4 b)
        {
            return (Mult(Mult(a, iif), Conj(b)) + Mult(Mult(b, iif), Conj(a))) / 2f;
        }

        public static float4 StarDub(float4 a)
        {
            return StarOpr(a, a);
        }

        public static quaternion Unit(quaternion a)
        {
            // make it norm of one
            return a;
        }

        public static float4 Unit(float4 a)
        {
            // make it norm of one
            return a;
        }

        public static float4 Mult (float4 a, float4 b) {
			float w = (b.w*a.w - b.x*a.x - b.y*a.y - b.z*a.z);
			float x = (b.w*a.x + b.x*a.w - b.y*a.z + b.z*a.y);
			float y = (b.w*a.y + b.x*a.z + b.y*a.w - b.z*a.x);
			float z = (b.w*a.z - b.x*a.y + b.y*a.x + b.z*a.w);
			return new float4 (x, y, z, w);
		}

        public static float4 MultM(float4 a, float4 b)
        {
            float w = (b.w * a.w - b.x * a.x - b.y * a.y - b.z * a.z);
            float x = (b.w * a.x + b.x * a.w - b.y * a.z + b.z * a.y);
            float y = (b.w * a.y + b.x * a.z + b.y * a.w - b.z * a.x);
            float z = (b.w * a.z - b.x * a.y + b.y * a.x + b.z * a.w);
            return new float4(x, y, z, w);
        }

        public static quaternion Mult(quaternion a, quaternion b)
        {
            float w = (b.value.w * a.value.w - b.value.x * a.value.x - b.value.y * a.value.y - b.value.z * a.value.z);
            float x = (b.value.w * a.value.x + b.value.x * a.value.w - b.value.y * a.value.z + b.value.z * a.value.y);
            float y = (b.value.w * a.value.y + b.value.x * a.value.z + b.value.y * a.value.w - b.value.z * a.value.x);
            float z = (b.value.w * a.value.z - b.value.x * a.value.y + b.value.y * a.value.x + b.value.z * a.value.w);
            return new quaternion(x, y, z, w);
        }

        public static float4 Div (float4 a, float4 b) { // a/b laikom, kad a = w, x, y, z; Daryti w pradzioje ar gale ???
			float daliklis = (b.w*b.w + b.x*b.x + b.y*b.y + b.z*b.z);

			float w = (b.w*a.w + b.x*a.x + b.y*a.y + b.z*a.z) / daliklis;
			float x = (b.w*a.x - b.x*a.w - b.y*a.z + b.z*a.y) / daliklis;
			float y = (b.w*a.y + b.x*a.z - b.y*a.w - b.z*a.x) / daliklis;
			float z = (b.w*a.z - b.x*a.y + b.y*a.x - b.z*a.w) / daliklis;
			return new float4 (x, y, z, w);
		}
        public static quaternion Div(quaternion a, quaternion b)
        { // a/b laikom, kad a = w, x, y, z; Daryti w pradzioje ar gale ???
            float daliklis = (b.value.w * b.value.w + b.value.x * b.value.x + b.value.y * b.value.y + b.value.z * b.value.z);

            float w = (b.value.w * a.value.w + b.value.x * a.value.x + b.value.y * a.value.y + b.value.z * a.value.z) / daliklis;
            float x = (b.value.w * a.value.x - b.value.x * a.value.w - b.value.y * a.value.z + b.value.z * a.value.y) / daliklis;
            float y = (b.value.w * a.value.y + b.value.x * a.value.z - b.value.y * a.value.w - b.value.z * a.value.x) / daliklis;
            float z = (b.value.w * a.value.z - b.value.x * a.value.y + b.value.y * a.value.x - b.value.z * a.value.w) / daliklis;
            return new quaternion(x, y, z, w);
        }

        public static float4 Conj(float4 quaternion) {
			float xNeg = quaternion.x * (-1.0f);
			float yNeg = quaternion.y * (-1.0f);
			float zNeg = quaternion.z * (-1.0f);
			return new float4 (xNeg, yNeg, zNeg, quaternion.w);
		}


		public static float4 Abs(float4 quaternion) {
			
			float xx = math.sqrt (quaternion.x * quaternion.x);
			float yy = math.sqrt (quaternion.y * quaternion.y);
			float zz = math.sqrt (quaternion.z * quaternion.z);
			float ww = math.sqrt (quaternion.w * quaternion.w);
			return new float4 (xx, yy, zz, ww);
		}

        public static quaternion Abs(quaternion quaternion)
        {
            float xx = math.sqrt(quaternion.value.x * quaternion.value.x);
            float yy = math.sqrt(quaternion.value.y * quaternion.value.y);
            float zz = math.sqrt(quaternion.value.z * quaternion.value.z);
            float ww = math.sqrt(quaternion.value.w * quaternion.value.w);
            return new quaternion(xx, yy, zz, ww);
        }

        public static float4 Rise_2Deg (Vector4 quat) { // arba kitaip Norm()
			float4 risedQuat = new float4(quat.x*quat.x, quat.y*quat.y, quat.z*quat.z, quat.w*quat.w);
			return risedQuat;
		}

		public static float4 Invers(float4 q) { // Quaternion Invers
			float n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
			return new float4(-1.0f*q.x/n, -1.0f*q.y/n, -1.0f*q.z/n, q.w/n);
			//Quaternion quat = new Quaternion(q.x, q.y, q.z, q.w);
			//quat = Quaternion.Inverse(quat);
			//return new Vector4(quat.x, quat.y, quat.z, quat.w);
		}

		public static float4 Norm(float4 quaternion) {
			float4 normalizedQuat = Div(quaternion, Abs(quaternion));
			return normalizedQuat;
		}

        public static quaternion Norm(quaternion quaternion)
        {
            // quaternion normalizedQuat = Div(quaternion, Abs(quaternion));
            return math.normalizesafe(quaternion);
        }

        public static quaternion Rotation(quaternion quaternionT, float angle ) {
			//if (angle > 720) { } Apsauga, kad angle == [0,720];
			Vector4 quat = Convert_Quat_toVec4(quaternionT);

			Vector4 normQuat = Norm(quat);
			float magnitude = quat.sqrMagnitude;
			Vector3 unitQuat = new Vector4 (quat.x / magnitude,
											quat.y / magnitude, 
											quat.z / magnitude);
			Vector3 unitQuatRot = unitQuat * Mathf.Sin(angle/2.0f);
			Vector4 rotationQuat = new Vector4 (unitQuatRot.x, unitQuatRot.y, unitQuatRot.z, Mathf.Cos(angle/2.0f)); // x,y,z,w
		
			Vector4 rotetedQuat = Mult(Mult(rotationQuat, quat), Invers(quat));
			return Convert_Vec4_toQuat(rotetedQuat);;
		}

		public static float4 Convert_Quat_toVec4(quaternion q) {
			return new float4(q.value.x, q.value.y, q.value.z, q.value.w);
		}

		public static quaternion Convert_Vec4_toQuat(float4 vec4) {
			return new quaternion(vec4.x, vec4.y, vec4.z, vec4.w);
		}

	}

}
