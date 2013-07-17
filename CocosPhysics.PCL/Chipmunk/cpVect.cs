/* Copyright (c) 2007 Scott Lembcke
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
using System;

namespace CocosPhysics.Chipmunk
{
    public struct cpVect { 
        public double x; public double y; 
        public cpVect(double _x, double _y)
        {
            x = _x;
            y = _y;
        }

        public cpVect(cpVect pt)
        {
            x = pt.x;
            y = pt.y;
        }

        public static bool Equal( cpVect point1,  cpVect point2)
        {
            return ((point1.x == point2.x) && (point1.y == point2.y));
        }

        public cpVect Offset(double dx, double dy)
        {
            cpVect pt;
            pt.x = x + dx;
            pt.y = y + dy;
            return pt;
        }

        public cpVect Reverse
        {
            get { return new cpVect(-x, -t); }
        }

        public override int GetHashCode()
        {
            return x.GetHashCode() + y.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return (Equals((cpVect) obj));
        }

        public bool Equals(cpVect p)
        {
            return x == p.x && y == p.y;
        }

        public override string ToString()
        {
            return string.Format("cpVect : ({0:N3} {1:N3})", x, y);
        }

        public double DistanceSQ( cpVect v2)
        {
            return Sub( v2).LengthSQ;
        }

        public static double DistanceSQ( cpVect v1,  cpVect v2)
        {
            return v1.Sub( v2).LengthSQ;
        }

        public static double Distance( cpVect v1,  cpVect v2)
        {
            return Math.Sqrt(v1.Sub( v2).LengthSQ);
        }

        public static cpVect Multiply( cpVect v1, double v)
        {
            cpVect pt;
            pt.x = v1.x * v;
            pt.y = v1.y * v;
            return (pt);
        }

        public static cpVect Add( cpVect v1,  cpVect v2)
        {
            cpVect v;
            v.x = v1.x + v2.x;
            v.y = v1.y + v2.y;
            return(v);
        }

        public cpVect Sub( cpVect v2)
        {
            cpVect pt;
            pt.x = x - v2.x;
            pt.y = y - v2.y;
            return pt;
        }

        public static cpVect Sub(cpVect v1, cpVect v2)
        {
            cpVect pt;
            pt.x = v1.x - v2.x;
            pt.y = v1.y - v2.y;
            return pt;
        }

        public double LengthSQ
        {
            get { return x * x + y * y; }
        }

        public double LengthSquare
        {
            get { return LengthSQ; }
        }

        /// <summary>
        ///     Computes the length of this point as if it were a vector with XY components relative to the
        ///     origin. This is computed each time this property is accessed, so cache the value that is
        ///     returned.
        /// </summary>
        public double Length
        {
            get { return (double) System.Math.Sqrt(x * x + y * y); }
        }

        /// <summary>
        ///     Inverts the direction or location of the Y component.
        /// </summary>
        public cpVect InvertY
        {
            get
            {
                cpVect pt;
                pt.x = x;
                pt.y = -y;
                return pt;
            }
        }

        /// <summary>
        ///     Normalizes the components of this point (convert to mag 1), and returns the orignial
        ///     magnitude of the vector defined by the XY components of this point.
        /// </summary>
        /// <returns></returns>
        public double Normalize()
        {
            var mag = (double) System.Math.Sqrt(x * x + y * y);
            if (mag < double.Epsilon)
            {
                return (0f);
            }
            double l = 1f / mag;
            x *= l;
            y *= l;
            return (mag);
        }

        #region Static Methods

        public static cpVect Lerp(cpVect a, cpVect b, double alpha)
        {
            return (a * (1f - alpha) + b * alpha);
        }


        /** @returns if points have fuzzy equality which means equal with some degree of variance.
            @since v0.99.1
        */

        public static bool FuzzyEqual(cpVect a, cpVect b, double variance)
        {
            if (a.x - variance <= b.x && b.x <= a.x + variance)
                if (a.y - variance <= b.y && b.y <= a.y + variance)
                    return true;

            return false;
        }


        /** Multiplies a nd b components, a.x*b.x, a.y*b.y
            @returns a component-wise multiplication
            @since v0.99.1
        */

        public static cpVect MultiplyComponents(cpVect a, cpVect b)
        {
            cpVect pt;
            pt.x = a.x * b.x;
            pt.y = a.y * b.y;
            return pt;
        }

        /** @returns the signed angle in radians between two vector directions
            @since v0.99.1
        */

        public static double AngleSigned(cpVect a, cpVect b)
        {
            cpVect a2 = Normalize(a);
            cpVect b2 = Normalize(b);
            var angle = (double) System.Math.Atan2(a2.x * b2.y - a2.y * b2.x, DotProduct(a2, b2));

            if (System.Math.Abs(angle) < double.Epsilon)
            {
                return 0.0f;
            }

            return angle;
        }

        /** Rotates a point counter clockwise by the angle around a pivot
            @param v is the point to rotate
            @param pivot is the pivot, naturally
            @param angle is the angle of rotation cw in radians
            @returns the rotated point
            @since v0.99.1
        */

        public static cpVect RotateByAngle(cpVect v, cpVect pivot, double angle)
        {
            cpVect r = v - pivot;
            double cosa = (double) Math.Cos(angle), sina = (double) Math.Sin(angle);
            double t = r.x;

            r.x = t * cosa - r.y * sina + pivot.x;
            r.y = t * sina + r.y * cosa + pivot.y;

            return r;
        }

        /** A general line-line intersection test
         @param p1 
            is the startpoint for the first line P1 = (p1 - p2)
         @param p2 
            is the endpoint for the first line P1 = (p1 - p2)
         @param p3 
            is the startpoint for the second line P2 = (p3 - p4)
         @param p4 
            is the endpoint for the second line P2 = (p3 - p4)
         @param s 
            is the range for a hitpoint in P1 (pa = p1 + s*(p2 - p1))
         @param t
            is the range for a hitpoint in P3 (pa = p2 + t*(p4 - p3))
         @return bool 
            indicating successful intersection of a line
            note that to truly test intersection for segments we have to make 
            sure that s & t lie within [0..1] and for rays, make sure s & t > 0
            the hit point is		p3 + t * (p4 - p3);
            the hit point also is	p1 + s * (p2 - p1);
         @since v0.99.1
         */

        public static bool LineIntersect(cpVect A, cpVect B, cpVect C, cpVect D,  double S,  double T)
        {
            // FAIL: Line undefined
            if ((A.x == B.x && A.y == B.y) || (C.x == D.x && C.y == D.y))
            {
                return false;
            }

            double BAx = B.x - A.x;
            double BAy = B.y - A.y;
            double DCx = D.x - C.x;
            double DCy = D.y - C.y;
            double ACx = A.x - C.x;
            double ACy = A.y - C.y;

            double denom = DCy * BAx - DCx * BAy;

            S = DCx * ACy - DCy * ACx;
            T = BAx * ACy - BAy * ACx;

            if (denom == 0)
            {
                if (S == 0 || T == 0)
                {
                    // Lines incident
                    return true;
                }
                // Lines parallel and not incident
                return false;
            }

            S = S / denom;
            T = T / denom;

            // Point of intersection
            // CGPoint P;
            // P.x = A.x + *S * (B.x - A.x);
            // P.y = A.y + *S * (B.y - A.y);

            return true;
        }

        /*
        ccpSegmentIntersect returns YES if Segment A-B intersects with segment C-D
        @since v1.0.0
        */

        public static bool SegmentIntersect(cpVect A, cpVect B, cpVect C, cpVect D)
        {
            double S = 0, T = 0;

            if (LineIntersect(A, B, C, D,  S,  T)
                && (S >= 0.0f && S <= 1.0f && T >= 0.0f && T <= 1.0f))
            {
                return true;
            }

            return false;
        }

        /*
        ccpIntersectPoint returns the intersection point of line A-B, C-D
        @since v1.0.0
        */

        public static cpVect IntersectPoint(cpVect A, cpVect B, cpVect C, cpVect D)
        {
            double S = 0, T = 0;

            if (LineIntersect(A, B, C, D,  S,  T))
            {
                // Point of intersection
                cpVect P;
                P.x = A.x + S * (B.x - A.x);
                P.y = A.y + S * (B.y - A.y);
                return P;
            }

            return Zero;
        }

        /** Converts radians to a normalized vector.
            @return cpVect
            @since v0.7.2
        */

        public static cpVect ForAngle(double a)
        {
            cpVect pt;
            pt.x = (double) Math.Cos(a);
            pt.y = (double) Math.Sin(a);
            return pt;
            //            return CreatePoint((double)Math.Cos(a), (double)Math.Sin(a));
        }

        /** Converts a vector to radians.
            @return CGdouble
            @since v0.7.2
        */

        public static double ToAngle(cpVect v)
        {
            return (double) Math.Atan2(v.y, v.x);
        }


        /** Clamp a value between from and to.
            @since v0.99.1
        */

        public static double Clamp(double value, double min_inclusive, double max_inclusive)
        {
            if (min_inclusive > max_inclusive)
            {
                double ftmp = min_inclusive;
                min_inclusive = max_inclusive;
                max_inclusive = ftmp;
            }

            return value < min_inclusive ? min_inclusive : value < max_inclusive ? value : max_inclusive;
        }

        /** Clamp a point between from and to.
            @since v0.99.1
        */

        public static cpVect Clamp(cpVect p, cpVect from, cpVect to)
        {
            cpVect pt;
            pt.x = Clamp(p.x, from.x, to.x);
            pt.y = Clamp(p.y, from.y, to.y);
            return pt;
            //            return CreatePoint(Clamp(p.x, from.x, to.x), Clamp(p.y, from.y, to.y));
        }

        /** Quickly convert CCSize to a cpVect
            @since v0.99.1
        */

        [Obsolete("Use explicit cast (cpVect)size.")]
        public static cpVect FromSize(CCSize s)
        {
            cpVect pt;
            pt.x = s.Width;
            pt.y = s.Height;
            return pt;
        }

        /**
         * Allow Cast CCSize to cpVect
         */

        public static explicit operator cpVect(CCSize size)
        {
            cpVect pt;
            pt.x = size.Width;
            pt.y = size.Height;
            return pt;
        }

        public static cpVect Perp(cpVect p)
        {
            cpVect pt;
            pt.x = -p.y;
            pt.y = p.x;
            return pt;
        }

        public static double Dot(cpVect p1, cpVect p2)
        {
            return p1.x * p2.x + p1.y * p2.y;
        }

        public static double Distance(cpVect v1, cpVect v2)
        {
            return (v1 - v2).Length;
        }

        public static cpVect Normalize(cpVect p)
        {
            double x = p.x;
            double y = p.y;
            double l = 1f / (double) Math.Sqrt(x * x + y * y);
            cpVect pt;
            pt.x = x * l;
            pt.y = y * l;
            return pt;
        }

        public static cpVect Midpoint(cpVect p1, cpVect p2)
        {
            cpVect pt;
            pt.x = (p1.x + p2.x) / 2f;
            pt.y = (p1.y + p2.y) / 2f;
            return pt;
        }

        public static double DotProduct(cpVect v1, cpVect v2)
        {
            return v1.x * v2.x + v1.y * v2.y;
        }

        /** Calculates cross product of two points.
            @return CGdouble
            @since v0.7.2
        */

        public static double CrossProduct(cpVect v1, cpVect v2)
        {
            return v1.x * v2.y - v1.y * v2.x;
        }

        /** Calculates perpendicular of v, rotated 90 degrees counter-clockwise -- cross(v, perp(v)) >= 0
            @return cpVect
            @since v0.7.2
        */

        public static cpVect PerpendicularCounterClockwise(cpVect v)
        {
            cpVect pt;
            pt.x = -v.y;
            pt.y = v.x;
            return pt;
        }

        /** Calculates perpendicular of v, rotated 90 degrees clockwise -- cross(v, rperp(v)) <= 0
            @return cpVect
            @since v0.7.2
        */

        public static cpVect PerpendicularClockwise(cpVect v)
        {
            cpVect pt;
            pt.x = v.y;
            pt.y = -v.x;
            return pt;
        }

        /** Calculates the projection of v1 over v2.
            @return cpVect
            @since v0.7.2
        */

        public static cpVect Project(cpVect v1, cpVect v2)
        {
            double dp1 = v1.x * v2.x + v1.y * v2.y;
            double dp2 = v2.LengthSQ;
            double f = dp1 / dp2;
            cpVect pt;
            pt.x = v2.x * f;
            pt.y = v2.y * f;
            return pt;
            // return Multiply(v2, DotProduct(v1, v2) / DotProduct(v2, v2));
        }

        /** Rotates two points.
            @return cpVect
            @since v0.7.2
        */

        public static cpVect Rotate(cpVect v1, cpVect v2)
        {
            cpVect pt;
            pt.x = v1.x * v2.x - v1.y * v2.y;
            pt.y = v1.x * v2.y + v1.y * v2.x;
            return pt;
        }

        /** Unrotates two points.
            @return cpVect
            @since v0.7.2
        */

        public static cpVect Unrotate(cpVect v1, cpVect v2)
        {
            cpVect pt;
            pt.x = v1.x * v2.x + v1.y * v2.y;
            pt.y = v1.y * v2.x - v1.x * v2.y;
            return pt;
        }

        #endregion

        #region Operator Overloads

        public static bool operator ==(cpVect p1, cpVect p2)
        {
            return (p1.Equals(p2));
        }

        public static bool operator !=(cpVect p1, cpVect p2)
        {
            return (!p1.Equals(p2));
        }

        public static cpVect operator -(cpVect p1, cpVect p2)
        {
            cpVect pt;
            pt.x = p1.x - p2.x;
            pt.y = p1.y - p2.y;
            return pt;
        }

        public static cpVect operator -(cpVect p1)
        {
            cpVect pt;
            pt.x = -p1.x;
            pt.y = -p1.y;
            return pt;
        }

        public static cpVect operator +(cpVect p1, cpVect p2)
        {
            cpVect pt;
            pt.x = p1.x + p2.x;
            pt.y = p1.y + p2.y;
            return pt;
        }

        public static cpVect operator +(cpVect p1)
        {
            cpVect pt;
            pt.x = +p1.x;
            pt.y = +p1.y;
            return pt;
        }

        public static cpVect operator *(cpVect p, double value)
        {
            cpVect pt;
            pt.x = p.x * value;
            pt.y = p.y * value;
            return pt;
        }

        #endregion

    }


    public static partial class Physics
    {
        cpVect
        cpvslerp(cpVect v1, cpVect v2, double t)
        {
            double dot = cpVect.Dot(cpVect.Normalize(v1), cpvnormalize(v2));
            double omega = System.Math.Acos(cpfclamp(dot, -1.0f, 1.0f));

            if (omega < 1e-3)
            {
                // If the angle between two vectors is very small, lerp instead to avoid precision issues.
                return cpvlerp(v1, v2, t);
            }
            else
            {
                double denom = 1.0f / System.Math.Sin(omega);
                return cpVect.Add(cpVect.Multiply(v1, System.Math.Sin((1.0f - t) * omega) * denom), cpVect.Multiply(v2, System.Math.Sin(t * omega) * denom));
            }
        }

        cpVect
        cpvslerpconst(cpVect v1, cpVect v2, double a)
        {
            double dot = cpVect.Dot(cpvnormalize(v1), cpvnormalize(v2));
            double omega = System.Math.Acos(cpfclamp(dot, -1.0f, 1.0f));

            return cpvslerp(v1, v2, System.Math.Min(a, omega) / omega);
        }

        string
        cpvstr(cpVect v)
{
            return(string.Format("({0:N3}, {1:N3})", v.x, v.y));
}
    }
}