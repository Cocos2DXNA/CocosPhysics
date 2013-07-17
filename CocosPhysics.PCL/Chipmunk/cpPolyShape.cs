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
    public static partial class Physics
    {
        static cpBB
        cpPolyShapeTransformVerts(cpPolyShape poly, cpVect p, cpVect rot)
        {
            cpVect[] src = poly.verts;
            cpVect[] dst = poly.tVerts;

            double l = double.PositiveInfinity, r = double.NegativeInfinity;
            double b = double.PositiveInfinity, t = double.NegativeInfinity;

            for (int i = 0; i < poly.numVerts; i++)
            {
                cpVect v = cpVect.Add(p, cpvrotate(src[i], rot));

                dst[i] = v;
                l = System.Math.Min(l, v.x);
                r = System.Math.Max(r, v.x);
                b = System.Math.Min(b, v.y);
                t = System.Math.Max(t, v.y);
            }

            return cpBBNew(l, b, r, t);
        }

        static void
        cpPolyShapeTransformAxes(cpPolyShape poly, cpVect p, cpVect rot)
        {
            cpSplittingPlane src = poly.planes;
            cpSplittingPlane dst = poly.tPlanes;

            for (int i = 0; i < poly.numVerts; i++)
            {
                cpVect n = cpvrotate(src[i].n, rot);
                dst[i].n = n;
                dst[i].d = cpVect.Dot(p, n) + src[i].d;
            }
        }

        static cpBB
        cpPolyShapeCacheData(cpPolyShape poly, cpVect p, cpVect rot)
        {
            cpPolyShapeTransformAxes(poly, p, rot);
            cpBB bb = poly.shape.bb = cpPolyShapeTransformVerts(poly, p, rot);

            return bb;
        }

        static void
        cpPolyShapeDestroy(cpPolyShape poly)
        {
            cpfree(poly.verts);
            cpfree(poly.planes);
        }

        static void
        cpPolyShapeNearestPointQuery(cpPolyShape poly, cpVect p, cpNearestPointQueryInfo* info)
        {
            int count = poly.numVerts;
            cpSplittingPlane planes = poly.tPlanes;
            cpVect[] verts = poly.tVerts;

            cpVect v0 = verts[count - 1];
            double minDist = double.PositiveInfinity;
            cpVect closestPoint = cpvzero;
            bool outside = false;

            for (int i = 0; i < count; i++)
            {
                if (cpSplittingPlaneCompare(planes[i], p) > 0.0f) outside = true;

                cpVect v1 = verts[i];
                cpVect closest = cpClosetPointOnSegment(p, v0, v1);

                double dist = cpVect.Distance(p, closest);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestPoint = closest;
                }

                v0 = v1;
            }

            info.shape = (cpShape)poly;
            info.p = closestPoint; // TODO div/0
            info.d = (outside ? minDist : -minDist);
        }

        static void
        cpPolyShapeSegmentQuery(cpPolyShape poly, cpVect a, cpVect b, cpSegmentQueryInfo* info)
        {
            cpSplittingPlane axes = poly.tPlanes;
            cpVect[] verts = poly.tVerts;
            int numVerts = poly.numVerts;

            for (int i = 0; i < numVerts; i++)
            {
                cpVect n = axes[i].n;
                double an = cpVect.Dot(a, n);
                if (axes[i].d > an) continue;

                double bn = cpVect.Dot(b, n);
                double t = (axes[i].d - an) / (bn - an);
                if (t < 0.0f || 1.0f < t) continue;

                cpVect point = cpvlerp(a, b, t);
                double dt = -cpVect.CrossProduct(n, point);
                double dtMin = -cpVect.CrossProduct(n, verts[i]);
                double dtMax = -cpVect.CrossProduct(n, verts[(i + 1) % numVerts]);

                if (dtMin <= dt && dt <= dtMax)
                {
                    info.shape = (cpShape)poly;
                    info.t = t;
                    info.n = n;
                }
            }
        }

        static cpShapeClass polyClass = new cpShapeClass() {
	CP_POLY_SHAPE,
	(cpShapeCacheDataImpl)cpPolyShapeCacheData,
	(cpShapeDestroyImpl)cpPolyShapeDestroy,
	(cpShapeNearestPointQueryImpl)cpPolyShapeNearestPointQuery,
	(cpShapeSegmentQueryImpl)cpPolyShapeSegmentQuery,
};

        bool
        cpPolyValidate(cpVect[] verts, int numVerts)
        {
            for (int i = 0; i < numVerts; i++)
            {
                cpVect a = verts[i];
                cpVect b = verts[(i + 1) % numVerts];
                cpVect c = verts[(i + 2) % numVerts];

                if (cpVect.CrossProduct(cpVect.Sub(b, a), cpVect.Sub(c, a)) > 0.0f)
                {
                    return false;
                }
            }

            return true;
        }

        int
        cpPolyShapeGetNumVerts(cpShape shape)
        {
            // cpAssertHard(shape.klass == &polyClass, "Shape is not a poly shape.");
            return ((cpPolyShape)shape).numVerts;
        }

        cpVect
        cpPolyShapeGetVert(cpShape shape, int idx)
        {
            // cpAssertHard(shape.klass == &polyClass, "Shape is not a poly shape.");
            // cpAssertHard(0 <= idx && idx < cpPolyShapeGetNumVerts(shape), "Index out of range.");

            return ((cpPolyShape)shape).verts[idx];
        }


        static void
        setUpVerts(cpPolyShape poly, int numVerts, cpVect[] verts, cpVect offset)
        {
            // Fail if the user attempts to pass a concave poly, or a bad winding.
            // cpAssertHard(cpPolyValidate(verts, numVerts), "Polygon is concave or has a reversed winding. Consider using cpConvexHull() or CP_CONVEX_HULL().");

            poly.numVerts = numVerts;
            poly.verts = new cpVect[2 * numVerts];
            poly.planes = new cpSplittingPlane[2 * numVerts];
            poly.tVerts = poly.verts + numVerts;
            poly.tPlanes = poly.planes + numVerts;

            for (int i = 0; i < numVerts; i++)
            {
                cpVect a = cpVect.Add(offset, verts[i]);
                cpVect b = cpVect.Add(offset, verts[(i + 1) % numVerts]);
                cpVect n = cpvnormalize(cpvperp(cpVect.Sub(b, a)));

                poly.verts[i] = a;
                poly.planes[i].n = n;
                poly.planes[i].d = cpVect.Dot(n, a);
            }

        }

        cpPolyShape
        cpPolyShapeInit(cpPolyShape poly, cpBody body, int numVerts, cpVect[] verts, cpVect offset)
        {
            setUpVerts(poly, numVerts, verts, offset);
            cpShapeInit((cpShape)poly, &polyClass, body);

            return poly;
        }

        cpShape
        cpPolyShapeNew(cpBody body, int numVerts, cpVect[] verts, cpVect offset)
        {
            return (cpShape)cpPolyShapeInit(new cpPolyShape(), body, numVerts, verts, offset);
        }

        cpPolyShape
        cpBoxShapeInit(cpPolyShape poly, cpBody body, double width, double height)
        {
            double hw = width / 2.0f;
            double hh = height / 2.0f;

            return cpBoxShapeInit2(poly, body, cpBBNew(-hw, -hh, hw, hh));
        }

        cpPolyShape
        cpBoxShapeInit2(cpPolyShape poly, cpBody body, cpBB box)
        {
            cpVect[] verts = new cpVect[] {
		cpv(box.l, box.b),
		cpv(box.l, box.t),
		cpv(box.r, box.t),
		cpv(box.r, box.b)
	};

            return cpPolyShapeInit(poly, body, 4, verts, cpvzero);
        }

        cpShape
        cpBoxShapeNew(cpBody body, double width, double height)
        {
            return (cpShape)cpBoxShapeInit(new cpPolyShape(), body, width, height);
        }

        cpShape
        cpBoxShapeNew2(cpBody body, cpBB box)
        {
            return (cpShape)cpBoxShapeInit2(new cpPolyShape(), body, box);
        }

        // Unsafe API (chipmunk_unsafe.h)

        void
        cpPolyShapeSetVerts(cpShape shape, int numVerts, cpVect[] verts, cpVect offset)
        {
            // cpAssertHard(shape.klass == &polyClass, "Shape is not a poly shape.");
            cpPolyShapeDestroy((cpPolyShape)shape);
            setUpVerts((cpPolyShape)shape, numVerts, verts, offset);
        }
    }
}