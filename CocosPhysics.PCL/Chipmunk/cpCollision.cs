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
 
// // typedef int (*collisionFunc)(cpShape , cpShape , cpContact );
using System;
namespace CocosPhysics.Chipmunk
{
public static partial class Physics {
// Add contact points for circle to circle collisions.
// Used by several collision tests.
static int
circle2circleQuery(cpVect p1, cpVect p2, double r1, double r2, cpContact con)
{
	double mindist = r1 + r2;
	cpVect delta = cpVect.Sub(p2, p1);
	double distsq = cpVect.LengthSQ(delta);
	if(distsq >= mindist*mindist) return 0;
	
	double dist = System.Math.Sqrt(distsq);

	// Allocate and initialize the contact.
	cpContactInit(
		con,
		cpVect.Add(p1, cpVect.Multiply(delta, 0.5f + (r1 - 0.5f*mindist)/(dist ? dist : double.PositiveInfinity))),
		(dist ? cpVect.Multiply(delta, 1.0f/dist) : cpv(1.0f, 0.0f)),
		dist - mindist,
		0
	);
	
	return 1;
}

// Collide circle shapes.
static int
circle2circle(cpShape shape1, cpShape shape2, cpContact arr)
{
	cpCircleShape circ1 = (cpCircleShape )shape1; //TODO
	cpCircleShape circ2 = (cpCircleShape )shape2;
	
	return circle2circleQuery(circ1.tc, circ2.tc, circ1.r, circ2.r, arr);
}

static int
circle2segment(cpCircleShape circleShape, cpSegmentShape segmentShape, cpContact con)
{
	cpVect seg_a = segmentShape.ta;
	cpVect seg_b = segmentShape.tb;
	cpVect center = circleShape.tc;
	
	cpVect seg_delta = cpVect.Sub(seg_b, seg_a);
	double closest_t = cpfclamp01(cpVect.Dot(seg_delta, cpVect.Sub(center, seg_a))/cpVect.LengthSQ(seg_delta));
	cpVect closest = cpVect.Add(seg_a, cpVect.Multiply(seg_delta, closest_t));
	
	if(circle2circleQuery(center, closest, circleShape.r, segmentShape.r, con)){
		cpVect n = con[0].n;
		
		// Reject endcap collisions if tangents are provided.
		if(
			(closest_t == 0.0f && cpVect.Dot(n, segmentShape.a_tangent) < 0.0) ||
			(closest_t == 1.0f && cpVect.Dot(n, segmentShape.b_tangent) < 0.0)
		) return 0;
		
		return 1;
	} else {
		return 0;
	}
}

// Helper function for working with contact buffers
// This used to malloc/realloc memory on the fly but was repurposed.
static cpContact 
nextContactPoint(cpContact arr, int *numPtr)
{
	int index = *numPtr;
	
	if(index < CP_MAX_CONTACTS_PER_ARBITER){
		(*numPtr) = index + 1;
		return &arr[index];
	} else {
		return &arr[CP_MAX_CONTACTS_PER_ARBITER - 1];
	}
}

// Find the minimum separating axis for the give poly and axis list.
static int
findMSA(cpPolyShape poly, cpSplittingPlane planes, int num, double min_out)
{
	int min_index = 0;
	double min = cpPolyShapeValueOnAxis(poly, planes.n, planes.d);
	if(min > 0.0f) return -1;
	
	for(int i=1; i<num; i++){
		double dist = cpPolyShapeValueOnAxis(poly, planes[i].n, planes[i].d);
		if(dist > 0.0f) {
			return -1;
		} else if(dist > min){
			min = dist;
			min_index = i;
		}
	}
	
	(*min_out) = min;
	return min_index;
}

// Add contacts for probably penetrating vertexes.
// This handles the degenerate case where an overlap was detected, but no vertexes fall inside
// the opposing polygon. (like a star of david)
static int
findVertsFallback(cpContact arr, cpPolyShape poly1, cpPolyShape poly2, cpVect n, double dist)
{
	int num = 0;
	
	for(int i=0; i<poly1.numVerts; i++){
		cpVect v = poly1.tVerts[i];
		if(cpPolyShapeContainsVertPartial(poly2, v, cpvneg(n)))
			cpContactInit(nextContactPoint(arr, &num), v, n, dist, CP_HASH_PAIR(poly1.shape.hashid, i));
	}
	
	for(int i=0; i<poly2.numVerts; i++){
		cpVect v = poly2.tVerts[i];
		if(cpPolyShapeContainsVertPartial(poly1, v, n))
			cpContactInit(nextContactPoint(arr, &num), v, n, dist, CP_HASH_PAIR(poly2.shape.hashid, i));
	}
	
	return num;
}

// Add contacts for penetrating vertexes.
static int
findVerts(cpContact arr, cpPolyShape poly1, cpPolyShape poly2, cpVect n, double dist)
{
	int num = 0;
	
	for(int i=0; i<poly1.numVerts; i++){
		cpVect v = poly1.tVerts[i];
		if(cpPolyShapeContainsVert(poly2, v))
			cpContactInit(nextContactPoint(arr, &num), v, n, dist, CP_HASH_PAIR(poly1.shape.hashid, i));
	}
	
	for(int i=0; i<poly2.numVerts; i++){
		cpVect v = poly2.tVerts[i];
		if(cpPolyShapeContainsVert(poly1, v))
			cpContactInit(nextContactPoint(arr, &num), v, n, dist, CP_HASH_PAIR(poly2.shape.hashid, i));
	}
	
	return (num ? num : findVertsFallback(arr, poly1, poly2, n, dist));
}

// Collide poly shapes together.
static int
poly2poly(cpShape shape1, cpShape shape2, cpContact arr)
{
	cpPolyShape poly1 = (cpPolyShape )shape1;
	cpPolyShape poly2 = (cpPolyShape )shape2;
	
	double min1;
	int mini1 = findMSA(poly2, poly1.tPlanes, poly1.numVerts, &min1);
	if(mini1 == -1) return 0;
	
	double min2;
	int mini2 = findMSA(poly1, poly2.tPlanes, poly2.numVerts, &min2);
	if(mini2 == -1) return 0;
	
	// There is overlap, find the penetrating verts
	if(min1 > min2)
		return findVerts(arr, poly1, poly2, poly1.tPlanes[mini1].n, min1);
	else
		return findVerts(arr, poly1, poly2, cpvneg(poly2.tPlanes[mini2].n), min2);
}

// Like cpPolyValueOnAxis(), but for segments.
static double
segValueOnAxis(cpSegmentShape seg, cpVect n, double d)
{
	double a = cpVect.Dot(n, seg.ta) - seg.r;
	double b = cpVect.Dot(n, seg.tb) - seg.r;
	return System.Math.Min(a, b) - d;
}

// Identify vertexes that have penetrated the segment.
static void
findPointsBehindSeg(cpContact arr, int *num, cpSegmentShape seg, cpPolyShape poly, double pDist, double coef) 
{
	double dta = cpVect.CrossProduct(seg.tn, seg.ta);
	double dtb = cpVect.CrossProduct(seg.tn, seg.tb);
	cpVect n = cpVect.Multiply(seg.tn, coef);
	
	for(int i=0; i<poly.numVerts; i++){
		cpVect v = poly.tVerts[i];
		if(cpVect.Dot(v, n) < cpVect.Dot(seg.tn, seg.ta)*coef + seg.r){
			double dt = cpVect.CrossProduct(seg.tn, v);
			if(dta >= dt && dt >= dtb){
				cpContactInit(nextContactPoint(arr, num), v, n, pDist, CP_HASH_PAIR(poly.shape.hashid, i));
			}
		}
	}
}

// This one is complicated and gross. Just don't go there...
// TODO: Comment me!
static int
seg2poly(cpShape shape1, cpShape shape2, cpContact arr)
{
	cpSegmentShape seg = (cpSegmentShape )shape1;
	cpPolyShape poly = (cpPolyShape )shape2;
	cpSplittingPlane planes = poly.tPlanes;
	
	double segD = cpVect.Dot(seg.tn, seg.ta);
	double minNorm = cpPolyShapeValueOnAxis(poly, seg.tn, segD) - seg.r;
	double minNeg = cpPolyShapeValueOnAxis(poly, cpvneg(seg.tn), -segD) - seg.r;
	if(minNeg > 0.0f || minNorm > 0.0f) return 0;
	
	int mini = 0;
	double poly_min = segValueOnAxis(seg, planes.n, planes.d);
	if(poly_min > 0.0f) return 0;
	for(int i=0; i<poly.numVerts; i++){
		double dist = segValueOnAxis(seg, planes[i].n, planes[i].d);
		if(dist > 0.0f){
			return 0;
		} else if(dist > poly_min){
			poly_min = dist;
			mini = i;
		}
	}
	
	int num = 0;
	
	cpVect poly_n = cpvneg(planes[mini].n);
	
	cpVect va = cpVect.Add(seg.ta, cpVect.Multiply(poly_n, seg.r));
	cpVect vb = cpVect.Add(seg.tb, cpVect.Multiply(poly_n, seg.r));
	if(cpPolyShapeContainsVert(poly, va))
		cpContactInit(nextContactPoint(arr, &num), va, poly_n, poly_min, CP_HASH_PAIR(seg.shape.hashid, 0));
	if(cpPolyShapeContainsVert(poly, vb))
		cpContactInit(nextContactPoint(arr, &num), vb, poly_n, poly_min, CP_HASH_PAIR(seg.shape.hashid, 1));
	
	// doubleing point precision problems here.
	// This will have to do for now.
//	poly_min -= cp_collision_slop; // TODO is this needed anymore?
	
	if(minNorm >= poly_min || minNeg >= poly_min) {
		if(minNorm > minNeg)
			findPointsBehindSeg(arr, &num, seg, poly, minNorm, 1.0f);
		else
			findPointsBehindSeg(arr, &num, seg, poly, minNeg, -1.0f);
	}
	
	// If no other collision points are found, try colliding endpoints.
	if(num == 0){
		cpVect poly_a = poly.tVerts[mini];
		cpVect poly_b = poly.tVerts[(mini + 1)%poly.numVerts];
		
		if(circle2circleQuery(seg.ta, poly_a, seg.r, 0.0f, arr)) return 1;
		if(circle2circleQuery(seg.tb, poly_a, seg.r, 0.0f, arr)) return 1;
		if(circle2circleQuery(seg.ta, poly_b, seg.r, 0.0f, arr)) return 1;
		if(circle2circleQuery(seg.tb, poly_b, seg.r, 0.0f, arr)) return 1;
	}

	return num;
}

// This one is less gross, but still gross.
// TODO: Comment me!
static int
circle2poly(cpShape shape1, cpShape shape2, cpContact con)
{
	cpCircleShape circ = (cpCircleShape )shape1;
	cpPolyShape poly = (cpPolyShape )shape2;
	cpSplittingPlane planes = poly.tPlanes;
	
	int mini = 0;
	double min = cpSplittingPlaneCompare(planes[0], circ.tc) - circ.r;
	for(int i=0; i<poly.numVerts; i++){
		double dist = cpSplittingPlaneCompare(planes[i], circ.tc) - circ.r;
		if(dist > 0.0f){
			return 0;
		} else if(dist > min) {
			min = dist;
			mini = i;
		}
	}
	
	cpVect n = planes[mini].n;
	cpVect a = poly.tVerts[mini];
	cpVect b = poly.tVerts[(mini + 1)%poly.numVerts];
	double dta = cpVect.CrossProduct(n, a);
	double dtb = cpVect.CrossProduct(n, b);
	double dt = cpVect.CrossProduct(n, circ.tc);
		
	if(dt < dtb){
		return circle2circleQuery(circ.tc, b, circ.r, 0.0f, con);
	} else if(dt < dta) {
		cpContactInit(
			con,
			cpVect.Sub(circ.tc, cpVect.Multiply(n, circ.r + min/2.0f)),
			cpvneg(n),
			min,
			0				 
		);
	
		return 1;
	} else {
		return circle2circleQuery(circ.tc, a, circ.r, 0.0f, con);
	}
}

// Submitted by LegoCyclon
static int
seg2seg(cpShape shape1, cpShape shape2, cpContact* con)
{
	cpSegmentShape* seg1 = (cpSegmentShape )shape1;
	cpSegmentShape* seg2 = (cpSegmentShape )shape2;
	
	cpVect v1 = cpVect.Sub(seg1.tb, seg1.ta);
	cpVect v2 = cpVect.Sub(seg2.tb, seg2.ta);
	double v1lsq = cpVect.LengthSQ(v1);
	double v2lsq = cpVect.LengthSQ(v2);
	// project seg2 onto seg1
	cpVect p1a = cpvproject(cpVect.Sub(seg2.ta, seg1.ta), v1);
	cpVect p1b = cpvproject(cpVect.Sub(seg2.tb, seg1.ta), v1);
	// project seg1 onto seg2
	cpVect p2a = cpvproject(cpVect.Sub(seg1.ta, seg2.ta), v2);
	cpVect p2b = cpvproject(cpVect.Sub(seg1.tb, seg2.ta), v2);
	
	// clamp projections to segment endcaps
	if (cpVect.Dot(p1a, v1) < 0.0f)
	 p1a = cpvzero;
	else if (cpVect.Dot(p1a, v1) > 0.0f && cpVect.LengthSQ(p1a) > v1lsq)
	 p1a = v1;
	if (cpVect.Dot(p1b, v1) < 0.0f)
	 p1b = cpvzero;
	else if (cpVect.Dot(p1b, v1) > 0.0f && cpVect.LengthSQ(p1b) > v1lsq)
	 p1b = v1;
	if (cpVect.Dot(p2a, v2) < 0.0f)
	 p2a = cpvzero;
	else if (cpVect.Dot(p2a, v2) > 0.0f && cpVect.LengthSQ(p2a) > v2lsq)
	 p2a = v2;
	if (cpVect.Dot(p2b, v2) < 0.0f)
	 p2b = cpvzero;
	else if (cpVect.Dot(p2b, v2) > 0.0f && cpVect.LengthSQ(p2b) > v2lsq)
	 p2b = v2;
	
	p1a = cpVect.Add(p1a, seg1.ta);
	p1b = cpVect.Add(p1b, seg1.ta);
	p2a = cpVect.Add(p2a, seg2.ta);
	p2b = cpVect.Add(p2b, seg2.ta);
	
	int num = 0;
	
	if (!circle2circleQuery(p1a, p2a, seg1.r, seg2.r, nextContactPoint(con, &num)))
	 --num;
	
	if (!circle2circleQuery(p1b, p2b, seg1.r, seg2.r, nextContactPoint(con, &num)))
	 --num;
	
	if (!circle2circleQuery(p1a, p2b, seg1.r, seg2.r, nextContactPoint(con, &num)))
	 --num;
	
	if (!circle2circleQuery(p1b, p2a, seg1.r, seg2.r, nextContactPoint(con, &num)))
	 --num;
	
	return num;
}

static collisionFunc builtinCollisionFuncs[9] = {
	circle2circle,
	null,
	null,
	(collisionFunc)circle2segment,
	null,
	null,
	circle2poly,
	seg2poly,
	poly2poly,
};
static collisionFunc *colfuncs = builtinCollisionFuncs;

static collisionFunc segmentCollisions[9] = {
	circle2circle,
	null,
	null,
	(collisionFunc)circle2segment,
	seg2seg,
	null,
	circle2poly,
	seg2poly,
	poly2poly,
};

void
cpEnableSegmentToSegmentCollisions()
{
	colfuncs = segmentCollisions;
}

int
cpCollideShapes(cpShape a, cpShape b, cpContact arr)
{
	// Their shape types must be in order.
	// cpAssertSoft(a.klass.type <= b.klass.type, "Collision shapes passed to cpCollideShapes() are not sorted.");
	
	collisionFunc cfunc = colfuncs[a.klass.type + b.klass.type*CP_NUM_SHAPES];
	return (cfunc) ? cfunc(a, b, arr) : 0;
}
}
}