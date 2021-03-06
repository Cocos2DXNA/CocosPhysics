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
    public delegate void cpSpatialIndexQueryFunc(cpShape s, object d);
    public delegate void cpSpaceNearestPointQueryFunc(cpShape s, ref cpShape outShape);

struct BBQueryContext {
	cpBB bb;
	cpLayers layers;
	cpGroup group;
	cpSpaceBBQueryFunc func;
};

struct PointQueryContext {
	cpVect point;
	cpLayers layers;
	cpGroup group;
    cpSpatialIndexQueryFunc func;
	object data;
};

    public static partial class Physics
    {


static void 
PointQuery(ref PointQueryContext context, cpShape shape, object data)
{
	if(
		!(shape.group && context.group == shape.group) && (context.layers & shape.layers) &&
		cpShapePointQuery(shape, context.point)
	){
		context.func(shape, context.data);
	}
}

void
cpSpacePointQuery(cpSpace space, cpVect point, cpLayers layers, cpGroup group, cpSpacePointQueryFunc func, object data)
{
	struct PointQueryContext context = {point, layers, group, func, data};
	cpBB bb = cpBBNewForCircle(point, 0.0f);
	
	cpSpaceLock(space); {
    cpSpatialIndexQuery(space.activeShapes, ref context, bb, new cpSpatialIndexQueryFunc(PointQuery), data);
    cpSpatialIndexQuery(space.staticShapes, ref context, bb, new cpSpatialIndexQueryFunc(PointQuery), data);
	} cpSpaceUnlock(space, true);
}

static void
PointQueryFirst(cpShape shape, ref cpShape outShape)
{
	if(!shape.sensor) 
        outShape = shape;
}

cpShape 
cpSpacePointQueryFirst(cpSpace space, cpVect point, cpLayers layers, cpGroup group)
{
	cpShape shape = null;
	cpSpacePointQuery(space, point, layers, group, new cpSpacePointQueryFunc(PointQueryFirst), ref shape);
	
	return shape;
}

//MARK: Nearest Point Query Functions

struct NearestPointQueryContext {
	cpVect point;
	double maxDistance;
	cpLayers layers;
	cpGroup group;
	cpSpaceNearestPointQueryFunc func;
};

static void 
NearestPointQuery(struct NearestPointQueryContext *context, cpShape shape, object data)
{
	if(
		!(shape.group && context.group == shape.group) && (context.layers&shape.layers)
	){
		cpNearestPointQueryInfo info;
		cpShapeNearestPointQuery(shape, context.point, ref info);
		
		if(info.shape && info.d < context.maxDistance) context.func(shape, info.d, info.p, data);
	}
}

void
cpSpaceNearestPointQuery(cpSpace space, cpVect point, double maxDistance, cpLayers layers, cpGroup group, cpSpaceNearestPointQueryFunc func, object data)
{
	struct NearestPointQueryContext context = {point, maxDistance, layers, group, func};
	cpBB bb = cpBBNewForCircle(point, System.Math.Max(maxDistance, 0.0f));
	
	cpSpaceLock(space); {
		cpSpatialIndexQuery(space.activeShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQuery, data);
		cpSpatialIndexQuery(space.staticShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQuery, data);
	} cpSpaceUnlock(space, true);
}

static void
NearestPointQueryNearest(struct NearestPointQueryContext *context, cpShape shape, cpNearestPointQueryInfo out)
{
	if(
		!(shape.group && context.group == shape.group) && (context.layers&shape.layers) && !shape.sensor
	){
		cpNearestPointQueryInfo info;
		cpShapeNearestPointQuery(shape, context.point, ref info);
		
		if(info.d < out.d) (*out) = info;
	}
}

cpShape 
cpSpaceNearestPointQueryNearest(cpSpace space, cpVect point, double maxDistance, cpLayers layers, cpGroup group, cpNearestPointQueryInfo out)
{
	cpNearestPointQueryInfo info = {null, cpvzero, maxDistance};
	if(out){
		(*out) = info;
  } else {
		out = ref info;
	}
	
	struct NearestPointQueryContext context = {
		point, maxDistance,
		layers, group,
		null
	};
	
	cpBB bb = cpBBNewForCircle(point, System.Math.Max(maxDistance, 0.0f));
	cpSpatialIndexQuery(space.activeShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQueryNearest, out);
	cpSpatialIndexQuery(space.staticShapes, &context, bb, (cpSpatialIndexQueryFunc)NearestPointQueryNearest, out);
	
	return out.shape;
}


//MARK: Segment Query Functions

struct SegmentQueryContext {
	cpVect start, end;
	cpLayers layers;
	cpGroup group;
	cpSpaceSegmentQueryFunc func;
};

static double
SegmentQuery(ref SegmentQueryContext context, cpShape shape, object data)
{
	cpSegmentQueryInfo info;
	
	if(
		!(shape.group && context.group == shape.group) && (context.layers&shape.layers) &&
		cpShapeSegmentQuery(shape, context.start, context.end, ref info)
	){
		context.func(shape, info.t, info.n, data);
	}
	
	return 1.0f;
}

void
cpSpaceSegmentQuery(cpSpace space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSpaceSegmentQueryFunc func, object data)
{
	struct SegmentQueryContext context = {
		start, end,
		layers, group,
		func,
	};
	
    cpSpatialIndexSegmentQuery(space.staticShapes, ref context, start, end, 1.0f, new cpSpatialIndexSegmentQueryFunc(SegmentQuery), data);
    cpSpatialIndexSegmentQuery(space.activeShapes, ref context, start, end, 1.0f, new cpSpatialIndexSegmentQueryFunc(SegmentQuery), data);
}

static double
SegmentQueryFirst(struct SegmentQueryContext *context, cpShape shape, ref cpSegmentQueryInfo queryOut)
{
	cpSegmentQueryInfo info;
	
	if(
		!(shape.group && context.group == shape.group) && (context.layers&shape.layers) &&
		!shape.sensor &&
		cpShapeSegmentQuery(shape, context.start, context.end, ref info) &&
		info.t < out.t
	){
		queryOut = info;
	}
	
	return out.t;
}

cpShape 
cpSpaceSegmentQueryFirst(cpSpace space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSegmentQueryInfo out)
{
	cpSegmentQueryInfo info = {null, 1.0f, cpvzero};
	if(out){
		(*out) = info;
  } else {
		out = ref info;
	}
	
	struct SegmentQueryContext context = {
		start, end,
		layers, group,
		null
	};
	
	cpSpatialIndexSegmentQuery(space.staticShapes, &context, start, end, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQueryFirst, out);
	cpSpatialIndexSegmentQuery(space.activeShapes, &context, start, end, out.t, (cpSpatialIndexSegmentQueryFunc)SegmentQueryFirst, out);
	
	return out.shape;
}

//MARK: BB Query Functions


static void 
BBQuery(struct BBQueryContext *context, cpShape shape, object data)
{
	if(
		!(shape.group && context.group == shape.group) && (context.layers&shape.layers) &&
		cpBBIntersects(context.bb, shape.bb)
	){
		context.func(shape, data);
	}
}

void
cpSpaceBBQuery(cpSpace space, cpBB bb, cpLayers layers, cpGroup group, cpSpaceBBQueryFunc func, object data)
{
	struct BBQueryContext context = {bb, layers, group, func};
	
	cpSpaceLock(space); {
    cpSpatialIndexQuery(space.activeShapes, &context, bb, (cpSpatialIndexQueryFunc)BBQuery, data);
    cpSpatialIndexQuery(space.staticShapes, &context, bb, (cpSpatialIndexQueryFunc)BBQuery, data);
	} cpSpaceUnlock(space, true);
}

//MARK: Shape Query Functions

struct ShapeQueryContext {
	cpSpaceShapeQueryFunc func;
	object data;
	bool anyCollision;
};

// Callback from the spatial hash.
static void
ShapeQuery(cpShape a, cpShape b, struct ShapeQueryContext *context)
{
	// Reject any of the simple cases
	if(
		(a.group && a.group == b.group) ||
		!(a.layers & b.layers) ||
		a == b
	) return;
	
	cpContact contacts[CP_MAX_CONTACTS_PER_ARBITER];
	int numContacts = 0;
	
	// Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
	if(a.klass.type <= b.klass.type){
		numContacts = cpCollideShapes(a, b, contacts);
	} else {
		numContacts = cpCollideShapes(b, a, contacts);
		for(int i=0; i<numContacts; i++) contacts[i].n = cpvneg(contacts[i].n);
	}
	
	if(numContacts){
		context.anyCollision = !(a.sensor || b.sensor);
		
		if(context.func){
			cpContactPointSet set;
			set.count = numContacts;
			
			for(int i=0; i<set.count; i++){
				set.points[i].point = contacts[i].p;
				set.points[i].normal = contacts[i].n;
				set.points[i].dist = contacts[i].dist;
			}
			
			context.func(b, &set, context.data);
		}
	}
}

bool
cpSpaceShapeQuery(cpSpace space, cpShape shape, cpSpaceShapeQueryFunc func, object data)
{
	cpBody body = shape.body;
	cpBB bb = (body ? cpShapeUpdate(shape, body.p, body.rot) : shape.bb);
	struct ShapeQueryContext context = {func, data, false};
	
    cpSpatialIndexQuery(space.activeShapes, shape, bb, new cpSpatialIndexQueryFunc(ShapeQuery), ref context);
    cpSpatialIndexQuery(space.staticShapes, shape, bb, new cpSpatialIndexQueryFunc(ShapeQuery), ref context);
	
	return context.anyCollision;
}
}
}+

