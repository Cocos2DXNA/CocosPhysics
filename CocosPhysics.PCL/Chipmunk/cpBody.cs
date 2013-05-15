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
public static partial class Physics {
// initialized in cpInitChipmunk()
cpBody cpStaticBodySingleton;

cpBody
cpBodyAlloc()
{
	return (cpBody )cpcalloc(1, sizeof(cpBody));
}

cpBody 
cpBodyInit(cpBody body, float m, float i)
{
	body.space = null;
	body.shapeList = null;
	body.arbiterList = null;
	body.constraintList = null;
	
	body.velocity_func = cpBodyUpdateVelocity;
	body.position_func = cpBodyUpdatePosition;
	
	cpComponentNode node = new cpComponentNode() {null, null, 0.0f};
	body.node = node;
	
	body.p = cpvzero;
	body.v = cpvzero;
	body.f = cpvzero;
	
	body.w = 0.0f;
	body.t = 0.0f;
	
	body.v_bias = cpvzero;
	body.w_bias = 0.0f;
	
	body.v_limit = float.PositiveInfinity;
	body.w_limit = float.PositiveInfinity;
	
	body.data = null;
	
	// Setters must be called after full initialization so the sanity checks don't assert on garbage data.
	cpBodySetMass(body, m);
	cpBodySetMoment(body, i);
	cpBodySetAngle(body, 0.0f);
	
	return body;
}

cpBody
cpBodyNew(float m, float i)
{
	return cpBodyInit(cpBodyAlloc(), m, i);
}

cpBody 
cpBodyInitStatic(cpBody body)
{
	cpBodyInit(body, float.PositiveInfinity, float.PositiveInfinity);
	body.node.idleTime = float.PositiveInfinity;
	
	return body;
}

cpBody 
cpBodyNewStatic()
{
	return cpBodyInitStatic(cpBodyAlloc());
}

void cpBodyDestroy(cpBody body){}

void
cpBodyFree(cpBody body)
{
	if(body){
		cpBodyDestroy(body);
		cpfree(body);
	}
}


void
cpBodySetMass(cpBody body, float mass)
{
	// cpAssertHard(mass > 0.0f, "Mass must be positive and non-zero.");
	
	cpBodyActivate(body);
	body.m = mass;
	body.m_inv = 1.0f/mass;
	cpBodyAssertSane(body);
}

void
cpBodySetMoment(cpBody body, float moment)
{
	// cpAssertHard(moment > 0.0f, "Moment of Inertia must be positive and non-zero.");
	
	cpBodyActivate(body);
	body.i = moment;
	body.i_inv = 1.0f/moment;
	cpBodyAssertSane(body);
}

void
cpBodyAddShape(cpBody body, cpShape *shape)
{
	cpShape *next = body.shapeList;
	if(next) next.prev = shape;
	
	shape.next = next;
	body.shapeList = shape;
}

void
cpBodyRemoveShape(cpBody body, cpShape *shape)
{
  cpShape *prev = shape.prev;
  cpShape *next = shape.next;
  
  if(prev){
		prev.next = next;
  } else {
		body.shapeList = next;
  }
  
  if(next){
		next.prev = prev;
	}
  
  shape.prev = null;
  shape.next = null;
}

static cpConstraint 
filterConstraints(cpConstraint node, cpBody body, cpConstraint filter)
{
	if(node == filter){
		return cpConstraintNext(node, body);
	} else if(node.a == body){
		node.next_a = filterConstraints(node.next_a, body, filter);
	} else {
		node.next_b = filterConstraints(node.next_b, body, filter);
	}
	
	return node;
}

void
cpBodyRemoveConstraint(cpBody body, cpConstraint constraint)
{
	body.constraintList = filterConstraints(body.constraintList, body, constraint);
}

void
cpBodySetPos(cpBody body, cpVect pos)
{
	cpBodyActivate(body);
	body.p = pos;
	cpBodyAssertSane(body);
}

static void
setAngle(cpBody body, float angle)
{
	body.a = angle;//fmod(a, (float)System.Math.PI*2.0f);
	body.rot = cpvforangle(angle);
	cpBodyAssertSane(body);
}

void
cpBodySetAngle(cpBody body, float angle)
{
	cpBodyActivate(body);
	setAngle(body, angle);
}

void
cpBodyUpdateVelocity(cpBody body, cpVect gravity, float damping, float dt)
{
	body.v = cpvclamp(cpvadd(cpvmult(body.v, damping), cpvmult(cpvadd(gravity, cpvmult(body.f, body.m_inv)), dt)), body.v_limit);
	
	float w_limit = body.w_limit;
	body.w = cpfclamp(body.w*damping + body.t*body.i_inv*dt, -w_limit, w_limit);
	
	cpBodySanityCheck(body);
}

void
cpBodyUpdatePosition(cpBody body, float dt)
{
	body.p = cpvadd(body.p, cpvmult(cpvadd(body.v, body.v_bias), dt));
	setAngle(body, body.a + (body.w + body.w_bias)*dt);
	
	body.v_bias = cpvzero;
	body.w_bias = 0.0f;
	
	cpBodySanityCheck(body);
}

void
cpBodyResetForces(cpBody body)
{
	cpBodyActivate(body);
	body.f = cpvzero;
	body.t = 0.0f;
}

void
cpBodyApplyForce(cpBody body, cpVect force, cpVect r)
{
	cpBodyActivate(body);
	body.f = cpvadd(body.f, force);
	body.t += cpvcross(r, force);
}

void
cpBodyApplyImpulse(cpBody body, cpVect j, cpVect r)
{
	cpBodyActivate(body);
	apply_impulse(body, j, r);
}

static cpVect
cpBodyGetVelAtPoint(cpBody body, cpVect r)
{
	return cpvadd(body.v, cpvmult(cpvperp(r), body.w));
}

cpVect
cpBodyGetVelAtWorldPoint(cpBody body, cpVect point)
{
	return cpBodyGetVelAtPoint(body, cpvsub(point, body.p));
}

cpVect
cpBodyGetVelAtLocalPoint(cpBody body, cpVect point)
{
	return cpBodyGetVelAtPoint(body, cpvrotate(point, body.rot));
}

void
cpBodyEachShape(cpBody body, cpBodyShapeIteratorFunc func, object data)
{
	cpShape *shape = body.shapeList;
	while(shape){
		cpShape *next = shape.next;
		func(body, shape, data);
		shape = next;
	}
}

void
cpBodyEachConstraint(cpBody body, cpBodyConstraintIteratorFunc func, object data)
{
	cpConstraint constraint = body.constraintList;
	while(constraint){
		cpConstraint next = cpConstraintNext(constraint, body);
		func(body, constraint, data);
		constraint = next;
	}
}

void
cpBodyEachArbiter(cpBody body, cpBodyArbiterIteratorFunc func, object data)
{
	cpArbiter arb = body.arbiterList;
	while(arb){
		cpArbiter next = cpArbiterNext(arb, body);
		
		arb.swappedColl = (body == arb.body_b);
		func(body, arb, data);
		
		arb = next;
	}
}
}
}