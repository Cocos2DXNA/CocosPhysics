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

namespace CocosPhysics.Chipmunk
{
public static class Physics {
cpContact
cpContactInit(cpContact con, cpVect p, cpVect n, float dist, cpHashValue hash)
{
	con.p = p;
	con.n = n;
	con.dist = dist;
	
	con.jnAcc = 0.0f;
	con.jtAcc = 0.0f;
	con.jBias = 0.0f;
	
	con.hash = hash;
		
	return con;
}

// TODO make this generic so I can reuse it for constraints also.
static void
unthreadHelper(cpArbiter arb, cpBody body)
{
	struct cpArbiterThread *thread = cpArbiterThreadForBody(arb, body);
	cpArbiter prev = thread.prev;
	cpArbiter next = thread.next;
	
	if(prev){
		cpArbiterThreadForBody(prev, body).next = next;
	} else if(body.arbiterList == arb) {
		// IFF prev is null and body.arbiterList == arb, is arb at the head of the list.
		// This function may be called for an arbiter that was never in a list.
		// In that case, we need to protect it from wiping out the body.arbiterList pointer.
		body.arbiterList = next;
	}
	
	if(next) cpArbiterThreadForBody(next, body).prev = prev;
	
	thread.prev = null;
	thread.next = null;
}

void
cpArbiterUnthread(cpArbiter arb)
{
	unthreadHelper(arb, arb.body_a);
	unthreadHelper(arb, arb.body_b);
}

bool cpArbiterIsFirstContact(cpArbiter arb)
{
	return arb.CP_PRIVATE(state) == cpArbiterStateFirstColl;
}

int cpArbiterGetCount(cpArbiter arb)
{
	// Return 0 contacts if we are in a separate callback.
	return (arb.CP_PRIVATE(state) != cpArbiterStateCached ? arb.CP_PRIVATE(numContacts) : 0);
}

cpVect
cpArbiterGetNormal(cpArbiter arb, int i)
{
	// cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");
	
	cpVect n = arb.contacts[i].n;
	return arb.swappedColl ? cpvneg(n) : n;
}

cpVect
cpArbiterGetPoint(cpArbiter arb, int i)
{
	// cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");
	
	return arb.CP_PRIVATE(contacts)[i].CP_PRIVATE(p);
}

float
cpArbiterGetDepth(cpArbiter arb, int i)
{
	// cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");
	
	return arb.CP_PRIVATE(contacts)[i].CP_PRIVATE(dist);
}

cpContactPointSet
cpArbiterGetContactPointSet(cpArbiter arb)
{
	cpContactPointSet set;
	set.count = cpArbiterGetCount(arb);
	
	for(int i=0; i<set.count; i++){
		set.points[i].point = arb.CP_PRIVATE(contacts)[i].CP_PRIVATE(p);
		set.points[i].normal = arb.CP_PRIVATE(contacts)[i].CP_PRIVATE(n);
		set.points[i].dist = arb.CP_PRIVATE(contacts)[i].CP_PRIVATE(dist);
	}
	
	return set;
}

void
cpArbiterSetContactPointSet(cpArbiter arb, cpContactPointSet *set)
{
	int count = set.count;
	arb.numContacts = count;
	
	for(int i=0; i<count; i++){
		arb.contacts[i].p = set.points[i].point;
		arb.contacts[i].n = set.points[i].normal;
		arb.contacts[i].dist = set.points[i].dist;
	}
}

cpVect
cpArbiterTotalImpulse(cpArbiter arb)
{
	cpContact contacts = arb.contacts;
	cpVect sum = cpvzero;
	
	for(int i=0, count=cpArbiterGetCount(arb); i<count; i++){
		cpContact con = &contacts[i];
		sum = cpvadd(sum, cpvmult(con.n, con.jnAcc));
	}
	
	return (arb.swappedColl ? sum : cpvneg(sum));
}

cpVect
cpArbiterTotalImpulseWithFriction(cpArbiter arb)
{
	cpContact contacts = arb.contacts;
	cpVect sum = cpvzero;
	
	for(int i=0, count=cpArbiterGetCount(arb); i<count; i++){
		cpContact con = &contacts[i];
		sum = cpvadd(sum, cpvrotate(con.n, cpv(con.jnAcc, con.jtAcc)));
	}
		
	return (arb.swappedColl ? sum : cpvneg(sum));
}

float
cpArbiterTotalKE(cpArbiter arb)
{
	float eCoef = (1 - arb.e)/(1 + arb.e);
	float sum = 0.0;
	
	cpContact contacts = arb.contacts;
	for(int i=0, count=cpArbiterGetCount(arb); i<count; i++){
		cpContact con = &contacts[i];
		float jnAcc = con.jnAcc;
		float jtAcc = con.jtAcc;
		
		sum += eCoef*jnAcc*jnAcc/con.nMass + jtAcc*jtAcc/con.tMass;
	}
	
	return sum;
}

// TODO this really shouldn't be a library function probably.
// Should either decide to put it in the API or throw it in a demo.
//float
//cpContactsEstimateCrushingImpulse(cpContact contacts, int numContacts)
//{
//	float fsum = 0.0f;
//	cpVect vsum = cpvzero;
//	
//	for(int i=0; i<numContacts; i++){
//		cpContact con = &contacts[i];
//		cpVect j = cpvrotate(con.n, cpv(con.jnAcc, con.jtAcc));
//		
//		fsum += cpvlength(j);
//		vsum = cpvadd(vsum, j);
//	}
//	
//	float vmag = cpvlength(vsum);
//	return fsum - vmag;
//}

void
cpArbiterIgnore(cpArbiter arb)
{
	arb.state = cpArbiterStateIgnore;
}

cpVect
cpArbiterGetSurfaceVelocity(cpArbiter arb)
{
	return cpvmult(arb.surface_vr, arb.swappedColl ? -1.0f : 1.0);
}

void
cpArbiterSetSurfaceVelocity(cpArbiter arb, cpVect vr)
{
	arb.surface_vr = cpvmult(vr, arb.swappedColl ? -1.0f : 1.0);
}


cpArbiter*
cpArbiterInit(cpArbiter arb, cpShape a, cpShape b)
{
	arb.handler = null;
	arb.swappedColl = false;
	
	arb.e = 0.0f;
	arb.u = 0.0f;
	arb.surface_vr = cpvzero;
	
	arb.numContacts = 0;
	arb.contacts = null;
	
	arb.a = a; arb.body_a = a.body;
	arb.b = b; arb.body_b = b.body;
	
	arb.thread_a.next = null;
	arb.thread_b.next = null;
	arb.thread_a.prev = null;
	arb.thread_b.prev = null;
	
	arb.stamp = 0;
	arb.state = cpArbiterStateFirstColl;
	
	arb.data = null;
	
	return arb;
}

void
cpArbiterUpdate(cpArbiter arb, cpContact contacts, int numContacts, cpCollisionHandler *handler, cpShape a, cpShape b)
{
	// Iterate over the possible pairs to look for hash value matches.
	for(int i=0; i<numContacts; i++){
		cpContact con = &contacts[i];
		
		for(int j=0; j<arb.numContacts; j++){
			cpContact old = &arb.contacts[j];
			
			// This could trigger false positives, but is fairly unlikely nor serious if it does.
			if(con.hash == old.hash){
				// Copy the persistant contact information.
				con.jnAcc = old.jnAcc;
				con.jtAcc = old.jtAcc;
			}
		}
	}
	
	arb.contacts = contacts;
	arb.numContacts = numContacts;
	
	arb.handler = handler;
	arb.swappedColl = (a.collision_type != handler.a);
	
	arb.e = a.e * b.e;
	arb.u = a.u * b.u;
	
	// Currently all contacts will have the same normal.
	// This may change in the future.
	cpVect n = (numContacts ? contacts[0].n : cpvzero);
	cpVect surface_vr = cpvsub(a.surface_v, b.surface_v);
	arb.surface_vr = cpvsub(surface_vr, cpvmult(n, cpvdot(surface_vr, n)));
	
	// For collisions between two similar primitive types, the order could have been swapped.
	arb.a = a; arb.body_a = a.body;
	arb.b = b; arb.body_b = b.body;
	
	// mark it as new if it's been cached
	if(arb.state == cpArbiterStateCached) arb.state = cpArbiterStateFirstColl;
}

void
cpArbiterPreStep(cpArbiter arb, float dt, float slop, float bias)
{
	cpBody a = arb.body_a;
	cpBody b = arb.body_b;
	
	for(int i=0; i<arb.numContacts; i++){
		cpContact con = &arb.contacts[i];
		
		// Calculate the offsets.
		con.r1 = cpvsub(con.p, a.p);
		con.r2 = cpvsub(con.p, b.p);
		
		// Calculate the mass normal and mass tangent.
		con.nMass = 1.0f/k_scalar(a, b, con.r1, con.r2, con.n);
		con.tMass = 1.0f/k_scalar(a, b, con.r1, con.r2, cpvperp(con.n));
				
		// Calculate the target bias velocity.
		con.bias = -bias*cpfmin(0.0f, con.dist + slop)/dt;
		con.jBias = 0.0f;
		
		// Calculate the target bounce velocity.
		con.bounce = normal_relative_velocity(a, b, con.r1, con.r2, con.n)*arb.e;
	}
}

void
cpArbiterApplyCachedImpulse(cpArbiter arb, float dt_coef)
{
	if(cpArbiterIsFirstContact(arb)) return;
	
	cpBody a = arb.body_a;
	cpBody b = arb.body_b;
	
	for(int i=0; i<arb.numContacts; i++){
		cpContact con = &arb.contacts[i];
		cpVect j = cpvrotate(con.n, cpv(con.jnAcc, con.jtAcc));
		apply_impulses(a, b, con.r1, con.r2, cpvmult(j, dt_coef));
	}
}

// TODO is it worth splitting velocity/position correction?

void
cpArbiterApplyImpulse(cpArbiter arb)
{
	cpBody a = arb.body_a;
	cpBody b = arb.body_b;
	cpVect surface_vr = arb.surface_vr;
	float friction = arb.u;

	for(int i=0; i<arb.numContacts; i++){
		cpContact con = arb.contacts[i];
		float nMass = con.nMass;
		cpVect n = con.n;
		cpVect r1 = con.r1;
		cpVect r2 = con.r2;
		
		cpVect vb1 = cpvadd(a.v_bias, cpvmult(cpvperp(r1), a.w_bias));
		cpVect vb2 = cpvadd(b.v_bias, cpvmult(cpvperp(r2), b.w_bias));
		cpVect vr = cpvadd(relative_velocity(a, b, r1, r2), surface_vr);
		
		float vbn = cpvdot(cpvsub(vb2, vb1), n);
		float vrn = cpvdot(vr, n);
		float vrt = cpvdot(vr, cpvperp(n));
		
		float jbn = (con.bias - vbn)*nMass;
		float jbnOld = con.jBias;
		con.jBias = cpfmax(jbnOld + jbn, 0.0f);
		
		float jn = -(con.bounce + vrn)*nMass;
		float jnOld = con.jnAcc;
		con.jnAcc = cpfmax(jnOld + jn, 0.0f);
		
		float jtMax = friction*con.jnAcc;
		float jt = -vrt*con.tMass;
		float jtOld = con.jtAcc;
		con.jtAcc = cpfclamp(jtOld + jt, -jtMax, jtMax);
		
		apply_bias_impulses(a, b, r1, r2, cpvmult(n, con.jBias - jbnOld));
		apply_impulses(a, b, r1, r2, cpvrotate(n, cpv(con.jnAcc - jnOld, con.jtAcc - jtOld)));
	}
}
}
}