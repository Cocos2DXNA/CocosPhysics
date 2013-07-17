/* Copyright (c) 2009 Scott Lembcke
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

static cpSpatialIndexClass Klass();

// typedef struct Node Node;
// typedef struct Pair Pair;

struct cpBBTree {
	cpSpatialIndex spatialIndex;
	cpBBTreeVelocityFunc velocityFunc;
	
	cpHashSet *leaves;
	Node root;
	
	Node pooledNodes;
	Pair *pooledPairs;
	cpArray *allocatedBuffers;
	
	cpTimestamp stamp;
};

struct Node {
	object obj;
	cpBB bb;
	Node parent;
	
	union {
		// Internal nodes
		struct { Node a, *b; } children;
		
		// Leaves
		struct {
			cpTimestamp stamp;
			Pair *pairs;
		} leaf;
	} node;
};

// Can't use anonymous unions and still get good x-compiler compatability
#define A node.children.a
#define B node.children.b
#define STAMP node.leaf.stamp
#define PAIRS node.leaf.pairs

// typedef struct Thread {
	Pair *prev;
	Node leaf;
	Pair *next;
} Thread;

struct Pair { Thread a, b; };

//MARK: Misc Functions

static cpBB
GetBB(cpBBTree tree, object obj)
{
	cpBB bb = tree.spatialIndex.bbfunc(obj);
	
	cpBBTreeVelocityFunc velocityFunc = tree.velocityFunc;
	if(velocityFunc){
		double coef = 0.1f;
		double x = (bb.r - bb.l)*coef;
		double y = (bb.t - bb.b)*coef;
		
		cpVect v = cpVect.Multiply(velocityFunc(obj), 0.1f);
		return cpBBNew(bb.l + System.Math.Min(-x, v.x), bb.b + System.Math.Min(-y, v.y), bb.r + System.Math.Max(x, v.x), bb.t + System.Math.Max(y, v.y));
	} else {
		return bb;
	}
}

static cpBBTree 
GetTree(cpSpatialIndex *index)
{
	return (index && index.klass == Klass() ? (cpBBTree )index : null);
}

static Node 
GetRootIfTree(cpSpatialIndex *index){
	return (index && index.klass == Klass() ? ((cpBBTree )index).root : null);
}

static cpBBTree 
GetMasterTree(cpBBTree tree)
{
	cpBBTree dynamicTree = GetTree(tree.spatialIndex.dynamicIndex);
	return (dynamicTree ? dynamicTree : tree);
}

static void
IncrementStamp(cpBBTree tree)
{
	cpBBTree dynamicTree = GetTree(tree.spatialIndex.dynamicIndex);
	if(dynamicTree){
		dynamicTree.stamp++;
	} else {
		tree.stamp++;
	}
}

//MARK: Pair/Thread Functions

static void
PairRecycle(cpBBTree tree, Pair *pair)
{
	// Share the pool of the master tree.
	// TODO would be lovely to move the pairs stuff into an external data structure.
	tree = GetMasterTree(tree);
	
	pair.a.next = tree.pooledPairs;
	tree.pooledPairs = pair;
}

static Pair *
PairFromPool(cpBBTree tree)
{
	// Share the pool of the master tree.
	// TODO would be lovely to move the pairs stuff into an external data structure.
	tree = GetMasterTree(tree);
	
	Pair *pair = tree.pooledPairs;
	
	if(pair){
		tree.pooledPairs = pair.a.next;
		return pair;
	} else {
		// Pool is exhausted, make more
		int count = CP_BUFFER_BYTES/sizeof(Pair);
		// cpAssertHard(count, "Internal Error: Buffer size is too small.");
		
		Pair *buffer = (Pair *)cpcalloc(1, CP_BUFFER_BYTES);
		cpArrayPush(tree.allocatedBuffers, buffer);
		
		// push all but the first one, return the first instead
		for(int i=1; i<count; i++) PairRecycle(tree, buffer + i);
		return buffer;
	}
}

static void
ThreadUnlink(Thread thread)
{
	Pair *next = thread.next;
	Pair *prev = thread.prev;
	
	if(next){
		if(next.a.leaf == thread.leaf) next.a.prev = prev; else next.b.prev = prev;
	}
	
	if(prev){
		if(prev.a.leaf == thread.leaf) prev.a.next = next; else prev.b.next = next;
	} else {
		thread.leaf.PAIRS = next;
	}
}

static void
PairsClear(Node leaf, cpBBTree tree)
{
	Pair *pair = leaf.PAIRS;
	leaf.PAIRS = null;
	
	while(pair){
		if(pair.a.leaf == leaf){
			Pair *next = pair.a.next;
			ThreadUnlink(pair.b);
			PairRecycle(tree, pair);
			pair = next;
		} else {
			Pair *next = pair.b.next;
			ThreadUnlink(pair.a);
			PairRecycle(tree, pair);
			pair = next;
		}
	}
}

static void
PairInsert(Node a, Node b, cpBBTree tree)
{
	Pair *nextA = a.PAIRS, *nextB = b.PAIRS;
	Pair *pair = PairFromPool(tree);
	Pair temp = {{null, a, nextA},{null, b, nextB}};
	
	a.PAIRS = b.PAIRS = pair;
	*pair = temp;
	
	if(nextA){
		if(nextA.a.leaf == a) nextA.a.prev = pair; else nextA.b.prev = pair;
	}
	
	if(nextB){
		if(nextB.a.leaf == b) nextB.a.prev = pair; else nextB.b.prev = pair;
	}
}


//MARK: Node Functions

static void
NodeRecycle(cpBBTree tree, Node node)
{
	node.parent = tree.pooledNodes;
	tree.pooledNodes = node;
}

static Node 
NodeFromPool(cpBBTree tree)
{
	Node node = tree.pooledNodes;
	
	if(node){
		tree.pooledNodes = node.parent;
		return node;
	} else {
		// Pool is exhausted, make more
		int count = CP_BUFFER_BYTES/sizeof(Node);
		// cpAssertHard(count, "Internal Error: Buffer size is too small.");
		
		Node buffer = (Node )cpcalloc(1, CP_BUFFER_BYTES);
		cpArrayPush(tree.allocatedBuffers, buffer);
		
		// push all but the first one, return the first instead
		for(int i=1; i<count; i++) NodeRecycle(tree, buffer + i);
		return buffer;
	}
}

static void
NodeSetA(Node node, Node value)
{
	node.A = value;
	value.parent = node;
}

static void
NodeSetB(Node node, Node value)
{
	node.B = value;
	value.parent = node;
}

static Node 
NodeNew(cpBBTree tree, Node a, Node b)
{
	Node node = NodeFromPool(tree);
	
	node.obj = null;
	node.bb = cpBBMerge(a.bb, b.bb);
	node.parent = null;
	
	NodeSetA(node, a);
	NodeSetB(node, b);
	
	return node;
}

static bool
NodeIsLeaf(Node node)
{
	return (node.obj != null);
}

static Node 
NodeOther(Node node, Node child)
{
	return (node.A == child ? node.B : node.A);
}

static void
NodeReplaceChild(Node parent, Node child, Node value, cpBBTree tree)
{
	// cpAssertSoft(!NodeIsLeaf(parent), "Internal Error: Cannot replace child of a leaf.");
	// cpAssertSoft(child == parent.A || child == parent.B, "Internal Error: Node is not a child of parent.");
	
	if(parent.A == child){
		NodeRecycle(tree, parent.A);
		NodeSetA(parent, value);
	} else {
		NodeRecycle(tree, parent.B);
		NodeSetB(parent, value);
	}
	
	for(Node node=parent; node; node = node.parent){
		node.bb = cpBBMerge(node.A.bb, node.B.bb);
	}
}

//MARK: Subtree Functions

static double
cpBBProximity(cpBB a, cpBB b)
{
	return System.Math.Abs(a.l + a.r - b.l - b.r) + System.Math.Abs(a.b + a.t - b.b - b.t);
}

static Node 
SubtreeInsert(Node subtree, Node leaf, cpBBTree tree)
{
	if(subtree == null){
		return leaf;
	} else if(NodeIsLeaf(subtree)){
		return NodeNew(tree, leaf, subtree);
	} else {
		double cost_a = cpBBArea(subtree.B.bb) + cpBBMergedArea(subtree.A.bb, leaf.bb);
		double cost_b = cpBBArea(subtree.A.bb) + cpBBMergedArea(subtree.B.bb, leaf.bb);
		
		if(cost_a == cost_b){
			cost_a = cpBBProximity(subtree.A.bb, leaf.bb);
			cost_b = cpBBProximity(subtree.B.bb, leaf.bb);
		}
		
		if(cost_b < cost_a){
			NodeSetB(subtree, SubtreeInsert(subtree.B, leaf, tree));
		} else {
			NodeSetA(subtree, SubtreeInsert(subtree.A, leaf, tree));
		}
		
		subtree.bb = cpBBMerge(subtree.bb, leaf.bb);
		return subtree;
	}
}

static void
SubtreeQuery(Node subtree, object obj, cpBB bb, cpSpatialIndexQueryFunc func, object data)
{
	if(cpBBIntersects(subtree.bb, bb)){
		if(NodeIsLeaf(subtree)){
			func(obj, subtree.obj, data);
		} else {
			SubtreeQuery(subtree.A, obj, bb, func, data);
			SubtreeQuery(subtree.B, obj, bb, func, data);
		}
	}
}


static double
SubtreeSegmentQuery(Node subtree, object obj, cpVect a, cpVect b, double t_exit, cpSpatialIndexSegmentQueryFunc func, object data)
{
	if(NodeIsLeaf(subtree)){
		return func(obj, subtree.obj, data);
	} else {
		double t_a = cpBBSegmentQuery(subtree.A.bb, a, b);
		double t_b = cpBBSegmentQuery(subtree.B.bb, a, b);
		
		if(t_a < t_b){
			if(t_a < t_exit) t_exit = System.Math.Min(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
			if(t_b < t_exit) t_exit = System.Math.Min(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
		} else {
			if(t_b < t_exit) t_exit = System.Math.Min(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
			if(t_a < t_exit) t_exit = System.Math.Min(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
		}
		
		return t_exit;
	}
}

static void
SubtreeRecycle(cpBBTree tree, Node node)
{
	if(!NodeIsLeaf(node)){
		SubtreeRecycle(tree, node.A);
		SubtreeRecycle(tree, node.B);
		NodeRecycle(tree, node);
	}
}

static Node 
SubtreeRemove(Node subtree, Node leaf, cpBBTree tree)
{
	if(leaf == subtree){
		return null;
	} else {
		Node parent = leaf.parent;
		if(parent == subtree){
			Node other = NodeOther(subtree, leaf);
			other.parent = subtree.parent;
			NodeRecycle(tree, subtree);
			return other;
		} else {
			NodeReplaceChild(parent.parent, parent, NodeOther(parent, leaf), tree);
			return subtree;
		}
	}
}

//MARK: Marking Functions

// typedef struct MarkContext {
	cpBBTree tree;
	Node staticRoot;
	cpSpatialIndexQueryFunc func;
	object data;
} MarkContext;

static void
MarkLeafQuery(Node subtree, Node leaf, bool left, MarkContext *context)
{
	if(cpBBIntersects(leaf.bb, subtree.bb)){
		if(NodeIsLeaf(subtree)){
			if(left){
				PairInsert(leaf, subtree, context.tree);
			} else {
				if(subtree.STAMP < leaf.STAMP) PairInsert(subtree, leaf, context.tree);
				context.func(leaf.obj, subtree.obj, context.data);
			}
		} else {
			MarkLeafQuery(subtree.A, leaf, left, context);
			MarkLeafQuery(subtree.B, leaf, left, context);
		}
	}
}

static void
MarkLeaf(Node leaf, MarkContext *context)
{
	cpBBTree tree = context.tree;
	if(leaf.STAMP == GetMasterTree(tree).stamp){
		Node staticRoot = context.staticRoot;
		if(staticRoot) MarkLeafQuery(staticRoot, leaf, false, context);
		
		for(Node node = leaf; node.parent; node = node.parent){
			if(node == node.parent.A){
				MarkLeafQuery(node.parent.B, leaf, true, context);
			} else {
				MarkLeafQuery(node.parent.A, leaf, false, context);
			}
		}
	} else {
		Pair *pair = leaf.PAIRS;
		while(pair){
			if(leaf == pair.b.leaf){
				context.func(pair.a.leaf.obj, leaf.obj, context.data);
				pair = pair.b.next;
			} else {
				pair = pair.a.next;
			}
		}
	}
}

static void
MarkSubtree(Node subtree, MarkContext *context)
{
	if(NodeIsLeaf(subtree)){
		MarkLeaf(subtree, context);
	} else {
		MarkSubtree(subtree.A, context);
		MarkSubtree(subtree.B, context);
	}
}

//MARK: Leaf Functions

static Node 
LeafNew(cpBBTree tree, object obj, cpBB bb)
{
	Node node = NodeFromPool(tree);
	node.obj = obj;
	node.bb = GetBB(tree, obj);
	
	node.parent = null;
	node.STAMP = 0;
	node.PAIRS = null;
	
	return node;
}

static bool
LeafUpdate(Node leaf, cpBBTree tree)
{
	Node root = tree.root;
	cpBB bb = tree.spatialIndex.bbfunc(leaf.obj);
	
	if(!cpBBContainsBB(leaf.bb, bb)){
		leaf.bb = GetBB(tree, leaf.obj);
		
		root = SubtreeRemove(root, leaf, tree);
		tree.root = SubtreeInsert(root, leaf, tree);
		
		PairsClear(leaf, tree);
		leaf.STAMP = GetMasterTree(tree).stamp;
		
		return true;
	}
	
	return false;
}

static void VoidQueryFunc(object obj1, object obj2, object data){}

static void
LeafAddPairs(Node leaf, cpBBTree tree)
{
	cpSpatialIndex *dynamicIndex = tree.spatialIndex.dynamicIndex;
	if(dynamicIndex){
		Node dynamicRoot = GetRootIfTree(dynamicIndex);
		if(dynamicRoot){
			cpBBTree dynamicTree = GetTree(dynamicIndex);
			MarkContext context = {dynamicTree, null, null, null};
			MarkLeafQuery(dynamicRoot, leaf, true, &context);
		}
	} else {
		Node staticRoot = GetRootIfTree(tree.spatialIndex.staticIndex);
		MarkContext context = {tree, staticRoot, VoidQueryFunc, null};
		MarkLeaf(leaf, &context);
	}
}

//MARK: Memory Management Functions

cpBBTree 
cpBBTreeAlloc()
{
	return (cpBBTree )cpcalloc(1, sizeof(cpBBTree));
}

static int
leafSetEql(object obj, Node node)
{
	return (obj == node.obj);
}

static object 
leafSetTrans(object obj, cpBBTree tree)
{
	return LeafNew(tree, obj, tree.spatialIndex.bbfunc(obj));
}

cpSpatialIndex *
cpBBTreeInit(cpBBTree tree, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
{
	cpSpatialIndexInit((cpSpatialIndex *)tree, Klass(), bbfunc, staticIndex);
	
	tree.velocityFunc = null;
	
	tree.leaves = cpHashSetNew(0, (cpHashSetEqlFunc)leafSetEql);
	tree.root = null;
	
	tree.pooledNodes = null;
	tree.allocatedBuffers = cpArrayNew(0);
	
	tree.stamp = 0;
	
	return (cpSpatialIndex *)tree;
}

void
cpBBTreeSetVelocityFunc(cpSpatialIndex *index, cpBBTreeVelocityFunc func)
{
	if(index.klass != Klass()){
		// cpAssertWarn(false, "Ignoring cpBBTreeSetVelocityFunc() call to non-tree spatial index.");
		return;
	}
	
	((cpBBTree )index).velocityFunc = func;
}

cpSpatialIndex *
cpBBTreeNew(cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
{
	return cpBBTreeInit(cpBBTreeAlloc(), bbfunc, staticIndex);
}

static void
cpBBTreeDestroy(cpBBTree tree)
{
	cpHashSetFree(tree.leaves);
	
	if(tree.allocatedBuffers) cpArrayFreeEach(tree.allocatedBuffers, cpfree);
	cpArrayFree(tree.allocatedBuffers);
}

//MARK: Insert/Remove

static void
cpBBTreeInsert(cpBBTree tree, object obj, cpHashValue hashid)
{
	Node leaf = (Node )cpHashSetInsert(tree.leaves, hashid, obj, tree, (cpHashSetTransFunc)leafSetTrans);
	
	Node root = tree.root;
	tree.root = SubtreeInsert(root, leaf, tree);
	
	leaf.STAMP = GetMasterTree(tree).stamp;
	LeafAddPairs(leaf, tree);
	IncrementStamp(tree);
}

static void
cpBBTreeRemove(cpBBTree tree, object obj, cpHashValue hashid)
{
	Node leaf = (Node )cpHashSetRemove(tree.leaves, hashid, obj);
	
	tree.root = SubtreeRemove(tree.root, leaf, tree);
	PairsClear(leaf, tree);
	NodeRecycle(tree, leaf);
}

static bool
cpBBTreeContains(cpBBTree tree, object obj, cpHashValue hashid)
{
	return (cpHashSetFind(tree.leaves, hashid, obj) != null);
}

//MARK: Reindex

static void
cpBBTreeReindexQuery(cpBBTree tree, cpSpatialIndexQueryFunc func, object data)
{
	if(!tree.root) return;
	
	// LeafUpdate() may modify tree.root. Don't cache it.
	cpHashSetEach(tree.leaves, (cpHashSetIteratorFunc)LeafUpdate, tree);
	
	cpSpatialIndex *staticIndex = tree.spatialIndex.staticIndex;
	Node staticRoot = (staticIndex && staticIndex.klass == Klass() ? ((cpBBTree )staticIndex).root : null);
	
	MarkContext context = {tree, staticRoot, func, data};
	MarkSubtree(tree.root, &context);
	if(staticIndex && !staticRoot) cpSpatialIndexCollideStatic((cpSpatialIndex *)tree, staticIndex, func, data);
	
	IncrementStamp(tree);
}

static void
cpBBTreeReindex(cpBBTree tree)
{
	cpBBTreeReindexQuery(tree, VoidQueryFunc, null);
}

static void
cpBBTreeReindexObject(cpBBTree tree, object obj, cpHashValue hashid)
{
	Node leaf = (Node )cpHashSetFind(tree.leaves, hashid, obj);
	if(leaf){
		if(LeafUpdate(leaf, tree)) LeafAddPairs(leaf, tree);
		IncrementStamp(tree);
	}
}

//MARK: Query

static void
cpBBTreeSegmentQuery(cpBBTree tree, object obj, cpVect a, cpVect b, double t_exit, cpSpatialIndexSegmentQueryFunc func, object data)
{
	Node root = tree.root;
	if(root) SubtreeSegmentQuery(root, obj, a, b, t_exit, func, data);
}

static void
cpBBTreeQuery(cpBBTree tree, object obj, cpBB bb, cpSpatialIndexQueryFunc func, object data)
{
	if(tree.root) SubtreeQuery(tree.root, obj, bb, func, data);
}

//MARK: Misc

static int
cpBBTreeCount(cpBBTree tree)
{
	return cpHashSetCount(tree.leaves);
}

// typedef struct eachContext {
	cpSpatialIndexIteratorFunc func;
	object data;
} eachContext;

static void each_helper(Node node, eachContext context){context.func(node.obj, context.data);}

static void
cpBBTreeEach(cpBBTree tree, cpSpatialIndexIteratorFunc func, object data)
{
	eachContext context = {func, data};
	cpHashSetEach(tree.leaves, (cpHashSetIteratorFunc)each_helper, &context);
}

static cpSpatialIndexClass klass = {
	(cpSpatialIndexDestroyImpl)cpBBTreeDestroy,
	
	(cpSpatialIndexCountImpl)cpBBTreeCount,
	(cpSpatialIndexEachImpl)cpBBTreeEach,
	
	(cpSpatialIndexContainsImpl)cpBBTreeContains,
	(cpSpatialIndexInsertImpl)cpBBTreeInsert,
	(cpSpatialIndexRemoveImpl)cpBBTreeRemove,
	
	(cpSpatialIndexReindexImpl)cpBBTreeReindex,
	(cpSpatialIndexReindexObjectImpl)cpBBTreeReindexObject,
	(cpSpatialIndexReindexQueryImpl)cpBBTreeReindexQuery,
	
	(cpSpatialIndexQueryImpl)cpBBTreeQuery,
	(cpSpatialIndexSegmentQueryImpl)cpBBTreeSegmentQuery,
};

static cpSpatialIndexClass Klass(){return klass;}


//MARK: Tree Optimization

static int
cpfcompare(double a, double b){
	return (a < b ? -1 : (b < a ? 1 : 0));
}

static void
fillNodeArray(Node node, Node[] cursor, ref int idx){
	cursor[idx] = node;
	idx++;
}

static Node 
partitionNodes(cpBBTree tree, Node[] nodes, int count)
{
	if(count == 1){
		return nodes[0];
	} else if(count == 2) {
		return NodeNew(tree, nodes[0], nodes[1]);
	}
	
	// Find the AABB for these nodes
	cpBB bb = nodes[0].bb;
	for(int i=1; i<count; i++) bb = cpBBMerge(bb, nodes[i].bb);
	
	// Split it on it's longest axis
	bool splitWidth = (bb.r - bb.l > bb.t - bb.b);
	
	// Sort the bounds and use the median as the splitting point
	double bounds = (double )cpcalloc(count*2, sizeof(double));
	if(splitWidth){
		for(int i=0; i<count; i++){
			bounds[2*i + 0] = nodes[i].bb.l;
			bounds[2*i + 1] = nodes[i].bb.r;
		}
	} else {
		for(int i=0; i<count; i++){
			bounds[2*i + 0] = nodes[i].bb.b;
			bounds[2*i + 1] = nodes[i].bb.t;
		}
	}
	
	qsort(bounds, count*2, sizeof(double), (int (*)(object , object ))cpfcompare);
	double split = (bounds[count - 1] + bounds[count])*0.5f; // use the medain as the split
	cpfree(bounds);

	// Generate the child BBs
	cpBB a = bb, b = bb;
	if(splitWidth) a.r = b.l = split; else a.t = b.b = split;
	
	// Partition the nodes
	int right = count;
	for(int left=0; left < right;){
		Node node = nodes[left];
		if(cpBBMergedArea(node.bb, b) < cpBBMergedArea(node.bb, a)){
//		if(cpBBProximity(node.bb, b) < cpBBProximity(node.bb, a)){
			right--;
			nodes[left] = nodes[right];
			nodes[right] = node;
		} else {
			left++;
		}
	}
	
	if(right == count){
		Node node = null;
		for(int i=0; i<count; i++) node = SubtreeInsert(node, nodes[i], tree);
		return node;
	}
	
	// Recurse and build the node!
	return NodeNew(tree,
		partitionNodes(tree, nodes, right),
		partitionNodes(tree, nodes + right, count - right)
	);
}

//static void
//cpBBTreeOptimizeIncremental(cpBBTree tree, int passes)
//{
//	for(int i=0; i<passes; i++){
//		Node root = tree.root;
//		Node node = root;
//		int bit = 0;
//		unsigned int path = tree.opath;
//		
//		while(!NodeIsLeaf(node)){
//			node = (path&(1<<bit) ? node.a : node.b);
//			bit = (bit + 1)&(sizeof(unsigned int)*8 - 1);
//		}
//		
//		root = subtreeRemove(root, node, tree);
//		tree.root = subtreeInsert(root, node, tree);
//	}
//}

void
cpBBTreeOptimize(cpSpatialIndex *index)
{
	if(index.klass != &klass){
		// cpAssertWarn(false, "Ignoring cpBBTreeOptimize() call to non-tree spatial index.");
		return;
	}
	
	cpBBTree tree = (cpBBTree )index;
	Node root = tree.root;
	if(!root) return;
	
	int count = cpBBTreeCount(tree);
	Node[] nodes = (Node[] )cpcalloc(count, sizeof(Node ));
	Node[] cursor = nodes;
	
	cpHashSetEach(tree.leaves, (cpHashSetIteratorFunc)fillNodeArray, &cursor);
	
	SubtreeRecycle(tree, root);
	tree.root = partitionNodes(tree, nodes, count);
	cpfree(nodes);
}

//MARK: Debug Draw


static void
NodeRender(Node node, int depth)
{
	if(!NodeIsLeaf(node) && depth <= 10){
		NodeRender(node.a, depth + 1);
		NodeRender(node.b, depth + 1);
	}
	
	cpBB bb = node.bb;
	
//	GLdouble v = depth/2.0f;	
//	glColor3f(1.0f - v, v, 0.0f);
	glLineWidth(System.Math.Max(5.0f - depth, 1.0f));
	glBegin(GL_LINES); {
		glVertex2f(bb.l, bb.b);
		glVertex2f(bb.l, bb.t);
		
		glVertex2f(bb.l, bb.t);
		glVertex2f(bb.r, bb.t);
		
		glVertex2f(bb.r, bb.t);
		glVertex2f(bb.r, bb.b);
		
		glVertex2f(bb.r, bb.b);
		glVertex2f(bb.l, bb.b);
	}; glEnd();
}

void
cpBBTreeRenderDebug(cpSpatialIndex *index){
	if(index.klass != &klass){
		// cpAssertWarn(false, "Ignoring cpBBTreeRenderDebug() call to non-tree spatial index.");
		return;
	}
	
	cpBBTree tree = (cpBBTree )index;
	if(tree.root) NodeRender(tree.root, 0);
}
}
}
