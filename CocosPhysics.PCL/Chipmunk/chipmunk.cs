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
    public struct cpMat2x2 {
	// Row major [[a, b][c d]]
	public double a, b, c, d;
};

public static void
cpMessage(string condition, string file, int line, bool isError, bool isHardError, string message, params string va_list[])
{
	System.Diagnostics.Debug.WriteLine((isError ? "Aborting due to Chipmunk error: " : "Chipmunk warning: "));
	
		System.Diagnostics.Debug.WriteLine(message, va_list);
	
	System.Diagnostics.Debug.WriteLine("\tFailed condition: {0}\n", condition);
	System.Diagnostics.Debug.WriteLine("\tSource:{0}:{1}\n", file, line);
}

string cpVersionString = "1.0.0.0";

void
cpInitChipmunk()
{
}

//MARK: Misc Functions

double
cpMomentForCircle(double m, double r1, double r2, cpVect offset)
{
	return m*(0.5f*(r1*r1 + r2*r2) + offset.LengthSQ);
}

double
cpAreaForCircle(double r1, double r2)
{
	return (double)System.Math.PI*System.Math.Abs(r1*r1 - r2*r2);
}

double
cpMomentForSegment(double m, cpVect a, cpVect b)
{
	cpVect offset = cpVect.Multiply(cpVect.Add(a, b), 0.5);
	return m*(cpVect.DistanceSQ(b, a)/12.0f + offset.LengthSQ);
}

double
cpAreaForSegment(cpVect a, cpVect b, double r)
{
	return r*((double)System.Math.PI*r + 2.0*cpVect.Distance(a, b));
}

double
cpMomentForPoly(double m, int numVerts, cpVect[] verts, cpVect offset)
{
	double sum1 = 0.0f;
	double sum2 = 0.0f;
	for(int i=0; i<numVerts; i++){
		cpVect v1 = cpVect.Add(verts[i], offset);
		cpVect v2 = cpVect.Add(verts[(i+1)%numVerts], offset);
		
		double a = cpVect.CrossProduct(v2, v1);
		double b = cpVect.Dot(v1, v1) + cpVect.Dot(v1, v2) + cpVect.Dot(v2, v2);
		
		sum1 += a*b;
		sum2 += a;
	}
	
	return (m*sum1)/(6.0f*sum2);
}

double
cpAreaForPoly(int numVerts, cpVect[] verts)
{
	double area = 0.0f;
	for(int i=0; i<numVerts; i++){
		area += cpVect.CrossProduct(verts[i], verts[(i+1)%numVerts]);
	}
	
	return -area/2.0f;
}

cpVect
cpCentroidForPoly(int numVerts, cpVect[] verts)
{
	double sum = 0.0f;
	cpVect vsum = cpvzero;
	
	for(int i=0; i<numVerts; i++){
		cpVect v1 = verts[i];
		cpVect v2 = verts[(i+1)%numVerts];
		double cross = cpVect.CrossProduct(v1, v2);
		
		sum += cross;
		vsum = cpVect.Add(vsum, cpVect.Multiply(cpVect.Add(v1, v2), cross));
	}
	
	return cpVect.Multiply(vsum, 1.0f/(3.0f*sum));
}

void
cpRecenterPoly(int numVerts, cpVect[] verts){
	cpVect centroid = cpCentroidForPoly(numVerts, verts);
	
	for(int i=0; i<numVerts; i++){
		verts[i] = cpVect.Sub(verts[i], centroid);
	}
}

double
cpMomentForBox(double m, double width, double height)
{
	return m*(width*width + height*height)/12.0f;
}

double
cpMomentForBox2(double m, cpBB box)
{
	double width = box.r - box.l;
	double height = box.t - box.b;
	cpVect offset = cpVect.Multiply(cpv(box.l + box.r, box.b + box.t), 0.5f);
	
	// TODO NaN when offset is 0 and m is double.PositiveInfinity
	return cpMomentForBox(m, width, height) + m*offset.LengthSQ;
}

//MARK: Quick Hull

void
cpLoopIndexes(cpVect[] verts, int count, ref int start, ref int end)
{
	start = end = 0;
	cpVect min = verts[0];
	cpVect max = min;
	
  for(int i=1; i<count; i++){
    cpVect v = verts[i];
		
    if(v.x < min.x || (v.x == min.x && v.y < min.y)){
      min = v;
      start = i;
    } else if(v.x > max.x || (v.x == max.x && v.y > max.y)){
			max = v;
			end = i;
		}
	}
}

static int
QHullPartition(cpVect[] verts, int count, cpVect a, cpVect b, double tol)
{
	if(count == 0) return 0;
	
	double max = 0;
	int pivot = 0;
	
	cpVect delta = cpVect.Sub(b, a);
	double valueTol = tol*delta.Length;
	
	int head = 0;
	for(int tail = count-1; head <= tail;){
		double value = cpVect.CrossProduct(delta, cpVect.Sub(verts[head], a));
		if(value > valueTol){
			if(value > max){
				max = value;
				pivot = head;
			}
			
			head++;
		} else {
			SWAP(ref verts[head], ref verts[tail]);
			tail--;
		}
	}
	
	// move the new pivot to the front if it's not already there.
	if(pivot != 0) SWAP(ref verts[0], ref verts[pivot]);
	return head;
}

static int
QHullReduce(double tol, cpVect[] verts, int offset, int count, cpVect a, cpVect pivot, cpVect b, cpVect[] result, int roffset)
{
	if(count < 0){
		return 0;
	} else if(count == 0) {
		result[roffset] = pivot;
		return 1;
	} else {
		int left_count = QHullPartition(verts, offset, count, a, pivot, tol);
		int index = QHullReduce(tol, verts, offset + 1, left_count - 1, a, verts[offset], pivot, result, roffset);
		
		result[roffset + index] = pivot;
		index++;

		int right_count = QHullPartition(verts, offset, offset + left_count, count - left_count, pivot, b, tol);
		return index + QHullReduce(tol, verts, offset + left_count + 1, right_count - 1, pivot, verts[offset + left_count], b, result, index);
	}
}

    static void SWAP(ref cpVect a, ref cpVect b) {
        cpVect h = a;
        a = b;
        b = h;
    }

// QuickHull seemed like a neat algorithm, and efficient-ish for large input sets.
// My implementation performs an in place reduction using the result array as scratch space.
int
cpConvexHull(int count, cpVect[] verts, int offset, cpVect[] result, ref int first, double tol)
{
	if(result != null){
		// Copy the line vertexes into the empty part of the result polyline to use as a scratch buffer.
        Array.Copy(verts, offset, result, 0, count);
        verts.CopyTo(result, count);
	} else {
		// If a result array was not specified, reduce the input instead.
        verts.CopyTo(result, 0);
	}
	
	// Degenerate case, all poins are the same.
	int start, end;
	cpLoopIndexes(verts, count, ref start, ref end);
	if(start == end){
		first = 0;
		return 1;
	}
	
	SWAP(ref result[0], ref result[start]);
	SWAP(ref result[1], ref result[end == 0 ? start : end]);
	
	cpVect a = result[0];
	cpVect b = result[1];
	
	first = start;
	int resultCount = QHullReduce(tol, result, 2, count - 2, a, b, a, result, 1) + 1;
	return resultCount;
}
}
}

