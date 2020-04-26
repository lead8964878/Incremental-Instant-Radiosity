using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using Delaunay.LR;

/** Unity-friendly Output from runnning the ported AS3Delaunay library

 "VoronoiNS" is a silly namespace name, but C# is buggy and can't cope with a class and namespace with same
name, and the original port (ab)uses Voronoi as a class-name */
namespace VoronoiNS
{
public class VoronoiCellEdge : MonoBehaviour {

/** Note: order is irrelevant here, for some Cells it will be 1,2, others it will be 2,1 */
public VoronoiCell cell1, cell2;
/** Note: order is irrelevant here, for some Cells it will be A,B, others it will be B,A */
public VoronoiCellVertex edgeVertexA, edgeVertexB;

		public void AddVertex( VoronoiCellVertex vertex )
		{
		if( edgeVertexA == null )
		edgeVertexA = vertex;
		else if( edgeVertexB == null )
		edgeVertexB = vertex;
		else
		{
		Debug.LogError("Impossible: an edge can only have 2 vertices, not 3!");
		return;
		}
		
		if( !vertex.ContainsEdge( this ))
				vertex.AddEdge( this );
		}
/** The side-effects you must be aware of:

1. The cell is added to the two Vertex's on the Edge
2. If this is the 2nd Cell added, the two Cells are explicitly connected to each other internally
*/
public void AddCellWithSideEffects( VoronoiCell c )
{
/** Add the cell or crash */
if( cell1 == null )
cell1 = c;
else if( cell2 == null )
cell2 = c;
else
{
Debug.LogError("Cannot add a third cell to an edge!");
return;
}

/** Add the cell to both vertices on this edge */
if( edgeVertexA == null || edgeVertexB == null )
{
Debug.LogError("Fail: this edge has one or both edgeVertex nodes missing; cannot continue");
return;
}
else
{
if( ! edgeVertexA.ConnectsToCell( c ) ) // might have been added by a previous edge
edgeVertexA.AddCellWithSideEffects( c );
        if( ! edgeVertexB.ConnectsToCell( c ) ) // might have been added by a previous edge
				edgeVertexB.AddCellWithSideEffects( c );
}

/** If both cells present, connect them together via this edge */
if( cell1 != null && cell2 != null )
{
/* now we can join the two cells via this edge */
if( cell1.neighbours == null )
					cell1.neighbours = new Dictionary<VoronoiCellEdge, VoronoiCell>();
cell1.neighbours.Add( this, cell2 );

if( cell2.neighbours == null )
					cell2.neighbours = new Dictionary<VoronoiCellEdge, VoronoiCell>();
			cell2.neighbours.Add( this, cell1 );
}
}
}
}