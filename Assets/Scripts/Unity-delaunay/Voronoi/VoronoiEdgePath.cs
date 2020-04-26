using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace VoronoiNS
{

  public enum VoronoiEdgePathNodeType { EDGE, VERTEX };
  
  public class VoronoiEdgePathNode
  {
    public VoronoiCellEdge edge;
    public VoronoiCellVertex vertex;
    public VoronoiEdgePathNodeType nodeType;
    
    public VoronoiEdgePathNode( VoronoiCellEdge e )
    {
      nodeType = VoronoiEdgePathNodeType.EDGE;
      edge = e;
    }
    
    public VoronoiEdgePathNode( VoronoiCellVertex v )
    {
      nodeType = VoronoiEdgePathNodeType.VERTEX;
      vertex = v;
    }
  }
  
/**
Ordered list of VoronoiCellEdge objects, with each one's orientation (A to B, or B to A)
stored within the list, so you can easily travers it VoronoiCellVertex to VoronoiCellVertex
*/
public class VoronoiEdgePath
{

public List<VoronoiEdgePathNode> pathNodes;

/** Special case: only use this when you have DISCONTINUOUS edges
and need to jump between them.

For all other cases, use AddEdge to have it auatomatically add the ends
intelligently as needed with no dupes
*/
    public void AddJumpToVertex( VoronoiCellVertex vertex )
    {
      if( pathNodes == null )
      {
        pathNodes = new List<VoronoiEdgePathNode>();
      }
      
      pathNodes.Add ( new VoronoiEdgePathNode(vertex));
      }
      
public void AddEdge( VoronoiCellEdge edge, bool isAFirstBSecond )
{
if( pathNodes == null )
{
pathNodes = new List<VoronoiEdgePathNode>();
}

/** When you add first edge, need to pre-add its first vertex too - but only for first edge! */
if( pathNodes.Count < 1 )
{
				if( isAFirstBSecond )
					pathNodes.Add ( new VoronoiEdgePathNode(edge.edgeVertexA));
				else
					pathNodes.Add ( new VoronoiEdgePathNode(edge.edgeVertexB));
}

pathNodes.Add( new VoronoiEdgePathNode(edge) );

if( isAFirstBSecond )
pathNodes.Add ( new VoronoiEdgePathNode(edge.edgeVertexB));
else
				pathNodes.Add ( new VoronoiEdgePathNode(edge.edgeVertexA));
}
}
}