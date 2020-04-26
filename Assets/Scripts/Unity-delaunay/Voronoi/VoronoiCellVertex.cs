using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace VoronoiNS
{
		public class VoronoiCellVertex : MonoBehaviour
		{
				/** (0,0) is the origin of the VoronoiDiagram, not the world-origin, nor any Unity 3D transform */
				public Vector2 positionInDiagram;
		
				/**
		By definition, a Voronoi vertex is the meeting of precisely 3 edges, unless it is the clipped
		vertex at the outer edge of the diagram, in which case it is the meeting point of technically
		only one edge, or two edges if you manually "CLOSE" the outer voronoi cells.
		
		So we hardcode a max of 3 cells here.
		
		 Note: order is irrelevant here, for some Cells it will be 1,2,3 others it will be 2,1,3, etc
         Note: Unity has many serialization bugs with arrays, its just not worth the
                pain and suffering to try working around all of them.
                
                Give up. Instead of a 3-element array, use 3 variables.
                */
				public VoronoiCell cell1, cell2, cell3;
                /** Note: Unity has many serialization bugs with arrays, its just not worth the
                pain and suffering to try working around all of them.
                
                Give up. Instead of a 3-element array, use 3 variables.
                */
				public VoronoiCellEdge edge1, edge2, edge3;
		
				public void AddCellWithSideEffects (VoronoiCell c)
				{
						/** Debug check / error checking */
						if (cell1 == c || cell2 == c || cell3 == c) {
        Debug.LogError ("Major error: you re-added a cell ("+c.name+") to a vertex  ("+this.name+")  that already had that cell; this suggests you've run a cel/vertex generation algorithm twice on the same VoronoiDiagram object (should never happen!)");
						}
				
						if (cell1 == null)
								cell1 = c;
						else if (cell2 == null)
								cell2 = c;
						else if (cell3 == null)
								cell3 = c;
						else {
        Debug.LogError ("Cannot add a fourth cell  ("+c.name+")  to a vertex  ("+this.name+") !");
								return;
						}
                        
      Debug.Log ("  vertex  ("+this.name+") + cell = ("+(cell1==null?"":cell1.name)+", "+(cell2==null?"":cell2.name)+", "+(cell3==null?"":cell3.name)+")  to ");
				}
				
                public bool ConnectsToCell( VoronoiCell c )
                {
                if( c == null )
                return false;
      return (c == cell1 || c == cell2 || c == cell3 );
                }
				public bool ContainsEdge (VoronoiCellEdge e)
				{
                if( e == null )
        return false;
						return (e == edge1 || e == edge2 || e == edge3);
				}
				
				public void AddEdge (VoronoiCellEdge e)
				{
						if( edge1 == null )
                        edge1 = e;
      else if( edge2 == null )
        edge2 = e;
      else if( edge3 == null )
        edge3 = e;
			
				}
		
        /**
        BORDER means "edges from this vertex that run along the perimeter of that cell"
        
        Useful when trying to draw a path around the perimeter of a cell, in several
        different ways (especially: when trying to auto-close that path if it is on boundary
        of Diagram and has a pair of infinite / unterminated, but disconnected, edges)
        */
        public List<VoronoiCellEdge> GetEdgesBorderingCell( VoronoiCell c )
        {
        List<VoronoiCellEdge> result = new List<VoronoiCellEdge>();
        
        if( !ConnectsToCell(c)) // This vertex may have an edge that touches that cell, but not border
        return result;
        
      foreach ( VoronoiCellEdge e in new VoronoiCellEdge[] { edge1, edge2, edge3})
      {
      if( e == null )
      continue;
      
      if( (e.edgeVertexA == this && e.edgeVertexB.ConnectsToCell(c))
      || (e.edgeVertexB == this && e.edgeVertexA.ConnectsToCell(c)) )
      result.Add( e );
      }
      
      return result;
        }
				public VoronoiCellEdge GetOtherEdgeContainingCell (VoronoiCellEdge currentEdge, VoronoiCell targetCell)
				{
                		foreach ( VoronoiCellEdge e in new VoronoiCellEdge[] { edge1, edge2, edge3})
                        {
                        if( e == null )
                        continue;
                        
								if (e == currentEdge)
										continue;
			
								if (e.cell1 == targetCell
										|| e.cell2 == targetCell)
										return e;
				   
						}
        
                        if( edge3 != null )
                        {                
                        /** this is ONLY an error if all 3 edges existed; otherwise its an expected outcome */
      Debug.LogError ("Cannot find another edge from this vertex (" + this + ") that borders cell: " + targetCell);
      }
      return null;
				}
		}
}