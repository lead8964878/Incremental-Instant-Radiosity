using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using System.Linq;

/** Unity-friendly Output from runnning the ported AS3Delaunay library

 "VoronoiNS" is a silly namespace name, but C# is buggy and can't cope with a class and namespace with same
name, and the original port (ab)uses Voronoi as a class-name */
namespace VoronoiNS
{
  public class VoronoiCell : MonoBehaviour, ISerializationCallbackReceiver /** workaround for ENORMOUS bugs in Unity Serialization, c.f. http://blogs.unity3d.com/2014/06/24/serialization-in-unity/ */
  {

    /** NOTE: Unity3D's Serialization is fundamentally broken, and CANNOT serialize Dictionary's, no matter what you do */
    public Dictionary<VoronoiCellEdge,VoronoiCell> neighbours;
                
    /** Exists ONLY to workaround Unity bugs in Serialization */
    public List<VoronoiCellEdge> serializedEdgeList;
    /** Exists ONLY to workaround Unity bugs in Serialization */
    public List<VoronoiCell> serializedCellList;
        
    public void OnBeforeSerialize()
    {
      serializedEdgeList = new List<VoronoiCellEdge>();
      serializedCellList = new List<VoronoiCell>();
        
      foreach (KeyValuePair<VoronoiCellEdge,VoronoiCell> item in neighbours)
      {
        serializedEdgeList.Add(item.Key);
        serializedCellList.Add(item.Value);
      }
    }
        
    public void OnAfterDeserialize()
    {
      neighbours = new Dictionary<VoronoiCellEdge, VoronoiCell>();
      for (int i=0; i<serializedEdgeList.Count; i++)
      {
        neighbours.Add(serializedEdgeList [i], serializedCellList [i]);
      }
    }
        
    public VoronoiCell parent {
      get {
        if (gameObject.transform.parent == null)
          return null;
        
        return GetComponentInParent<VoronoiCell>();
      }
    }
                
    /** AS3Delaunay library automatically gives us this, and its useless for most things, but great when
                trying to make a simple mesh.
                
                 Complex meshes would need to know the original Edge objects, so they can set vertex-colouring,
                 vertex-texture-blending etc (which this CANNOT provide), but we might as well make the simple
                 version easy while we can! 
                 
                 TODO: border cells will randomly miss some of their edges because this
                 method stops when it hits an outside / infinite edge, and does NOT scan
                 through the dictionary to try and find a resume point
                 */
    //public List<Vector2> orderedPointsOnCircumference;
    public VoronoiEdgePath CalculateOrderedPathAroundEdges()
    {
      VoronoiEdgePath path = new VoronoiEdgePath();
        
      /** pick a random edge */
      VoronoiCellEdge firstEdge = neighbours.Keys.ToList().First();
            
      VoronoiCellEdge currentEdge = firstEdge;
      path.AddEdge(currentEdge, true); // true means nextVertex will be B, false means A; you can choose, just implement next few lines appropriately!
        
      /** Trace the edge, and when yuo reach a vertex, find the only OTHER edge from 
        that vertex that leads to a vertex that is ALSO on this cell */
      VoronoiCellVertex nextVertex = currentEdge.edgeVertexB;
      VoronoiCellEdge nextEdge = nextVertex.GetOtherEdgeContainingCell(currentEdge, this);
      while (nextEdge != firstEdge)
      {
        if (nextEdge == null)
        {
          /** We hit the end prematurely; means this Cell is on the outer edge of diagram
        and is UNCLOSED.
        
        So we need to scan through neighbours looking for a vertex that
        only has ONE edge in this cell, but is NOT the last edge (which meets that criteria too)
        */
          foreach (KeyValuePair<VoronoiCellEdge, VoronoiCell> item in neighbours)
          {
            if (item.Key == currentEdge)
              continue; // found the edge we're already on
        
            VoronoiCellVertex newNextVertex = null;
            if (item.Key.edgeVertexA.GetEdgesBorderingCell(this).Count < 2)
            {
              /** we've found the other one. Now need to start from that vertex.
        */
              newNextVertex = item.Key.edgeVertexA;
            }
            else if (item.Key.edgeVertexB.GetEdgesBorderingCell(this).Count < 2)
            {
              /** we've found the other one. Now need to start from that vertex.
        */
              newNextVertex = item.Key.edgeVertexB;
            }
        
            if (newNextVertex != null)
            {
              /** Need to FORCE-ADD that vertex onto the path too, since we're jumping
         the gap */
              path.AddJumpToVertex(newNextVertex);
         
              nextVertex = newNextVertex;
              nextEdge = item.Key;
            }
          }
          
          if (nextEdge == firstEdge) // rare, but can happen!
            break; // we added the forced jump to vertex, but have no more edges to add, so stop
            
          if (nextEdge == null)
          {
            /** Something has gone badly wrong */
            Debug.LogError("Major error: was trying to close the loop of an outer Cell, but couldn't find a restart point");
            break;
          }
        }
        
        /** ... normal body of while loop starts here ... */
        bool isNextEdgeAToB = (nextVertex == nextEdge.edgeVertexA);
        path.AddEdge(nextEdge, isNextEdgeAToB);
                
        if (isNextEdgeAToB)
          nextVertex = nextEdge.edgeVertexB;
        else
          nextVertex = nextEdge.edgeVertexA;
        
        currentEdge = nextEdge;
        nextEdge = nextVertex.GetOtherEdgeContainingCell(currentEdge, this);
      }
        
      return path;
    }
    
    private List<Vector2> _cachedGizmoOutline;

    void OnDrawGizmosSelected()
    {
    Debug.Log("GIZMOS SELECTED");
     // if (_cachedGizmoOutline == null)
      {
        _cachedGizmoOutline = new List<Vector2>();
    
        /** Convert the cell's edge-path into a list of simple 2D co-ordinates
                */
        VoronoiEdgePath edgePath = CalculateOrderedPathAroundEdges();
        foreach (VoronoiEdgePathNode node in edgePath.pathNodes)
        {
          if (node.nodeType == VoronoiEdgePathNodeType.VERTEX)
            _cachedGizmoOutline.Add(node.vertex.positionInDiagram);
        }
      }
      
      Debug.Log("GIZMOS SELECTED: NODES = "+_cachedGizmoOutline.Count);
      
      Gizmos.color = new Color(Random.Range(0, 1f), Random.Range(0, 1f), Random.Range(0, 1f));
      
      for (int i = 0; i+1< _cachedGizmoOutline.Count; i++)
      {
        Vector2 left = _cachedGizmoOutline [i];
        Vector2 right = _cachedGizmoOutline [i + 1];
        Gizmos.DrawLine((Vector3)left, (Vector3)right);
      }
    }
    
    void Start()
    {
    }
    
    // Update is called once per frame
    void Update()
    {
    
    }
  }
}
