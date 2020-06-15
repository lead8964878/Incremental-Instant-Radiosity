using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Voronoi_Delaunay
{
    struct CenterOrEdge
    {
        public
        CenterOrEdge(bool _isCenter, int _idx, Vector3 _edgePos,int _which)
        {
            isCenter = _isCenter;
            idx = _idx;
            edgePos = _edgePos;
            which = _which;
        }
        public bool isCenter;
        public int idx;
        public Vector3 edgePos;
        public int which;
    }
    struct EdgeLengthUtil
    {
        public
        EdgeLengthUtil(int _idx, int _which)
        {
            idx = _idx;
            which = _which;
        }
        public int idx;
        public int which;
    }
    class Collections
    {
        public static List<Point> allPoints1;
        public static List<Triangle> allTriangles1;
        public static List<Point> allPoints2;
        public static List<Triangle> allTriangles2;
        public static SortedList<double, CenterOrEdge> minCircleSizeList1;
        public static SortedList<double, CenterOrEdge> minCircleSizeList2;
        public static SortedList<double, EdgeLengthUtil> edgeLengthList1;
        public static SortedList<double, EdgeLengthUtil> edgeLengthList2;
    }
}
