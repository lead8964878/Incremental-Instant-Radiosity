using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

namespace Voronoi_Delaunay
{
    class Voronoi
    {
        public static List<VoronoiEdge> VoronoiEdges(List<DelaunayEdge> delaunayEdges)
        {
            List<VoronoiEdge> voronoiEdgeList = new List<VoronoiEdge>();

            for (int i = 0; i < delaunayEdges.Count; i++)
            {
                List<int> neighbours = new List<int>();
                for (int j = 0; j < Collections.allPoints[delaunayEdges[i].start].adjoinTriangles.Count; j++)
                {
                    for (int k = 0; k < Collections.allPoints[delaunayEdges[i].end].adjoinTriangles.Count; k++)
                    {
                        if (Collections.allPoints[delaunayEdges[i].start].adjoinTriangles[j] == Collections.allPoints[delaunayEdges[i].end].adjoinTriangles[k])
                        {
                            neighbours.Add(Collections.allPoints[delaunayEdges[i].start].adjoinTriangles[j]);
                        }
                    }
                }
                VoronoiEdge voronoiEdge = new VoronoiEdge(Collections.allTriangles[neighbours[0]].center, Collections.allTriangles[neighbours[1]].center);
                voronoiEdgeList.Add(voronoiEdge);
            }

            
            return voronoiEdgeList;
        }

        public static int cn_PnPoly(Point P,ref List<Point> V)
        {
            int n = V.Count-1;
            int cn = 0;    // the  crossing number counter

            // loop through all edges of the polygon
            for (int i = 0; i < n; i++)
            {    // edge from V[i]  to V[i+1]
                if (((V[i].y <= P.y) && (V[i + 1].y > P.y))     // an upward crossing
                 || ((V[i].y > P.y) && (V[i + 1].y <= P.y)))
                { // a downward crossing
                  // compute  the actual edge-ray intersect x-coordinate
                    float vt = (float)(P.y - V[i].y) / (float)(V[i + 1].y - V[i].y);
                    if (P.x < V[i].x + vt * (V[i + 1].x - V[i].x)) // P.x < intersect
                        ++cn;   // a valid crossing of y=P.y right of P.x
                }
            }
            return (cn & 1);    // 0 if even (out), and 1 if  odd (in)

        }

        public static int intersect2D_SegPoly(Point p0, Point p1, ref List<Point> V,ref Point o0,ref Point o1)
        {

            if (p0 == p1)
            {         // the segment S is a single point
                      // test for inclusion of S.P0 in the polygon
                o0 = p0;
                o1 = p1;// same point if inside polygon
                return cn_PnPoly(p0,ref V);
            }

            int n = V.Count-1;

            float tE = 0;              // the maximum entering segment parameter
            float tL = 1;              // the minimum leaving segment parameter
            float t, N, D;             // intersect parameter t = N / D
            Point dS = p1-p0;     // the  segment direction vector
            Point e;                   // edge vector
                                        // Vector ne;               // edge outward normal (not explicit in code)

            for (int i = 0; i < n; i++)   // process polygon edge V[i]V[i+1]
            {
                e = V[i + 1] - V[i];
                N = (float)p0.perp(e, p0 - V[i]); // = -dot(ne, S.P0 - V[i])
                D = (float)-p0.perp(e, dS);       // = dot(ne, dS)
                if (Math.Abs(D) < 0.0000001f)
                {  // S is nearly parallel to this edge
                    if (N < 0)              // P0 is outside this edge, so
                        return 0;      // S is outside the polygon
                    else                    // S cannot cross this edge, so
                        continue;          // ignore this edge
                }

                t = N / D;
                if (D < 0)
                {            // segment S is entering across this edge
                    if (t > tE)
                    {       // new max tE
                        tE = t;
                        if (tE > tL)   // S enters after leaving polygon
                            return 0;
                    }
                }
                else
                {                  // segment S is leaving across this edge
                    if (t < tL)
                    {       // new min tL
                        tL = t;
                        if (tL < tE)   // S leaves before entering polygon
                            return 0;
                    }
                }
            }

            // tE <= tL implies that there is a valid intersection subsegment
            o0 = p0 + tE * dS;   // = P(tE) = point where S enters polygon
            o1 = p0 + tL * dS;   // = P(tL) = point where S leaves polygon
            return 1;
        }

        public static List<double> calculateAreas()
        {
            List<double> allAreas = new List<double>();
            for(int i = 0; i < Collections.allPoints.Count; i++)
            {
                List<Tuple<Point, double>> topPoints = new List<Tuple<Point, double>>();
                List<Tuple<Point, double>> bottomPoints = new List<Tuple<Point, double>>();

                Point delaunayPoint = Collections.allPoints[i];

                for (int j = 0; j < delaunayPoint.adjoinTriangles.Count; j++)
                {
                    Point center = Collections.allTriangles[delaunayPoint.adjoinTriangles[j]].center;
                    double dotValue = delaunayPoint.x * center.x + delaunayPoint.y * center.y + delaunayPoint.z * center.z;
                    if(center.y >= delaunayPoint.y)
                    {
                        topPoints.Add(new Tuple<Point, double>(center, dotValue));
                    }
                    else
                    {
                        bottomPoints.Add(new Tuple<Point, double>(center, dotValue));
                    }
                }
                //check topPoints
                topPoints.Sort((a, b) => b.Item2.CompareTo(a.Item2));
                bottomPoints.Sort((a, b) => a.Item2.CompareTo(b.Item2));
                topPoints.AddRange(bottomPoints);

                double area = 0.0f;
                for(int j = 0; j < topPoints.Count; j++)
                {
                    Point d2st = topPoints[j].Item1 - delaunayPoint;
                    Point d2ed = topPoints[(j + 1) % topPoints.Count].Item1 - delaunayPoint;

                    Point crossVal = d2st.cross(d2st, d2ed);
                    area += Math.Sqrt(crossVal.x * crossVal.x + crossVal.y * crossVal.y + crossVal.z * crossVal.z) * 0.5;
                }
                allAreas.Add(area);
            }
            return allAreas;
        }


        public static List<VoronoiEdge> VoronoiEdges(List<Triangle> allTriangles)
        {
            List<VoronoiEdge> voronoiEdgeList = new List<VoronoiEdge>();

            for (int i = 0; i < allTriangles.Count; i++)
            {
                for (int j = 0; j < allTriangles.Count; j++)
                {
                    if (j != i)
                    {
                        DelaunayEdge neighborEdge = allTriangles[i].FindCommonEdgeWith(allTriangles[j]);
                        if (neighborEdge != null)
                        {
                            VoronoiEdge voronoiEdge = new VoronoiEdge(allTriangles[i].center, allTriangles[j].center);
                            if (!voronoiEdgeList.Contains(voronoiEdge))
                            {
                                voronoiEdgeList.Add(voronoiEdge);
                            }
                        }
                    }
                }
            }

            return voronoiEdgeList;
        }
    }
}
