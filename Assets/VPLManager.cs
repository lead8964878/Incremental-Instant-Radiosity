using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

using Voronoi_Delaunay;
public class VPLManager : MonoBehaviour
{
    public GameObject VPLCollector;
    public Light PrimaryLight;
    public Bounds bounds;

    public int VPLCount = 60;
    public int minimalDeletedVPLPerFrame = 4;
    List<Point> polygonSides = new List<Point>();

    List<Vector3> pointList1 = new List<Vector3>();
    List<Vector3> pointList2 = new List<Vector3>();

    List<Vector3> pointListCopy1 = new List<Vector3>();
    List<Vector3> pointListCopy2 = new List<Vector3>();

    List<int> pointMapping1 = new List<int>();
    List<int> pointMapping2 = new List<int>();

    List<Triangle> realTriangleList1 = new List<Triangle>();
    List<Triangle> realTriangleList2 = new List<Triangle>();

    List<Vector3> pointsOnEdgeList1 = new List<Vector3>();
    List<Vector3> pointsOnEdgeList2 = new List<Vector3>();

    double getLength(Point a, Point b)
    {
        return Mathf.Sqrt((float)((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
    }

    Triangle superTriangle1 = new Triangle();
    Triangle superTriangle2 = new Triangle();

    List<Triangle> delaunayTriangleList1;
    List<Triangle> delaunayTriangleList2;
    List<DelaunayEdge> delaunayEdgeList1;
    List<DelaunayEdge> delaunayEdgeList2;
    List<VoronoiEdge> voronoiEdgeList1;
    List<VoronoiEdge> voronoiEdgeList2;

    List<Vector3> pList1 = new List<Vector3>();
    List<Vector3> pList2 = new List<Vector3>();

    public void Recalculate()
    {

        //Debug.Log("Calculate once");
        if (PrimaryLight.type == LightType.Spot)
        {
            Collections.allPoints1.Clear();
            Collections.minCircleSizeList1.Clear();
            Collections.edgeLengthList1.Clear();
            pointMapping1.Clear();
            pointListCopy1.Clear();
            realTriangleList1.Clear();
            pointsOnEdgeList1.Clear();

            for (int i = 0; i < pointList1.Count; i++)
                pointListCopy1.Add(new Vector3(pointList1[i].x, pointList1[i].y, i));

            pointListCopy1 = pointListCopy1.OrderBy(point => point.x).ToList();
            for (int i = 0; i < pointListCopy1.Count; i++)
                pointMapping1.Add((int)pointListCopy1[i].z);


            for (int i = 0; i < pointList1.Count; i++)
                Collections.allPoints1.Add(ToP(pointList1[i]));

            Collections.allPoints1 = Collections.allPoints1.OrderBy(point => point.x).ToList();
            DelaunayTriangulate();
            VoronoiDiagram();

            int idx = 0;
            for (int i = 0; i < delaunayTriangleList1.Count; i++)
            {
                //Debug.Log("ST");
                Triangle t = delaunayTriangleList1[i];
                if (!t.SharesVertexWith(superTriangle1))
                {
                    realTriangleList1.Add(t);
                    //Debug.Log(Tov3(Collections.allPoints[t.vertex1]));
                    //Debug.Log(Tov3(Collections.allPoints[t.vertex2]));
                    //Debug.Log(Tov3(Collections.allPoints[t.vertex3]));
                    float distance1 = Tov3(t.center - Collections.allPoints1[t.vertex1]).magnitude;
                    if (distance1 == float.NaN)
                        distance1 = Random.value - 2;
                    if (!Collections.minCircleSizeList1.ContainsKey(distance1))
                        Collections.minCircleSizeList1.Add(distance1, new CenterOrEdge(true, idx, Vector3.one, 1));
                    idx++;
                }
            }

            idx = 0;
            foreach (DelaunayEdge e in delaunayEdgeList1)
            {
                //Debug.Log(e.start + " " + e.end);
                double lng = getLength(Collections.allPoints1[e.start], Collections.allPoints1[e.end]);
                if (!Collections.edgeLengthList1.ContainsKey(lng))
                    Collections.edgeLengthList1.Add(lng, new EdgeLengthUtil(idx, 1));
                idx++;
            }

            List<VoronoiEdge> tmp = new List<VoronoiEdge>();
            foreach (VoronoiEdge v in voronoiEdgeList1)
            {

                if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 0 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 0)
                {
                    //Discard
                }
                else if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 1 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 1)
                {
                    //Keep
                    tmp.Add(v);
                }
                else
                {
                    //Intersection
                    Point o0 = ToP(Vector3.one);
                    Point o1 = o0;
                    if (Voronoi.intersect2D_SegPoly(v.start, v.end, ref polygonSides, ref o0, ref o1) == 1)
                    {
                        if (Tov3(v.start) != Tov3(o0))
                        {
                            pointsOnEdgeList1.Add(Tov3(o0));
                            v.start = o0;
                        }
                        if (Tov3(v.end) != Tov3(o1))
                        {
                            pointsOnEdgeList1.Add(Tov3(o1));
                            v.end = o1;
                        }
                    }
                    tmp.Add(v);
                }
                voronoiEdgeList1 = tmp;
            }

            foreach (Vector3 p in pointsOnEdgeList1)
            {
                SortedList<double, int> distanceList = new SortedList<double, int>();
                foreach (Vector3 pp in pointList1)
                {
                    double distance = Vector3.Distance(p, pp);
                    if (!distanceList.ContainsKey(distance))
                        distanceList.Add(distance, 0);
                }
                double shortestDistance = distanceList.Keys.First();
                if (!Collections.minCircleSizeList1.ContainsKey(shortestDistance))
                    Collections.minCircleSizeList1.Add(shortestDistance, new CenterOrEdge(false, 0, p, 1));
            }
        }
        else if (PrimaryLight.type == LightType.Point)
        {
            Collections.allPoints1.Clear();
            Collections.allPoints2.Clear();
            Collections.minCircleSizeList1.Clear();
            Collections.minCircleSizeList2.Clear();
            Collections.edgeLengthList1.Clear();
            Collections.edgeLengthList2.Clear();
            pointMapping1.Clear();
            pointMapping2.Clear();
            pointListCopy1.Clear();
            pointListCopy2.Clear();
            realTriangleList1.Clear();
            realTriangleList2.Clear();
            pointsOnEdgeList1.Clear();
            pointsOnEdgeList2.Clear();


            for (int i = 0; i < pointList1.Count; i++)
                pointListCopy1.Add(new Vector3(pointList1[i].x, pointList1[i].y, i));
            for (int i = 0; i < pointList2.Count; i++)
                pointListCopy2.Add(new Vector3(pointList2[i].x, pointList2[i].y, i));

            pointListCopy1 = pointListCopy1.OrderBy(point => point.x).ToList();
            for (int i = 0; i < pointListCopy1.Count; i++)
                pointMapping1.Add((int)pointListCopy1[i].z);
            pointListCopy2 = pointListCopy2.OrderBy(point => point.x).ToList();
            for (int i = 0; i < pointListCopy2.Count; i++)
                pointMapping2.Add((int)pointListCopy2[i].z);

            for (int i = 0; i < pointList1.Count; i++)
                Collections.allPoints1.Add(ToP(pointList1[i]));
            for (int i = 0; i < pointList2.Count; i++)
                Collections.allPoints2.Add(ToP(pointList2[i]));

            Collections.allPoints1 = Collections.allPoints1.OrderBy(point => point.x).ToList();
            Collections.allPoints2 = Collections.allPoints2.OrderBy(point => point.x).ToList();
            DelaunayTriangulate();
            VoronoiDiagram();

            int idx = 0;
            for (int i = 0; i < delaunayTriangleList1.Count; i++)
            {
                //Debug.Log("ST");
                Triangle t = delaunayTriangleList1[i];
                if (!t.SharesVertexWith(superTriangle1))
                {
                    realTriangleList1.Add(t);
                    //Debug.Log(Tov3(Collections.allPoints[t.vertex1]));
                    //Debug.Log(Tov3(Collections.allPoints[t.vertex2]));
                    //Debug.Log(Tov3(Collections.allPoints[t.vertex3]));
                    float distance1 = Tov3(t.center - Collections.allPoints1[t.vertex1]).magnitude;
                    if (distance1 == float.NaN)
                        distance1 = Random.value - 2;
                    if (!Collections.minCircleSizeList1.ContainsKey(distance1))
                        Collections.minCircleSizeList1.Add(distance1, new CenterOrEdge(true, idx, Vector3.one, 1));
                    idx++;
                }
            }
            idx = 0;
            for (int i = 0; i < delaunayTriangleList2.Count; i++)
            {
                //Debug.Log("ST");
                Triangle t = delaunayTriangleList2[i];
                if (!t.SharesVertexWith(superTriangle2))
                {
                    realTriangleList2.Add(t);
                    float distance1 = Tov3(t.center - Collections.allPoints2[t.vertex1]).magnitude;
                    if (distance1 == float.NaN)
                        distance1 = Random.value - 2;
                    if (!Collections.minCircleSizeList2.ContainsKey(distance1))
                        Collections.minCircleSizeList2.Add(distance1, new CenterOrEdge(true, idx, Vector3.one, 2));
                    idx++;
                }
            }

            idx = 0;
            foreach (DelaunayEdge e in delaunayEdgeList1)
            {
                //Debug.Log(e.start + " " + e.end);
                double lng = getLength(Collections.allPoints1[e.start], Collections.allPoints1[e.end]);
                if (!Collections.edgeLengthList1.ContainsKey(lng))
                    Collections.edgeLengthList1.Add(lng, new EdgeLengthUtil(idx, 1));
                idx++;
            }
            idx = 0;
            foreach (DelaunayEdge e in delaunayEdgeList2)
            {
                //Debug.Log(e.start + " " + e.end);
                double lng = getLength(Collections.allPoints2[e.start], Collections.allPoints2[e.end]);
                if (!Collections.edgeLengthList2.ContainsKey(lng))
                    Collections.edgeLengthList2.Add(lng, new EdgeLengthUtil(idx, 2));
                idx++;
            }

            List<VoronoiEdge> tmp = new List<VoronoiEdge>();
            foreach (VoronoiEdge v in voronoiEdgeList1)
            {

                if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 0 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 0)
                {
                    //Discard
                }
                else if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 1 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 1)
                {
                    //Keep
                    tmp.Add(v);
                }
                else
                {
                    //Intersection
                    Point o0 = ToP(Vector3.one);
                    Point o1 = o0;
                    if (Voronoi.intersect2D_SegPoly(v.start, v.end, ref polygonSides, ref o0, ref o1) == 1)
                    {
                        if (Tov3(v.start) != Tov3(o0))
                        {
                            pointsOnEdgeList1.Add(Tov3(o0));
                            v.start = o0;
                        }
                        if (Tov3(v.end) != Tov3(o1))
                        {
                            pointsOnEdgeList1.Add(Tov3(o1));
                            v.end = o1;
                        }
                    }
                    tmp.Add(v);
                }
                voronoiEdgeList1 = tmp;
            }
            tmp.Clear();
            foreach (VoronoiEdge v in voronoiEdgeList2)
            {

                if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 0 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 0)
                {
                    //Discard
                }
                else if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 1 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 1)
                {
                    //Keep
                    tmp.Add(v);
                }
                else
                {
                    //Intersection
                    Point o0 = ToP(Vector3.one);
                    Point o1 = o0;
                    if (Voronoi.intersect2D_SegPoly(v.start, v.end, ref polygonSides, ref o0, ref o1) == 1)
                    {
                        if (Tov3(v.start) != Tov3(o0))
                        {
                            pointsOnEdgeList2.Add(Tov3(o0));
                            v.start = o0;
                        }
                        if (Tov3(v.end) != Tov3(o1))
                        {
                            pointsOnEdgeList2.Add(Tov3(o1));
                            v.end = o1;
                        }
                    }
                    tmp.Add(v);
                }
                voronoiEdgeList2 = tmp;
            }

            foreach (Vector3 p in pointsOnEdgeList1)
            {
                SortedList<double, int> distanceList = new SortedList<double, int>();
                foreach (Vector3 pp in pointList1)
                {
                    double distance = Vector3.Distance(p, pp);
                    if (!distanceList.ContainsKey(distance))
                        distanceList.Add(distance, 0);
                }
                double shortestDistance = distanceList.Keys.First();
                if (!Collections.minCircleSizeList1.ContainsKey(shortestDistance))
                    Collections.minCircleSizeList1.Add(shortestDistance, new CenterOrEdge(false, 0, p, 1));
            }
            foreach (Vector3 p in pointsOnEdgeList2)
            {
                SortedList<double, int> distanceList = new SortedList<double, int>();
                foreach (Vector3 pp in pointList2)
                {
                    double distance = Vector3.Distance(p, pp);
                    if (!distanceList.ContainsKey(distance))
                        distanceList.Add(distance, 0);
                }
                double shortestDistance = distanceList.Keys.First();
                if (!Collections.minCircleSizeList2.ContainsKey(shortestDistance))
                    Collections.minCircleSizeList2.Add(shortestDistance, new CenterOrEdge(false, 0, p, 2));
            }
        }
    }
    public void DelaunayTriangulate()
    {
        superTriangle1 = Delaunay.SuperTriangle(Collections.allPoints1, 1);
        delaunayTriangleList1 = Delaunay.Triangulate(superTriangle1, Collections.allPoints1, 1);
        delaunayEdgeList1 = Delaunay.DelaunayEdges(superTriangle1, delaunayTriangleList1);
        if (PrimaryLight.type == LightType.Point)
        {
            //Debug.Log(Collections.allPoints2.Count);
            superTriangle2 = Delaunay.SuperTriangle(Collections.allPoints2, 2);
            delaunayTriangleList2 = Delaunay.Triangulate(superTriangle2, Collections.allPoints2, 2);
            delaunayEdgeList2 = Delaunay.DelaunayEdges(superTriangle2, delaunayTriangleList2);
        }
    }

    public void VoronoiDiagram()
    {
        voronoiEdgeList1 = Voronoi.VoronoiEdges(delaunayEdgeList1, 1);
        if (PrimaryLight.type == LightType.Point)
        {
            voronoiEdgeList2 = Voronoi.VoronoiEdges(delaunayEdgeList2, 2);
        }
    }

    float halton(int index, int b)
    {
        float result = 0;
        float f = 1.0f / (float)b;
        float i = index;
        while (i > 0)
        {
            result = result + f * (i % b);
            i = Mathf.Floor(i / b);
            f = f / b;
        }
        return result;
    }

    void haltonGenerator(ref List<Point> points, int numpts, int basex = 0, int basey = 0)
    {
        // 2, 3 Halton Sequence by default
        if (basex == 0)
            basex = 2;
        if (basey == 0)
            basey = 3;
        int index = 20;
        for (var i = 0; i < numpts; i++)
        {
            Point p = new Point(halton(index, basex) * 2 - 1, halton(index, basey) * 2 - 1);
            Debug.Log(Tov3(p));
            if (Vector3.Distance(Tov3(p), Vector3.zero) < 1)
                points.Add(p);
            else i--;
            index++;
        }
    }

    void Start()
    {
        //圓形clipping
        int sideCount = 20;

        float perAngle = 360 / sideCount * Mathf.Deg2Rad;
        float nowAngle = 90 * Mathf.Deg2Rad;
        for (int i = 0; i < sideCount + 1; i++)
        {
            float x = Mathf.Cos(nowAngle) * 1.0128f;// * 1.4142135623f;
            float y = Mathf.Sin(nowAngle) * 1.0128f;// * 1.4142135623f;
            nowAngle += perAngle;
            polygonSides.Add(new Point(x, y));
        }
        Voronoi.setPolygonSides(polygonSides);

        Collections.minCircleSizeList1 = new SortedList<double, CenterOrEdge>();
        Collections.minCircleSizeList2 = new SortedList<double, CenterOrEdge>();
        Collections.edgeLengthList1 = new SortedList<double, EdgeLengthUtil>();
        Collections.edgeLengthList2 = new SortedList<double, EdgeLengthUtil>();
        Collections.allPoints1 = new List<Point>();
        Collections.allPoints2 = new List<Point>();


        if (PrimaryLight.type == LightType.Spot)
        {
            List<Point> points = new List<Point>();
            haltonGenerator(ref points, VPLCount);
            for (int i = 0; i < points.Count; i++)
            {
                GetHitPointFromPrimaryLight_Quasi(out RaycastHit hit, ref pointList1, 1, points[i]);
                GameObject VPLLight = new GameObject("VPL");
                Light lightComp = VPLLight.AddComponent<Light>();

                VPLLight.transform.position = hit.point + hit.normal.normalized * 0.0001f;
                lightComp.type = LightType.Point;
                lightComp.range = 1000;
                lightComp.intensity = 10.0f / VPLCount;
                lightComp.shadows = LightShadows.Soft;
                lightComp.shadowResolution = UnityEngine.Rendering.LightShadowResolution.Low;
                VPLLight.transform.SetParent(VPLCollector.transform);
            }
            //for (int i = 0; i < VPLCount; i++)
            //{
            //    GetHitPointFromPrimaryLight(out RaycastHit hit, ref pointList1,1);

            //    GameObject VPLLight = new GameObject("VPL");
            //    Light lightComp = VPLLight.AddComponent<Light>();

            //    VPLLight.transform.position = hit.point + hit.normal.normalized * 0.0001f;
            //    lightComp.type = LightType.Point;
            //    lightComp.range = 1000;
            //    lightComp.intensity = 10.0f / VPLCount;
            //    lightComp.shadows = LightShadows.Soft;
            //    lightComp.shadowResolution = UnityEngine.Rendering.LightShadowResolution.Low;
            //    VPLLight.transform.SetParent(VPLCollector.transform);

            //}
        }
        else if (PrimaryLight.type == LightType.Point)
        {
            for (int i = 0; i < VPLCount / 2; i++)
            {
                GetHitPointFromPrimaryLight(out RaycastHit hit, ref pointList1,1);

                GameObject VPLLight = new GameObject("VPL");
                Light lightComp = VPLLight.AddComponent<Light>();

                VPLLight.transform.position = hit.point + hit.normal.normalized * 0.0001f;
                lightComp.type = LightType.Point;
                lightComp.range = 1000;
                lightComp.intensity = 10.0f / VPLCount;
                lightComp.shadows = LightShadows.Soft;
                lightComp.shadowResolution = UnityEngine.Rendering.LightShadowResolution.Low;
                VPLLight.transform.SetParent(VPLCollector.transform);

            }
            for (int i = VPLCount / 2; i < VPLCount; i++)
            {
                GetHitPointFromPrimaryLight(out RaycastHit hit, ref pointList2,2);

                GameObject VPLLight = new GameObject("VPL");
                Light lightComp = VPLLight.AddComponent<Light>();

                VPLLight.transform.position = hit.point + hit.normal.normalized * 0.0001f;
                lightComp.type = LightType.Point;
                lightComp.range = 1000;
                lightComp.intensity = 10.0f / VPLCount;
                lightComp.shadows = LightShadows.Soft;
                lightComp.shadowResolution = UnityEngine.Rendering.LightShadowResolution.Low;
                VPLLight.transform.SetParent(VPLCollector.transform);

            }
        }
        Recalculate();

    }

    Point ToP(Vector3 v)
    {
        return new Point(v.x, v.y, v.z);
    }

    Vector3 Tov3(Point p)
    {
        return new Vector3((float)p.x, (float)p.y, (float)p.z);
    }



    public void test()
    {

    }

    public void Update()
    {
        if(PrimaryLight.type == LightType.Spot)
        PrimaryLight.transform.Rotate(0, 0.2f, 0);

        if (true)
        {
            Light[] transforms = VPLCollector.GetComponentsInChildren<Light>();

            pList1.Clear();
            pList2.Clear();
            List<int> deletedIdxList = new List<int>();
            for (int i = 0; i < transforms.Length; i++)
            {
                if (PrimaryLight.type == LightType.Spot || PrimaryLight.type == LightType.Point)
                {
                    //檢查VPL是否不可見
                    Vector3 dirToVPL = Vector3.Normalize(transforms[i].transform.position - PrimaryLight.transform.position);
                    float rayLength = Vector3.Magnitude(transforms[i].transform.position - PrimaryLight.transform.position) - 0.0001f;
                    bool rayHit = false;

                    //光源射向VPL
                    Ray ray = new Ray(PrimaryLight.transform.position, dirToVPL);
                    rayHit = Physics.Raycast(ray, out RaycastHit gomi, rayLength);

                    //VPL射向光源
                    ray.origin = transforms[i].transform.position;
                    ray.direction = -dirToVPL;
                    rayHit = Physics.Raycast(ray, out gomi, rayLength);

                    if (rayHit)
                    {
                        deletedIdxList.Add(i);
                        //Debug.LogWarning("Blocked");
                        continue;
                    }

                    if (PrimaryLight.type == LightType.Spot)
                    {
                        //檢查是否超出Spotlight
                        Vector3 pointDir = Vector3.Normalize(transforms[i].transform.position - PrimaryLight.transform.position);
                        float angle = Mathf.Acos(Vector3.Dot(pointDir, PrimaryLight.transform.forward)) * Mathf.Rad2Deg;
                        if (angle > PrimaryLight.spotAngle / 2.0f-1f)
                        {
                            //Debug.LogWarning("Angle");
                            deletedIdxList.Add(i);
                        }
                    }
                }
            }
            if (PrimaryLight.type == LightType.Spot)
            {
                float radius = Mathf.Tan(Mathf.Deg2Rad * PrimaryLight.spotAngle / 2) * PrimaryLight.range;
                int tmpIdx = 0;
                for (int i = 0; i < transforms.Length; i++)
                {
                    int deleteIdx;
                    if (deletedIdxList.Count == 0)
                        deleteIdx = -1;
                    else
                        deleteIdx = deletedIdxList[tmpIdx];
                    if (i != deleteIdx)
                    {
                        Vector3 n = PrimaryLight.transform.forward;
                        Vector3 p0 = PrimaryLight.transform.position + PrimaryLight.transform.forward * PrimaryLight.range;
                        Vector3 l0 = PrimaryLight.transform.position;
                        Vector3 l = Vector3.Normalize(transforms[i].transform.position - PrimaryLight.transform.position);
                        float t = 0;
                        intersectPlane(ref n, ref p0, ref l0, ref l, ref t);
                        Vector3 p = l0 + l * t;
                        p = PrimaryLight.transform.InverseTransformPoint(p) / radius;
                        p.x /= Mathf.Cos(Vector3.Dot(n, l));
                        p.y /= Mathf.Cos(Vector3.Dot(n, l));
                        p.z = 0;
                        pList1.Add(p);
                    }
                    else if (i == deleteIdx)
                    {
                        if (tmpIdx + 1 < deletedIdxList.Count)
                            tmpIdx++;
                    }
                }
            }
            else if(PrimaryLight.type == LightType.Point)
            {
                float radius = Mathf.Tan(Mathf.Deg2Rad * 179.9f / 2) * PrimaryLight.range;
                int tmpIdx = 0;
                for (int i = 0; i < transforms.Length; i++)
                {
                    int deleteIdx;
                    if (deletedIdxList.Count == 0)
                        deleteIdx = -1;
                    else
                        deleteIdx = deletedIdxList[tmpIdx];
                    if (i != deleteIdx)
                    {
                        Vector3 n = PrimaryLight.transform.forward;
                        //Debug.Log(n);
                        Vector3 l = Vector3.Normalize(transforms[i].transform.position - PrimaryLight.transform.position);
                        //Debug.Log(l);
                        if (Vector3.Dot(n, l) >= 0)
                        {
                            //Debug.Log(Vector3.Dot(n, l)+ " "+1);
                            Vector3 p0 = PrimaryLight.transform.position + PrimaryLight.transform.forward * PrimaryLight.range;
                            Vector3 l0 = PrimaryLight.transform.position;

                            float t = 0;
                            intersectPlane(ref n, ref p0, ref l0, ref l, ref t);
                            Vector3 p = l0 + l * t;
                            p = PrimaryLight.transform.InverseTransformPoint(p) / radius;
                            p.x /= Mathf.Cos(Vector3.Dot(n, l));
                            p.y /= Mathf.Cos(Vector3.Dot(n, l));
                            p.z = 0;
                            pList1.Add(p);
                        }
                        else if (Vector3.Dot(n, l) < 0)
                        {
                            //Debug.Log(Vector3.Dot(n, l) + " " + 2);
                            n = -n;
                            Vector3 p0 = PrimaryLight.transform.position - PrimaryLight.transform.forward * PrimaryLight.range;
                            Vector3 l0 = PrimaryLight.transform.position;

                            float t = 0;
                            intersectPlane(ref n, ref p0, ref l0, ref l, ref t);
                            Vector3 p = l0 + l * t;
                            p = PrimaryLight.transform.InverseTransformPoint(p) / radius;
                            p.x /= Mathf.Cos(Vector3.Dot(n, l));
                            p.y /= Mathf.Cos(Vector3.Dot(n, l));
                            p.z = 0;
                            pList2.Add(p);
                        }
                    }
                    else if (i == deleteIdx)
                    {
                        if (tmpIdx + 1 < deletedIdxList.Count)
                            tmpIdx++;
                    }
                }
            }


            pointList1.Clear();
            for (int i = 0; i < pList1.Count; i++)
                pointList1.Add(pList1[i]);
            if (PrimaryLight.type == LightType.Point)
            {
                pointList2.Clear();
                for (int i = 0; i < pList2.Count; i++)
                    pointList2.Add(pList2[i]);
            }

            Recalculate();

            //透過Voronoi刪除新增
            int remainVPLToDelete = minimalDeletedVPLPerFrame - deletedIdxList.Count;

            if (remainVPLToDelete < 0)
            {
                remainVPLToDelete = 0;
            }

            //取出最小距離VPL 刪除
            while (remainVPLToDelete > 0)
            {
                if (PrimaryLight.type == LightType.Spot)
                {
                    Debug.Log("222");
                    EdgeLengthUtil minEdgeIdx = Collections.edgeLengthList1.Values.First();

                    int pointA = pointMapping1[delaunayEdgeList1[minEdgeIdx.idx].start];
                    int pointB = pointMapping1[delaunayEdgeList1[minEdgeIdx.idx].end];

                    float minDis = float.MaxValue;
                    bool flag = true;
                    for (int i = 0; i < delaunayEdgeList1.Count; i++)
                    {
                        if (i != minEdgeIdx.idx)
                        {
                            DelaunayEdge dE = delaunayEdgeList1[i];
                            if (pointA == dE.start || pointA == dE.end)
                            {
                                float distance = Vector3.Distance(Tov3(Collections.allPoints1[dE.start]), Tov3(Collections.allPoints1[dE.end]));
                                if (distance < minDis)
                                {
                                    minDis = distance;
                                    flag = true;
                                }
                            }

                            if (pointB == dE.start || pointB == dE.end)
                            {
                                float distance = Vector3.Distance(Tov3(Collections.allPoints1[dE.start]), Tov3(Collections.allPoints1[dE.end]));
                                if (distance < minDis)
                                {
                                    minDis = distance;
                                    flag = false;
                                }
                            }
                        }
                    }

                    int deleteIndex = flag ? (int)pointA : (int)pointB;
                    deletedIdxList.Add(deleteIndex);
                    pointList1.RemoveAt(deleteIndex);

                    Recalculate();
                    remainVPLToDelete--;
                }
                else if (PrimaryLight.type == LightType.Point)
                {

                    //Debug.Log("222");

                    float minDis = float.MaxValue;
                    bool flag = true;
                    EdgeLengthUtil minEdgeIdx1 = Collections.edgeLengthList1.Values.First();

                    int pointA = pointMapping1[delaunayEdgeList1[minEdgeIdx1.idx].start];
                    int pointB = pointMapping1[delaunayEdgeList1[minEdgeIdx1.idx].end];
                    for (int i = 0; i < delaunayEdgeList1.Count; i++)
                    {
                        if (i != minEdgeIdx1.idx)
                        {
                            DelaunayEdge dE = delaunayEdgeList1[i];
                            if (pointA == dE.start || pointA == dE.end)
                            {
                                float distance = Vector3.Distance(Tov3(Collections.allPoints1[dE.start]), Tov3(Collections.allPoints1[dE.end]));
                                if (distance < minDis)
                                {
                                    minDis = distance;
                                    flag = true;
                                }
                            }

                            if (pointB == dE.start || pointB == dE.end)
                            {
                                float distance = Vector3.Distance(Tov3(Collections.allPoints1[dE.start]), Tov3(Collections.allPoints1[dE.end]));
                                if (distance < minDis)
                                {
                                    minDis = distance;
                                    flag = false;
                                }
                            }
                        }
                    }
                    int deleteIndex = flag ? (int)pointA : (int)pointB;
                    deletedIdxList.Add(deleteIndex);
                    pointList1.RemoveAt(deleteIndex);

                    Recalculate();
                    remainVPLToDelete--;

                    EdgeLengthUtil minEdgeIdx2 = Collections.edgeLengthList2.Values.First();

                    pointA = pointMapping2[delaunayEdgeList2[minEdgeIdx2.idx].start];
                    pointB = pointMapping2[delaunayEdgeList2[minEdgeIdx2.idx].end];
                    for (int i = 0; i < delaunayEdgeList2.Count; i++)
                    {
                        if (i != minEdgeIdx2.idx)
                        {
                            DelaunayEdge dE = delaunayEdgeList2[i];
                            if (pointA == dE.start || pointA == dE.end)
                            {
                                float distance = Vector3.Distance(Tov3(Collections.allPoints2[dE.start]), Tov3(Collections.allPoints2[dE.end]));
                                if (distance < minDis)
                                {
                                    minDis = distance;
                                    flag = true;
                                }
                            }

                            if (pointB == dE.start || pointB == dE.end)
                            {
                                float distance = Vector3.Distance(Tov3(Collections.allPoints2[dE.start]), Tov3(Collections.allPoints2[dE.end]));
                                if (distance < minDis)
                                {
                                    minDis = distance;
                                    flag = false;
                                }
                            }
                        }
                    }
                    deleteIndex = flag ? (int)pointA : (int)pointB;
                    deletedIdxList.Add(deleteIndex);
                    pointList2.RemoveAt(deleteIndex);

                    Recalculate();
                    remainVPLToDelete--;
                }
            }

            int modifyIdx = 0;
            int addCount = deletedIdxList.Count > minimalDeletedVPLPerFrame ? deletedIdxList.Count : minimalDeletedVPLPerFrame;
            //Debug.Log("AddCount " + addCount);
            //取出最大面積Delaunay 新增
            while (addCount > 0)
            {
                Point pos= new Point(0,0,0);
                if (PrimaryLight.type == LightType.Spot)
                {
                    CenterOrEdge coe = Collections.minCircleSizeList1.Values.Last();
                    if (coe.isCenter)
                    {
                        int minCircleIdx = coe.idx;

                        pos = realTriangleList1[minCircleIdx].center;

                        Point center = pos;

                        //Debug.Log(Tov3(center));
                        int endPointInsideOutside = Voronoi.cn_PnPoly(center, ref polygonSides);

                        if (endPointInsideOutside == 0)
                        {
                            Point st = ToP(Tov3(Collections.allPoints1[realTriangleList1[minCircleIdx].vertex1] + Collections.allPoints1[realTriangleList1[minCircleIdx].vertex2] + Collections.allPoints1[realTriangleList1[minCircleIdx].vertex3]) / 3.0f);
                            Point ed = center;

                            Point o0 = ToP(Vector3.one);
                            Point o1 = o0;
                            if (Voronoi.intersect2D_SegPoly(st, ed, ref polygonSides, ref o0, ref o1) == 1)
                            {
                                //Debug.Log("0"+ Tov3(o1));
                                if (Tov3(ed) != Tov3(o1))
                                    ed = o1;
                            }
                            center = ed;

                            //Debug.Log("Outside");
                        }
                        pos = center;
                        //Debug.Log("1" + Tov3(pos));
                    }
                    else if (!coe.isCenter)
                    {
                        pos = ToP(coe.edgePos);
                        //Debug.Log("2" + Tov3(pos));
                    }

                    bool rayHit = GetHitPointFromRay(Tov3(pos), out RaycastHit hit);
                    if (rayHit)
                    {
                        pointList1.Add(Tov3(pos));
                        //Debug.Log(Tov3(pos));
                        transforms[deletedIdxList[modifyIdx]].transform.position = hit.point + hit.normal.normalized * 0.0001f;
                    }
                    else
                    {
                        GetHitPointFromPrimaryLight(out RaycastHit hitRand, ref pointList1, 1);
                        transforms[deletedIdxList[modifyIdx]].transform.position = hitRand.point + hitRand.normal.normalized * 0.0001f;
                    }

                    Recalculate();
                    modifyIdx++;
                    addCount--;
                }
                else
                {
                    CenterOrEdge coe1 = Collections.minCircleSizeList1.Values.Last();

                    if (coe1.isCenter)
                    {
                        int minCircleIdx = coe1.idx;

                        pos = realTriangleList1[minCircleIdx].center;

                        Point center = pos;

                        //Debug.Log(Tov3(center));
                        int endPointInsideOutside = Voronoi.cn_PnPoly(center, ref polygonSides);

                        if (endPointInsideOutside == 0)
                        {
                            Point st = ToP(Tov3(Collections.allPoints1[realTriangleList1[minCircleIdx].vertex1] + Collections.allPoints1[realTriangleList1[minCircleIdx].vertex2] + Collections.allPoints1[realTriangleList1[minCircleIdx].vertex3]) / 3.0f);
                            Point ed = center;

                            Point o0 = ToP(Vector3.one);
                            Point o1 = o0;
                            if (Voronoi.intersect2D_SegPoly(st, ed, ref polygonSides, ref o0, ref o1) == 1)
                            {
                                //Debug.Log("0"+ Tov3(o1));
                                if (Tov3(ed) != Tov3(o1))
                                    ed = o1;
                            }
                            center = ed;

                            //Debug.Log("Outside");
                        }
                        pos = center;
                        //Debug.Log("1" + Tov3(pos));
                    }
                    else if (!coe1.isCenter)
                    {
                        pos = ToP(coe1.edgePos);
                        //Debug.Log("2" + Tov3(pos));
                    }

                    bool rayHit = GetHitPointFromRay(Tov3(pos), out RaycastHit hit);
                    if (rayHit)
                    {
                        pointList1.Add(Tov3(pos));
                        //Debug.Log(Tov3(pos));
                        transforms[deletedIdxList[modifyIdx]].transform.position = hit.point + hit.normal.normalized * 0.0001f;
                    }
                    else
                    {
                        GetHitPointFromPrimaryLight(out RaycastHit hitRand, ref pointList1, 1);
                        transforms[deletedIdxList[modifyIdx]].transform.position = hitRand.point + hitRand.normal.normalized * 0.0001f;
                    }

                    Recalculate();
                    modifyIdx++;
                    addCount--;
                    if (modifyIdx >= deletedIdxList.Count)
                        break;

                    CenterOrEdge coe2 = Collections.minCircleSizeList2.Values.Last();

                    if (coe2.isCenter)
                    {
                        int minCircleIdx = coe2.idx;

                        pos = realTriangleList2[minCircleIdx].center;

                        Point center = pos;

                        int endPointInsideOutside = Voronoi.cn_PnPoly(center, ref polygonSides);

                        if (endPointInsideOutside == 0)
                        {
                            Point st = ToP(Tov3(Collections.allPoints2[realTriangleList2[minCircleIdx].vertex1] + Collections.allPoints2[realTriangleList2[minCircleIdx].vertex2] + Collections.allPoints2[realTriangleList2[minCircleIdx].vertex3]) / 3.0f);
                            Point ed = center;

                            Point o0 = ToP(Vector3.one);
                            Point o1 = o0;
                            if (Voronoi.intersect2D_SegPoly(st, ed, ref polygonSides, ref o0, ref o1) == 1)
                            {
                                if (Tov3(ed) != Tov3(o1))
                                    ed = o1;
                            }
                            center = ed;

                            //Debug.Log("Outside");
                        }
                        pos = center;
                    }
                    else if (!coe2.isCenter)
                    {
                        pos = ToP(coe2.edgePos);
                    }

                    rayHit = GetHitPointFromRay(Tov3(pos), out hit);
                    if (rayHit)
                    {
                        pointList2.Add(Tov3(pos));
                        transforms[deletedIdxList[modifyIdx]].transform.position = hit.point + hit.normal.normalized * 0.0001f;
                    }
                    else
                    {
                        GetHitPointFromPrimaryLight(out RaycastHit hitRand, ref pointList2, 2);
                        transforms[deletedIdxList[modifyIdx]].transform.position = hitRand.point + hitRand.normal.normalized * 0.0001f;
                    }

                    Recalculate();
                    modifyIdx++;
                    addCount--;

                }
            }


            //List<double> ratio = Voronoi.calculateAreas();
            //for (int i = 0; i < transforms.Length; i++)
            //{
            //    transforms[i].intensity = 10.0f * (float)ratio[i];
            //}
        }
    }


    bool GetHitPointFromRay(Vector3 dir, out RaycastHit hit)
    {
        hit = new RaycastHit();
        float radius = Mathf.Tan(Mathf.Deg2Rad * PrimaryLight.spotAngle / 2) * PrimaryLight.range;
        Vector3 target = PrimaryLight.transform.forward * PrimaryLight.range + PrimaryLight.transform.rotation * dir * radius;
        Ray ray = new Ray(PrimaryLight.transform.position, target - PrimaryLight.transform.position);
        return Physics.Raycast(ray, out hit, 100);

    }
    void GetHitPointFromPrimaryLight_Quasi(out RaycastHit hit, ref List<Vector3> pointList, int which,Point p)
    {
        hit = new RaycastHit();
        bool rayHit = false;
        float radius = Mathf.Tan(Mathf.Deg2Rad * PrimaryLight.spotAngle / 2) * PrimaryLight.range;
        Vector2 circle = Tov3(p) * radius;
        Vector2 bias = Random.insideUnitCircle * new Vector2(0.005f, 0.005f);
        bool first = true;
        while (rayHit == false)
        {
            if (first)
                first = false;
            else
                circle += bias;

            Vector3 dir = Vector3.one;
            dir = PrimaryLight.transform.forward * PrimaryLight.range + PrimaryLight.transform.rotation * new Vector3(circle.x, circle.y);

            Ray ray = new Ray(PrimaryLight.transform.position, dir - PrimaryLight.transform.position + new Vector3(0.001f, 0.001f, 0.001f));
            rayHit = Physics.Raycast(ray, out hit, 100);

        }
        pointList.Add(circle);
    }

    void GetHitPointFromPrimaryLight(out RaycastHit hit,ref List<Vector3> pointList,int which)
    {
        hit = new RaycastHit();
        bool rayHit = false;
        Vector3 unitCirclePos = Vector3.one;
        while (rayHit == false)
        {
            Vector3 dir = Vector3.one;

            dir = GetDirectionFromSpotlight(ref unitCirclePos,which);

            Ray ray = new Ray(PrimaryLight.transform.position, dir - PrimaryLight.transform.position + new Vector3(0.001f, 0.001f, 0.001f));
            rayHit = Physics.Raycast(ray, out hit, 100);

        }
        pointList.Add(unitCirclePos);
    }

    bool intersectPlane(ref Vector3 n,ref Vector3 p0,ref Vector3 l0,ref Vector3 l,ref float t)
    {
        // assuming vectors are all normalized
        float denom = Vector3.Dot(n, l);
        if (denom > 1e-6) {
            Vector3 p0l0 = p0 - l0;
            t = Vector3.Dot(p0l0, n) / denom;
            return (t >= 0);
        }

        return false;
    }

    Vector3 GetDirectionFromSpotlight(ref Vector3 unitCirclePos,int which)
    {
        if (PrimaryLight.type == LightType.Spot)
        {
            float radius = Mathf.Tan(Mathf.Deg2Rad * PrimaryLight.spotAngle / 2) * PrimaryLight.range;
            Vector2 unitCircle = NonUniformInsideUnitCircle();
            Vector2 circle = unitCircle * radius;

            unitCirclePos = unitCircle;

            Vector3 target = PrimaryLight.transform.forward * PrimaryLight.range + PrimaryLight.transform.rotation * new Vector3(circle.x, circle.y);
            return target;
        }
        else
        {
            if (which == 1)
            {
                float radius = Mathf.Tan(Mathf.Deg2Rad * 179.9f / 2) * PrimaryLight.range;
                Vector2 unitCircle = NonUniformInsideUnitCircle();
                Vector2 circle = unitCircle * radius;

                unitCirclePos = unitCircle;

                Vector3 target = PrimaryLight.transform.forward * PrimaryLight.range + PrimaryLight.transform.rotation * new Vector3(circle.x, circle.y);
                return target;
            }
            else
            {
                float radius = Mathf.Tan(Mathf.Deg2Rad * 179.9f / 2) * PrimaryLight.range;
                Vector2 unitCircle = NonUniformInsideUnitCircle();
                Vector2 circle = unitCircle * radius;

                unitCirclePos = unitCircle;

                Vector3 target = -PrimaryLight.transform.forward * PrimaryLight.range + PrimaryLight.transform.rotation * new Vector3(circle.x, circle.y);
                return target;
            }
        }
    }

    Vector2 NonUniformInsideUnitCircle()
    {
        float angle = Random.value * Mathf.PI * 2f;
        float radius = Random.value;
        Point st = new Point(0, 0);
        Point ed = new Point(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
        int startPointInsideOutside = Voronoi.cn_PnPoly(st, ref polygonSides);
        int endPointInsideOutside = Voronoi.cn_PnPoly(ed, ref polygonSides);

        if (startPointInsideOutside == 0 || endPointInsideOutside == 0)
        {
            Point o0 = ToP(Vector3.one);
            Point o1 = o0;
            if (Voronoi.intersect2D_SegPoly(st, ed, ref polygonSides, ref o0, ref o1) == 1)
            {
                if (Tov3(st) != Tov3(o0))
                    st = o0;
                if (Tov3(ed) != Tov3(o1))
                    ed = o1;
            }
        }
        return Tov3(ed);
    }

    void OnDrawGizmos()
    {
        //Gizmos.color = Color.yellow;
        //for (int i=0;i<pList.Count;i++)
        //    Gizmos.DrawSphere(pList[i], 0.02f);

        Vector3 leftsideBias = new Vector3(-1.5f, 0, 0);
        Vector3 rightsideBias = new Vector3(1.5f, 0, 0);

        if (polygonSides.Count != 0)
            for (int i = 0; i < polygonSides.Count - 1; i++)
                Gizmos.DrawLine(Tov3(polygonSides[i]) + leftsideBias, Tov3(polygonSides[i + 1]) + leftsideBias);
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(new Vector3(0, 0, 0) + leftsideBias, 1);
        if (voronoiEdgeList1!=null)
        foreach (VoronoiEdge v in voronoiEdgeList1)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(new Vector3((float)v.start.x, (float)v.start.y, (float)v.start.z) + leftsideBias,
                new Vector3((float)v.end.x, (float)v.end.y, (float)v.end.z) + leftsideBias);
            //Debug.Log(v.start.x + " " + v.start.y + " " + v.end.x + " " + v.end.y);
        }
        if (delaunayEdgeList1 != null)
            foreach (DelaunayEdge d in delaunayEdgeList1)
            {
                Vector3 st = pointList1[pointMapping1[d.start]];
                Vector3 en = pointList1[pointMapping1[d.end]];
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(new Vector3((float)st.x, (float)st.y, (float)st.z) + leftsideBias,
                    new Vector3((float)en.x, (float)en.y, (float)en.z) + leftsideBias);
            }
        if(pointList1 != null)
            foreach (Vector3 p in pointList1)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(p + leftsideBias, 0.01f);
            }

        if (PrimaryLight.type == LightType.Point)
        {
            if (polygonSides.Count != 0)
                for (int i = 0; i < polygonSides.Count - 1; i++)
                    Gizmos.DrawLine(Tov3(polygonSides[i]) + rightsideBias, Tov3(polygonSides[i + 1]) + rightsideBias);
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(new Vector3(0, 0, 0) + rightsideBias, 1);
            if (voronoiEdgeList2 != null)
                foreach (VoronoiEdge v in voronoiEdgeList2)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(new Vector3((float)v.start.x, (float)v.start.y, (float)v.start.z) + rightsideBias,
                        new Vector3((float)v.end.x, (float)v.end.y, (float)v.end.z) + rightsideBias);
                }
            if (delaunayEdgeList2 != null)
                foreach (DelaunayEdge d in delaunayEdgeList2)
                {
                    Vector3 st = pointList2[pointMapping2[d.start]];
                    Vector3 en = pointList2[pointMapping2[d.end]];
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(new Vector3((float)st.x, (float)st.y, (float)st.z) + rightsideBias,
                        new Vector3((float)en.x, (float)en.y, (float)en.z) + rightsideBias);
                }
            if (pointList2 != null)
                foreach (Vector3 p in pointList2)
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(p + rightsideBias, 0.01f);
                }
        }
    }
}
