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

    List<Vector3> pointList = new List<Vector3>();

    List<Vector3> pointListCopy = new List<Vector3>();

    List<int> pointMapping = new List<int>();

    List<Triangle> realTriangleList = new List<Triangle>();

    //List<Vector3> transformGraphVector = new List<Vector3>();

    double getLength(Point a,Point b)
    {
        return Mathf.Sqrt((float)((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
    }

    Triangle superTriangle = new Triangle();

    List<Triangle> delaunayTriangleList;
    List<DelaunayEdge> delaunayEdgeList;
    List<VoronoiEdge> voronoiEdgeList;

    List<Vector3> pList = new List<Vector3>();
    public void Recalculate()
    {
        //Debug.Log("Calculate once");
        Collections.allPoints.Clear();
        Collections.areaSizeList.Clear();
        Collections.edgeLengthList.Clear();
        pointMapping.Clear();
        pointListCopy.Clear();
        realTriangleList.Clear();

        for (int i = 0; i < pointList.Count; i++)
            pointListCopy.Add(new Vector3(pointList[i].x, pointList[i].y, i));

        pointListCopy = pointListCopy.OrderBy(point => point.x).ToList();
        for (int i = 0; i < pointListCopy.Count; i++) 
            pointMapping.Add((int)pointListCopy[i].z);


        for (int i = 0; i < pointList.Count; i++)
            Collections.allPoints.Add(ToP(pointList[i]));

        Collections.allPoints = Collections.allPoints.OrderBy(point => point.x).ToList();
        DelaunayTriangulate();
        VoronoiDiagram();


        //Debug.LogWarning(delaunayTriangleList.Count);
        int idx = 0;
        for (int i = 0; i < delaunayTriangleList.Count; i++) 
        {
            //Debug.Log("ST");
            Triangle t = delaunayTriangleList[i];
            if (!t.SharesVertexWith(superTriangle))
            {
                realTriangleList.Add(t);
                //Debug.Log(Tov3(Collections.allPoints[t.vertex1]));
                //Debug.Log(Tov3(Collections.allPoints[t.vertex2]));
                //Debug.Log(Tov3(Collections.allPoints[t.vertex3]));

                if (!Collections.areaSizeList.ContainsKey(t.areaSize))
                    Collections.areaSizeList.Add(t.areaSize, idx);
                idx++;
            }
        }



        //Debug.LogWarning(delaunayEdgeList.Count);
        idx = 0;
        foreach (DelaunayEdge e in delaunayEdgeList)
        {
            //Debug.Log(e.start + " " + e.end);
            double lng = getLength(Collections.allPoints[e.start], Collections.allPoints[e.end]);
            if (!Collections.edgeLengthList.ContainsKey(lng))
                Collections.edgeLengthList.Add(lng, idx);
            idx++;
            //else
            //{
            //  if(delaunayEdgeList[Collections.edgeLengthList[lng]].start == Collections.allPoints[e.start])
            //}
        }

        //Debug.LogWarning("Voronoi " + voronoiEdgeList.Count);
        List<VoronoiEdge> tmp = new List<VoronoiEdge>();
        foreach (VoronoiEdge v in voronoiEdgeList)
        {

            if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 0 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 0)
            {
                //Discard
                //Debug.Log(0);
            }
            else if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 1 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 1)
            {

                //Keep
                //Debug.Log(1);
                tmp.Add(v);
            }
            else
            {

                //Intersection
                //Debug.Log(2);
                Point o0 = ToP(Vector3.one);
                Point o1 = o0;
                if (Voronoi.intersect2D_SegPoly(v.start, v.end, ref polygonSides, ref o0, ref o1) == 1)
                {
                    //Debug.Log(Tov3(v.start) + " " + Tov3(v.end) + " " + Tov3(o0) + " " + Tov3(o1));
                    if (Tov3(v.start) != Tov3(o0))
                        v.start = o0;
                    if (Tov3(v.end) != Tov3(o1))
                        v.end = o1;
                }
                tmp.Add(v);
            }
            voronoiEdgeList = tmp;
        }
    }
    public void DelaunayTriangulate()
    {
        superTriangle = Delaunay.SuperTriangle(Collections.allPoints);
        delaunayTriangleList = Delaunay.Triangulate(superTriangle, Collections.allPoints);
        delaunayEdgeList = Delaunay.DelaunayEdges(superTriangle, delaunayTriangleList);
    }

    public void VoronoiDiagram()
    {
        voronoiEdgeList = Voronoi.VoronoiEdges(delaunayEdgeList);
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

        Collections.areaSizeList = new SortedList<double, int>();
        Collections.edgeLengthList = new SortedList<double, int>();
        Collections.allPoints = new List<Point>();
        for (int i = 0 ; i < VPLCount; i++)
        {
            if (PrimaryLight.type == LightType.Spot||PrimaryLight.type==LightType.Point)
            {
                GetHitPointFromPrimaryLight(out RaycastHit hit);

                GameObject VPLLight = new GameObject("VPL");
                Light lightComp = VPLLight.AddComponent<Light>();

                VPLLight.transform.position = hit.point + hit.normal.normalized * 0.0001f;
                lightComp.type = LightType.Point;
                lightComp.range = 1000;
                lightComp.intensity = 3.0f / VPLCount;
                lightComp.shadows = LightShadows.Soft;
                lightComp.shadowResolution = UnityEngine.Rendering.LightShadowResolution.Low;
                VPLLight.transform.SetParent(VPLCollector.transform);
            }
        }

        Recalculate();
        //Debug.Log("Length " + pointListCopy.Count + " " + pointList.Count);

        //for (int i=0;i< pointListCopy.Count;i++)
        //{
        //    Debug.Log(pointListCopy[i] + " " + pointList[i]);
        //}

    }

    Point ToP(Vector3 v)
    {
        return new Point(v.x, v.y, v.z);
    }

    Vector3 Tov3(Point p)
    {
        return new Vector3((float)p.x, (float)p.y, (float)p.z);
    }

    int times = 2;
    void Update()
    {
        if (true)
        {
            Light[] transforms = VPLCollector.GetComponentsInChildren<Light>();

            pList.Clear();
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
                        if (angle > PrimaryLight.spotAngle / 2.0f)
                        {
                            //Debug.LogWarning("Angle");
                            deletedIdxList.Add(i);
                        }
                    }
                }
            }
            float radius = Mathf.Tan(Mathf.Deg2Rad * PrimaryLight.spotAngle / 2) * PrimaryLight.range;
            //Debug.Log(radius);
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
                    intersectPlane(ref n,ref p0,ref l0,ref l,ref t);
                    Vector3 p = l0 + l * t;
                    p = PrimaryLight.transform.InverseTransformPoint(p) / radius;
                    p.z = 0;
                    //Debug.Log(p);
                    pList.Add(p);
                }
                else if(i == deleteIdx)
                {
                    if (tmpIdx + 1 < deletedIdxList.Count)
                        tmpIdx++;
                }
            }

            pointList.Clear();
            for (int i = 0; i < pList.Count; i++)
                pointList.Add(pList[i]);
            Recalculate();

            //透過Voronoi刪除新增
            int remainVPLToDelete = minimalDeletedVPLPerFrame - deletedIdxList.Count;
            //Debug.Log("deletedIdxList " + deletedIdxList.Count);
            //List<int> removeList = new List<int>();
            //取出最小距離VPL 刪除
            while (remainVPLToDelete > 0)
            {
                int minEdgeIdx = Collections.edgeLengthList.Values.First();
                // Point st = Collections.allPoints[delaunayEdgeList[minEdgeIdx].start];
                // Point ed = Collections.allPoints[delaunayEdgeList[minEdgeIdx].end];

                deletedIdxList.Add(pointMapping[delaunayEdgeList[minEdgeIdx].start]);
                pointList.RemoveAt(pointMapping[delaunayEdgeList[minEdgeIdx].start]);

                Recalculate();
                remainVPLToDelete--;
            }

            int modifyIdx = 0;
            int addCount = deletedIdxList.Count > minimalDeletedVPLPerFrame ? deletedIdxList.Count : minimalDeletedVPLPerFrame;
            //Debug.Log("AddCount " + addCount);
            //取出最大面積Delaunay 新增
            while (addCount > 0)
            {
                int maxAreaIdx = Collections.areaSizeList.Values.Last();
                int times = 5;
                int maxAreaValidIdx = 0;
                while (times > 0)
                {
                    maxAreaIdx = Collections.areaSizeList.Values.ElementAt(Collections.areaSizeList.Count - 1 - maxAreaValidIdx);
                    //Debug.Log(maxAreaIdx);
                    Point center = realTriangleList[maxAreaIdx].center;
                    int endPointInsideOutside = Voronoi.cn_PnPoly(center, ref polygonSides);

                    if (endPointInsideOutside == 1)
                    {
                        break;
                    }
                    else
                    {
                        //If all not valid will error,hope it wont happen

                        //Debug.LogWarning("Outside once!");
                        //Debug.Log(Tov3(center));
                        maxAreaValidIdx++;
                    }
                    times--;
                }

                bool rayHit = GetHitPointFromRay(Tov3(realTriangleList[maxAreaIdx].center), out RaycastHit hit);
                if (rayHit)
                {
                    pointList.Add(Tov3(realTriangleList[maxAreaIdx].center));
                    transforms[deletedIdxList[modifyIdx]].transform.position = hit.point + hit.normal.normalized * 0.0001f;
                }
                else
                {
                    GetHitPointFromPrimaryLight(out RaycastHit hitRand);
                    transforms[deletedIdxList[modifyIdx]].transform.position = hitRand.point + hitRand.normal.normalized * 0.0001f;
                }

                modifyIdx++;
                addCount--;
            }
            times--;
        }
    }

    bool GetHitPointFromRay(Vector3 dir, out RaycastHit hit)
    {
        hit = new RaycastHit();
        float radius = Mathf.Tan(Mathf.Deg2Rad * PrimaryLight.spotAngle / 2) * PrimaryLight.range;
        Vector3 target = PrimaryLight.transform.forward * PrimaryLight.range + PrimaryLight.transform.rotation * dir;
        Ray ray = new Ray(PrimaryLight.transform.position, target);
        return Physics.Raycast(ray, out hit, 100);

    }

    void GetHitPointFromPrimaryLight(out RaycastHit hit)
    {
        hit = new RaycastHit();
        bool rayHit = false;
        Vector3 unitCirclePos = Vector3.one;
        while (rayHit == false)
        {
            Vector3 dir = Vector3.one;
            if (PrimaryLight.type == LightType.Spot)
                dir = GetDirectionFromSpotlight(ref unitCirclePos);
            else if (PrimaryLight.type == LightType.Point)
                dir = new Vector3(Random.value * 2 - 1, Random.value * 2 - 1, Random.value * 2 - 1);

            Ray ray = new Ray(PrimaryLight.transform.position, dir);
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

    Vector3 GetDirectionFromSpotlight(ref Vector3 unitCirclePos)
    {
        float radius = Mathf.Tan(Mathf.Deg2Rad * PrimaryLight.spotAngle / 2) * PrimaryLight.range;
        Vector2 unitCircle = NonUniformInsideUnitCircle();
        Vector2 circle = unitCircle * radius;

        unitCirclePos = unitCircle;

        Vector3 target = PrimaryLight.transform.forward*PrimaryLight.range + PrimaryLight.transform.rotation * new Vector3(circle.x, circle.y);
        return target;
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
        Gizmos.color = Color.yellow;
        for (int i=0;i<pList.Count;i++)
            Gizmos.DrawSphere(pList[i], 0.02f);



        if (polygonSides.Count != 0)
            for (int i = 0; i < polygonSides.Count - 1; i++)
                Gizmos.DrawLine(Tov3(polygonSides[i]), Tov3(polygonSides[i + 1]));
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(new Vector3(0, 0, 0), 1);
        if (voronoiEdgeList!=null)
        foreach (VoronoiEdge v in voronoiEdgeList)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(new Vector3((float)v.start.x, (float)v.start.y, (float)v.start.z),
                new Vector3((float)v.end.x, (float)v.end.y, (float)v.end.z));
            //Debug.Log(v.start.x + " " + v.start.y + " " + v.end.x + " " + v.end.y);
        }
        if (delaunayEdgeList != null)
            foreach (DelaunayEdge d in delaunayEdgeList)
            {
                Vector3 st = pointList[pointMapping[d.start]];
                Vector3 en = pointList[pointMapping[d.end]];
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(new Vector3((float)st.x, (float)st.y, (float)st.z),
                    new Vector3((float)en.x, (float)en.y, (float)en.z));
            }
        if(pointList != null)
            foreach (Vector3 p in pointList)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(p, 0.01f);
            }
    }
}
