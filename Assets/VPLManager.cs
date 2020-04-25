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

    //List<Vector3> transformGraphVector = new List<Vector3>();

    double getLength(Point a,Point b)
    {
        return Mathf.Sqrt((float)((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
    }

    Triangle superTriangle = new Triangle();

    List<Triangle> delaunayTriangleList;
    List<DelaunayEdge> delaunayEdgeList;
    List<VoronoiEdge> voronoiEdgeList;

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

    public void Recalculate()
    {
        Collections.allPoints = Collections.allPoints.OrderBy(point => point.x).ToList();
        DelaunayTriangulate();
        VoronoiDiagram();

        int idx = 0;
        foreach (Triangle t in Collections.allTriangles)
        {
            if (!Collections.areaSizeList.ContainsKey(t.areaSize))
                Collections.areaSizeList.Add(t.areaSize, idx);
            idx++;
        }

        idx = 0;
        foreach (DelaunayEdge e in delaunayEdgeList)
        {
            double lng = getLength(Collections.allPoints[e.start], Collections.allPoints[e.end]);
            if (!Collections.edgeLengthList.ContainsKey(lng))
                Collections.edgeLengthList.Add(lng, idx);
            idx++;
            //else
            //{
            //  if(delaunayEdgeList[Collections.edgeLengthList[lng]].start == Collections.allPoints[e.start])
            //}
        }
    }

    void Start()
    {
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


        //int sideCount = 20;

        //float perAngle = 360 / sideCount * Mathf.Deg2Rad;
        //float nowAngle = 90 * Mathf.Deg2Rad;
        //for (int i = 0; i < sideCount + 1; i++) 
        //{
        //    float x = Mathf.Cos(nowAngle)*1.0128f;// * 1.4142135623f;
        //    float y = Mathf.Sin(nowAngle)*1.0128f;// * 1.4142135623f;
        //    nowAngle += perAngle;
        //    polygonSides.Add(new Point(x, y));
        //}

        //List<VoronoiEdge> tmp = new List<VoronoiEdge>();
        //foreach (VoronoiEdge v in voronoiEdgeList)
        //{

        //    if (Voronoi.cn_PnPoly(v.start, ref polygonSides) == 0 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 0)
        //    {
        //        //Discard
        //        //Debug.Log(0);
        //    }
        //    else if(Voronoi.cn_PnPoly(v.start, ref polygonSides) == 1 && Voronoi.cn_PnPoly(v.end, ref polygonSides) == 1)
        //    {

        //        //Keep
        //        //Debug.Log(1);
        //        tmp.Add(v);
        //    }
        //    else
        //    {

        //        //Intersection
        //        //Debug.Log(2);
        //        Point o0 = ToP(Vector3.one);
        //        Point o1 = o0;
        //        if (Voronoi.intersect2D_SegPoly(v.start, v.end, ref polygonSides, ref o0, ref o1)==1)
        //        {
        //            Debug.Log(Tov3(v.start) + " "+ Tov3(v.end) + " "+ Tov3(o0) + " " + Tov3(o1));
        //            if (Tov3(v.start) != Tov3(o0))
        //                v.start = o0;
        //            if (Tov3(v.end) != Tov3(o1))
        //                v.end = o1;
        //        }
        //        tmp.Add(v);
        //    }
        //    voronoiEdgeList = tmp;
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

    void Update()
    {
        //Debug.Log(PrimaryLight.transform.rotation);
        //Debug.Log("---");
        Light[] transforms = VPLCollector.GetComponentsInChildren<Light>();
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

                if(rayHit)
                {

                    deletedIdxList.Add(i);
                    //GetHitPointFromPrimaryLight(out RaycastHit hit);
                    //transforms[i].transform.position = hit.point + hit.normal.normalized * 0.0001f;
                    continue;
                }

                if (PrimaryLight.type == LightType.Spot)
                {
                    //檢查是否超出Spotlight
                    Vector3 pointDir = Vector3.Normalize(transforms[i].transform.position - PrimaryLight.transform.position);
                    float angle = Mathf.Acos(Vector3.Dot(pointDir, PrimaryLight.transform.forward)) * Mathf.Rad2Deg;
                    if (angle > PrimaryLight.spotAngle / 2.0f)
                    {

                        deletedIdxList.Add(i);
                        //GetHitPointFromPrimaryLight(out RaycastHit hit);
                        //transforms[i].transform.position = hit.point + hit.normal.normalized * 0.0001f;
                    }
                }
            }
        }


        for(int i = deletedIdxList.Count; i>=0;i--)
        {
            Collections.allPoints.RemoveAt(i);
        }

        Recalculate();

        //int remainVPLToDelete = minimalDeletedVPLPerFrame - deletedIdxList.Count;


        //List<int> removeList = new List<int>();
        ////取出最小距離VPL 刪除
        //while (remainVPLToDelete > 0) 
        //{
        //    int minEdgeIdx = Collections.edgeLengthList.Values.First();
        //    Point st = Collections.allPoints[delaunayEdgeList[minEdgeIdx].start];
        //    Point ed = Collections.allPoints[delaunayEdgeList[minEdgeIdx].end];

        //    deletedIdxList.Add(delaunayEdgeList[minEdgeIdx].start);
        //    Collections.allPoints.Remove(st);

        //    Recalculate();
        //    remainVPLToDelete--;
        //}

        //int addIdx = 0;
        //int deletedCount = minimalDeletedVPLPerFrame;
        ////取出最大面積Delaunay 新增
        //while (deletedCount>0)
        //{
        //    int maxAreaIdx = Collections.areaSizeList.Values.Last();
        //    Collections.allPoints.Add(Collections.allTriangles[maxAreaIdx].center);

        //    bool rayHit = GetHitPointFromRay(Tov3(Collections.allTriangles[maxAreaIdx].center), out RaycastHit hit);
        //    if (rayHit)
        //        transforms[deletedIdxList[addIdx]].transform.position = hit.point + hit.normal.normalized * 0.0001f;
        //    else
        //    {
        //        GetHitPointFromPrimaryLight(out RaycastHit hitRand);
        //        transforms[deletedIdxList[addIdx]].transform.position = hitRand.point + hitRand.normal.normalized * 0.0001f;
        //    }


        //    Recalculate();
        //    deletedCount--;
        //}
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
        while (rayHit == false)
        {
            Vector3 dir = Vector3.one;
            if (PrimaryLight.type == LightType.Spot)
                dir = GetDirectionFromSpotlight(PrimaryLight);
            else if (PrimaryLight.type == LightType.Point)
                dir = new Vector3(Random.value * 2 - 1, Random.value * 2 - 1, Random.value * 2 - 1);

            Ray ray = new Ray(PrimaryLight.transform.position, dir);
            rayHit = Physics.Raycast(ray, out hit, 100);

        }
    }

    Vector3 GetDirectionFromSpotlight(Light spot)
    {
        float radius = Mathf.Tan(Mathf.Deg2Rad * spot.spotAngle / 2) * spot.range;
        Vector2 unitCircle = NonUniformInsideUnitCircle();
        Vector2 circle = unitCircle * radius;
        //bounds.extents = new Vector3(1, 0, 1);
        //bounds.center = spot.transform.position;
        Collections.allPoints.Add(new Point(unitCircle.x, unitCircle.y, 0));

        Vector3 target = spot.transform.forward * spot.range + spot.transform.rotation * new Vector3(circle.x, circle.y);
        return target;
    }

    Vector2 NonUniformInsideUnitCircle()
    {
        float angle = Random.value * Mathf.PI * 2f;
        float radius = Random.value;
        return new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
    }

    void OnDrawGizmos()
    {

        Gizmos.color = Color.yellow;

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
                Point st = Collections.allPoints[d.start];
                Point en = Collections.allPoints[d.end];
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(new Vector3((float)st.x, (float)st.y, (float)st.z),
                    new Vector3((float)en.x, (float)en.y, (float)en.z));
            }
        if(Collections.allPoints!=null)
            foreach (Point p in Collections.allPoints)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(Tov3(p), 0.01f);
            }
    }
}
