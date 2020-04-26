using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Delaunay;
using Delaunay.Geo;


public class MyVPLManager : MonoBehaviour
{
    public GameObject VPLCollector;
    public Light lightSource;

    public int VPLCount = 20;

    public int minDeleteVPLCount = 2;
    List<Vector2> polygonSides = new List<Vector2>();

    private List<Light> VPLs = new List<Light>();

    private List<Vector2> allPoints = new List<Vector2>();
   
    void Start()
    {
        int sideCount = 20;
        float perAngle = 360 / sideCount * Mathf.Deg2Rad;
        float nowAngle = 90 * Mathf.Deg2Rad;
        for (int i = 0; i < sideCount + 1; i++)
        {
            float x = Mathf.Cos(nowAngle) * 1.0128f;// * 1.4142135623f;
            float y = Mathf.Sin(nowAngle) * 1.0128f;// * 1.4142135623f;
            nowAngle += perAngle;
            polygonSides.Add(new Vector2(x, y));
        }


        //Generate initial VPL
        for(int i = 0; i < VPLCount; i++)
        {
            GetHitPointFromPrimaryLight(out RaycastHit hit);
            GameObject VPLLight = new GameObject("VPL_" + i.ToString());
            Light lightComp = VPLLight.AddComponent<Light>();

            VPLLight.transform.position = hit.point + hit.normal.normalized * 0.0001f;
            lightComp.type = LightType.Point;
            lightComp.range = 1000;
            lightComp.intensity = 3.0f / VPLCount;
            lightComp.shadows = LightShadows.Soft;
            lightComp.shadowResolution = UnityEngine.Rendering.LightShadowResolution.Low;
            VPLLight.transform.SetParent(VPLCollector.transform);

            VPLs.Add(lightComp);
        }

        //Voronoi voronoi = new Voronoi(allPoints, null, new Rect(-1, -1, 2, 2));
}
    
    void Update()
    {
        //print(allPoints.Count);
        //print(VPLs.Count);

        int culledCount = 0;
        for (int i = 0; i < VPLs.Count; i++)
        {
            ////檢查VPL是否不可見
            //Vector3 dirToVPL = Vector3.Normalize(VPLs[i].transform.position - lightSource.transform.position);
            //float rayLength = Vector3.Magnitude(VPLs[i].transform.position - lightSource.transform.position) - 0.0001f;
            //bool rayHit = false;

            ////光源射向VPL
            //Ray ray = new Ray(lightSource.transform.position, dirToVPL);
            //rayHit = Physics.Raycast(ray, out RaycastHit gomi, rayLength);

            ////VPL射向光源
            //ray.origin = VPLs[i].transform.position;
            //ray.direction = -dirToVPL;
            //rayHit = Physics.Raycast(ray, out gomi, rayLength);

            //if (rayHit)
            //{
            //    Destroy(VPLs[i].gameObject);
            //    culledCount++;
            //    continue;
            //}

            Vector3 pointDir = Vector3.Normalize(VPLs[i].transform.position - lightSource.transform.position);
            float angle = Mathf.Acos(Vector3.Dot(pointDir, lightSource.transform.forward)) * Mathf.Rad2Deg;
            if (angle > lightSource.spotAngle / 2.0f)
            {
                DestroyImmediate(VPLs[i].gameObject);

                VPLs.RemoveAt(i);
                allPoints.RemoveAt(i);
                i--;
                culledCount++;
            }
        }
        

        //print("Culled Count :" + culledCount.ToString());
        
        // delete VPLs
        int remainVPL2Delete = minDeleteVPLCount - culledCount;
        while (remainVPL2Delete-- > 0)
        {
            Voronoi voronoi = new Voronoi(allPoints, null, new Rect(-1, -1, 2, 2));
            List<Edge> delaunayEdgeList = voronoi._edges;
            List<Triangle> delaunayTriangleList = voronoi._triangles;

            float minDis = float.MaxValue;
            int minIndex = 0;
            for (int i = 0; i < delaunayEdgeList.Count; i++)
            {
                Edge dE = delaunayEdgeList[i];
                float distance = dE.SitesDistance();
                if (distance < minDis)
                {
                    minIndex = i;
                    minDis = distance;
                }
            }

            uint pointA = delaunayEdgeList[minIndex].leftSite._siteIndex;
            uint pointB = delaunayEdgeList[minIndex].rightSite._siteIndex;
            minDis = float.MaxValue;

            // true : pointA, false : pointB
            bool flag = true;
            for (int i = 0; i < delaunayEdgeList.Count; i++)
            {
                if(i != minIndex)
                {
                    Edge dE = delaunayEdgeList[i];
                    if(pointA == dE.leftSite._siteIndex || pointA == dE.rightSite._siteIndex)
                    {
                        float distance = dE.SitesDistance();
                        if (distance < minDis)
                            flag = true;
                    }

                    if (pointB == dE.leftSite._siteIndex || pointB == dE.rightSite._siteIndex)
                    {
                        float distance = dE.SitesDistance();
                        if (distance < minDis)
                            flag = false;
                    }
                }
            }

            int deleteIndex = flag ? (int)pointA : (int)pointB;
            allPoints.RemoveAt(deleteIndex);
            DestroyImmediate(VPLs[deleteIndex].gameObject);
            VPLs.RemoveAt(deleteIndex);
        }

        // add VPLs
        
        int addCount = culledCount > minDeleteVPLCount ? culledCount : minDeleteVPLCount;
        while (addCount-- > 0)
        {
            Voronoi voronoi = new Voronoi(allPoints, null, new Rect(-1, -1, 2, 2));
            //List<Edge> delaunayEdgeList = voronoi._edges;
            List<Triangle> delaunayTriangleList = voronoi._triangles;

            float maxArea = -float.MaxValue;
            int maxIndex = 0;
            for (int i = 0; i < delaunayTriangleList.Count; i++)
            {
                Triangle dT = delaunayTriangleList[i];
                List<Vector2> triPoints = new List<Vector2>();
                foreach (Site s in dT.sites)
                    triPoints.Add(s.Coord);
                Polygon poly = new Polygon(triPoints);

                float triArea = poly.Area();
                if (triArea > maxArea)
                {
                    delaunayTriangleList[i].CircumcircleCenter(out bool IsReal);
                    if (IsReal)
                    {
                        maxIndex = i;
                        maxArea = triArea;
                    }
                }
            }

            //print(maxIndex);
            Vector2 circulum = delaunayTriangleList[maxIndex].CircumcircleCenter(out bool isReal);
            allPoints.Add(circulum);
            //print(circulum);

            float radius = Mathf.Tan(Mathf.Deg2Rad * lightSource.spotAngle / 2) * lightSource.range;
            Vector3 dir = circulum * radius;
            Vector3 target = lightSource.transform.forward * lightSource.range + dir;
            Ray ray = new Ray(lightSource.transform.position, target);

            if (Physics.Raycast(ray, out RaycastHit hit, 100))
            {
                GameObject VPLLight = new GameObject("VPL_Add");
                Light lightComp = VPLLight.AddComponent<Light>();

                VPLLight.transform.position = hit.point + hit.normal.normalized * 0.0001f;
                lightComp.type = LightType.Point;
                lightComp.range = 1000;
                lightComp.intensity = 3.0f / VPLCount;
                lightComp.shadows = LightShadows.Soft;
                lightComp.shadowResolution = UnityEngine.Rendering.LightShadowResolution.Low;
                VPLLight.transform.SetParent(VPLCollector.transform);
                VPLs.Add(lightComp);
            }
            else
                print("Add light failed!");
        }
        
    }
    


    private void GetHitPointFromPrimaryLight(out RaycastHit hit)
    {
        hit = new RaycastHit();

        bool rayHit = false;
        Vector2 unitCircle = Vector2.zero;
        while (rayHit == false)
        {
            Vector3 dir = Vector3.one;
            if (lightSource.type == LightType.Spot)
            {
                float radius = Mathf.Tan(Mathf.Deg2Rad * lightSource.spotAngle / 2) * lightSource.range;
                unitCircle = NonUniformInsideUnitCircle();
                Vector2 circle = unitCircle * radius;
                dir = lightSource.transform.forward * lightSource.range + lightSource.transform.rotation * new Vector3(circle.x, circle.y);
            }
            else if (lightSource.type == LightType.Point)
                dir = new Vector3(Random.value * 2 - 1, Random.value * 2 - 1, Random.value * 2 - 1);

            Ray ray = new Ray(lightSource.transform.position, dir);
            rayHit = Physics.Raycast(ray, out hit, 100);
        }
        allPoints.Add(unitCircle);
    }

    private Vector2 NonUniformInsideUnitCircle()
    {
        float angle = Random.value * Mathf.PI * 2f;
        float radius = Random.value;
        return new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
    }

    private List<LineSegment> m_edges = null;
    private List<LineSegment> m_spanningTree;
    private List<LineSegment> m_delaunayTriangulation;

    void OnDrawGizmos()
    {
        Voronoi v = new Delaunay.Voronoi(allPoints, null, new Rect(-1, -1, 2, 2));
        m_edges = v.VoronoiDiagram();
        m_spanningTree = v.SpanningTree(KruskalType.MINIMUM);
        m_delaunayTriangulation = v.DelaunayTriangulation();

        Gizmos.color = Color.red;
        if (allPoints != null)
        {
            for (int i = 0; i < allPoints.Count; i++)
            {
                Gizmos.DrawSphere(allPoints[i], 0.0002f);
            }
        }

        if (m_edges != null)
        {
            Gizmos.color = Color.white;
            for (int i = 0; i < m_edges.Count; i++)
            {
                Vector2 left = (Vector2)m_edges[i].p0;
                Vector2 right = (Vector2)m_edges[i].p1;
                Gizmos.DrawLine((Vector3)left, (Vector3)right);
            }
        }

        Gizmos.color = Color.magenta;
        if (m_delaunayTriangulation != null)
        {
            for (int i = 0; i < m_delaunayTriangulation.Count; i++)
            {
                Vector2 left = (Vector2)m_delaunayTriangulation[i].p0;
                Vector2 right = (Vector2)m_delaunayTriangulation[i].p1;
                Gizmos.DrawLine((Vector3)left, (Vector3)right);
            }
        }

        /*if (m_spanningTree != null)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < m_spanningTree.Count; i++)
            {
                LineSegment seg = m_spanningTree[i];
                Vector2 left = (Vector2)seg.p0;
                Vector2 right = (Vector2)seg.p1;
                Gizmos.DrawLine((Vector3)left, (Vector3)right);
            }
        }*/
    }
}
