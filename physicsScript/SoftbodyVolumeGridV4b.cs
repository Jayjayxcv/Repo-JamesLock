using System.Collections.Generic;
using UnityEngine;

public class SoftbodyVolumeGridV4b : MonoBehaviour
{
    [Header("Grid Settings")]
    public Vector3Int resolution = new Vector3Int(10, 10, 10); // Grid resolution (X, Y, Z)
    public float spacing = 0.1f;                               // Distance between points

    [Header("Physics")]
    public float mass = 1f;
    public float gravity = -9.81f;
    public float damping = 0.98f;
    public float volumeStiffness = 0.1f;
    public float springStiffness = 100f;
    public float collisionStiffness = 500f;

    [Header("Mesh Settings")]
    public MeshFilter meshFilter;         // The mesh to deform
    public MeshCollider meshCollider;     // Non-convex MeshCollider for volume testing

    // ... existing fields ...
    public float springDamping = 0.1f;
    public float volumeDamping = 0.1f;

    class Point
    {
        public Vector3 position, velocity, force, prevPosition;
        public float mass;
        public bool inside;
        public Point up, down, left, right, forward, back;
        public Dictionary<Point, float> restLengths = new Dictionary<Point, float>();
        public Point(Vector3 pos, bool inside, float mass)
        {
            this.position = pos;
            this.prevPosition = pos;
            this.velocity = Vector3.zero;
            this.force = Vector3.zero;
            this.inside = inside;
            this.mass = mass;
        }
    }

    class Triangle
    {
        public Point p0, p1, p2;
        public float restArea;
        public Triangle(Point p0, Point p1, Point p2)
        {
            this.p0 = p0; this.p1 = p1; this.p2 = p2;
            Vector3 v1 = p1.position - p0.position;
            Vector3 v2 = p2.position - p0.position;
            this.restArea = 0.5f * Vector3.Cross(v1, v2).magnitude;
        }
    }

    private Point[,,] grid;
    private Vector3 gridOrigin;

    void Start()
    {
        InitGrid();
    }

    void Update()
    {
        ApplyForces();
        Integrate(Time.deltaTime);
    }

    private List<Triangle> triangles = new List<Triangle>();

    void InitGrid()
    {
        grid = new Point[resolution.x, resolution.y, resolution.z];
        Bounds bounds = meshCollider.bounds;
        gridOrigin = bounds.center - 0.5f * Vector3.Scale(spacing * Vector3.one, (Vector3)(resolution - Vector3Int.one));

        for (int x = 0; x < resolution.x; x++)
        {
            for (int y = 0; y < resolution.y; y++)
            {
                for (int z = 0; z < resolution.z; z++)
                {
                    Vector3 pos = gridOrigin + new Vector3(x, y, z) * spacing;
                    bool isInside = PointInsideMesh(pos);
                    grid[x, y, z] = new Point(pos, isInside, mass);
                }
            }
        }

        for (int x = 0; x < resolution.x; x++)
        {
            for (int y = 0; y < resolution.y; y++)
            {
                for (int z = 0; z < resolution.z; z++)
                {
                    Point p = grid[x, y, z];
                    if (x > 0) { p.left = grid[x - 1, y, z]; p.restLengths[p.left] = spacing; }
                    if (x < resolution.x - 1) { p.right = grid[x + 1, y, z]; p.restLengths[p.right] = spacing; }
                    if (y > 0) { p.down = grid[x, y - 1, z]; p.restLengths[p.down] = spacing; }
                    if (y < resolution.y - 1) { p.up = grid[x, y + 1, z]; p.restLengths[p.up] = spacing; }
                    if (z > 0) { p.back = grid[x, y, z - 1]; p.restLengths[p.back] = spacing; }
                    if (z < resolution.z - 1) { p.forward = grid[x, y, z + 1]; p.restLengths[p.forward] = spacing; }
                    // Add triangles (reduced set)
                    if (x < resolution.x - 1 && y < resolution.y - 1 && p.inside && p.up != null && p.up.inside && p.right != null && p.right.inside)
                        triangles.Add(new Triangle(p, p.up, p.right));
                    if (x < resolution.x - 1 && z < resolution.z - 1 && p.inside && p.forward != null && p.forward.inside && p.right != null && p.right.inside)
                        triangles.Add(new Triangle(p, p.forward, p.right));
                }
            }
        }
    }

    /*bool PointInsideMesh(Vector3 point)
    {
        Mesh mesh = meshFilter.sharedMesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        int hitCount = 0;
        Vector3 dir = Random.onUnitSphere; // Random direction
        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 v0 = meshFilter.transform.TransformPoint(vertices[triangles[i]]);
            Vector3 v1 = meshFilter.transform.TransformPoint(vertices[triangles[i + 1]]);
            Vector3 v2 = meshFilter.transform.TransformPoint(vertices[triangles[i + 2]]);
            if (RayIntersectsTriangle(point, dir, v0, v1, v2))
                hitCount++;
        }
        return hitCount % 2 == 1;
    }*/

    bool PointInsideMesh(Vector3 point)
    {
        // Custom ray-mesh intersection test
        Mesh mesh = meshFilter.sharedMesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        int hitCount = 0;
        Vector3 dir = Vector3.right;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 v0 = meshFilter.transform.TransformPoint(vertices[triangles[i]]);
            Vector3 v1 = meshFilter.transform.TransformPoint(vertices[triangles[i + 1]]);
            Vector3 v2 = meshFilter.transform.TransformPoint(vertices[triangles[i + 2]]);

            if (RayIntersectsTriangle(point, dir, v0, v1, v2))
                hitCount++;
        }

        return hitCount % 2 == 1;
    }

    // Möller–Trumbore ray-triangle intersection algorithm
    bool RayIntersectsTriangle(Vector3 rayOrigin, Vector3 rayDir, Vector3 v0, Vector3 v1, Vector3 v2)
    {
        const float EPSILON = 1e-6f;
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 h = Vector3.Cross(rayDir, edge2);
        float a = Vector3.Dot(edge1, h);

        if (a > -EPSILON && a < EPSILON)
            return false; // Ray is parallel to triangle

        float f = 1.0f / a;
        Vector3 s = rayOrigin - v0;
        float u = f * Vector3.Dot(s, h);
        if (u < 0.0f || u > 1.0f)
            return false;

        Vector3 q = Vector3.Cross(s, edge1);
        float v = f * Vector3.Dot(rayDir, q);
        if (v < 0.0f || u + v > 1.0f)
            return false;

        float t = f * Vector3.Dot(edge2, q);
        return t > EPSILON;
    }

    void ApplyForces()
    {
        foreach (var p in grid)
        {
            if (!p.inside) continue;
            p.force = Vector3.zero;
            p.force += Vector3.up * gravity * p.mass;
            if (p.up != null && p.up.inside) ApplySpringForce(p, p.up);
            if (p.down != null && p.down.inside) ApplySpringForce(p, p.down);
            if (p.left != null && p.left.inside) ApplySpringForce(p, p.left);
            if (p.right != null && p.right.inside) ApplySpringForce(p, p.right);
            if (p.forward != null && p.forward.inside) ApplySpringForce(p, p.forward);
            if (p.back != null && p.back.inside) ApplySpringForce(p, p.back);
        }
        foreach (var t in triangles)
        {
            if (t.p0.inside && t.p1.inside && t.p2.inside)
                AddVolumeForce(t.p0, t.p1, t.p2);
        }
    }

    void AddVolumeForce(Point p0, Point p1, Point p2)
    {
        Vector3 v1 = p1.position - p0.position;
        Vector3 v2 = p2.position - p0.position;
        Vector3 normal = Vector3.Cross(v1, v2);
        float currentArea = normal.magnitude * 0.5f;
        float restArea = (Mathf.Sqrt(3) / 4) * spacing * spacing; // Equilateral triangle area
        float delta = currentArea - restArea;
        Vector3 n = normal.normalized;
        Vector3 force0 = volumeStiffness * delta * Vector3.Cross(p2.position - p1.position, n);
        Vector3 force1 = volumeStiffness * delta * Vector3.Cross(p0.position - p2.position, n);
        Vector3 force2 = volumeStiffness * delta * Vector3.Cross(p1.position - p0.position, n);
        Vector3 relativeVelocity = p0.velocity - (p1.velocity + p2.velocity) / 2f;
        Vector3 dampingForce = -volumeDamping * Vector3.Dot(relativeVelocity, n) * Vector3.Cross(p2.position - p1.position, n);
        p0.force += force0 + dampingForce;
        p1.force += force1 + dampingForce;
        p2.force += force2 + dampingForce;
    }

    void ApplySpringForce(Point a, Point b)
    {
        float restLength = a.restLengths.ContainsKey(b) ? a.restLengths[b] : spacing;
        Vector3 delta = b.position - a.position;
        float currentLength = delta.magnitude;
        float diff = currentLength - restLength;
        if (Mathf.Abs(diff) > 1e-4f * restLength)
        {
            Vector3 direction = delta.normalized;
            Vector3 springForce = springStiffness * diff * direction;
            Vector3 relativeVelocity = b.velocity - a.velocity;
            float velocityAlongSpring = Vector3.Dot(relativeVelocity, direction);
            Vector3 dampingForce = -springDamping * velocityAlongSpring * direction;
            Vector3 force = springForce + dampingForce;
            a.force += force;
            b.force -= force;
        }
    }

    void Integrate(float dt)
    {
        foreach (var p in grid)
        {
            if (!p.inside) continue;
            Vector3 acceleration = p.force / p.mass;
            Vector3 newPosition = 2f * p.position - p.prevPosition + acceleration * dt * dt;
            p.velocity = (newPosition - p.position) / dt;
            p.prevPosition = p.position;
            p.position = newPosition;
            if (p.position.y < 0f)
            {
                p.position.y = 0f;
                if (p.velocity.y < 0f)
                    p.velocity.y *= -0.3f;
            }
            /*if (!PointInsideMesh(p.position))
            {
                Vector3 closestPoint = meshCollider.ClosestPoint(p.position);
                p.position = closestPoint;
                Vector3 normal = (p.position - meshCollider.transform.position).normalized;
                p.velocity -= Vector3.Dot(p.velocity, normal) * normal;
            }*/
}
    }

    void OnDrawGizmosSelected()
    {
        if (grid == null) return;

        Gizmos.color = Color.cyan;
        foreach (var p in grid)
        {
            if (p != null && p.inside)
            {
                Gizmos.DrawSphere(p.position, spacing * 0.1f);
                if (p.up != null && p.up.inside)
                    Gizmos.DrawLine(p.position, p.up.position);
                if (p.forward != null && p.forward.inside)
                    Gizmos.DrawLine(p.position, p.forward.position);
                if (p.right != null && p.right.inside)
                    Gizmos.DrawLine(p.position, p.right.position);
            }
        }
    }
}