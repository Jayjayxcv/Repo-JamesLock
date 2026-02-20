// SoftbodyVolumeGrid.cs
// Attach this script to an empty GameObject.
// Assign a target mesh to deform and the MeshCollider that defines its volume.

using System.Collections.Generic;
using UnityEngine;

public class SoftbodyVolumeGridV3 : MonoBehaviour
{
    [Header("Grid Settings")]
    public Vector3Int resolution = new Vector3Int(10, 10, 10); // Grid resolution (X, Y, Z)
    public float spacing = 0.5f;                               // Distance between points

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

    class Point
    {
        public Vector3 position;
        public Vector3 velocity;
        public Vector3 force;
        public float mass;
        public bool inside;

        // Neighbor references (axis-aligned only)
        public Point up, down, left, right, forward, back;

        public Point(Vector3 pos, bool inside, float mass)
        {
            this.position = pos;
            this.velocity = Vector3.zero;
            this.force = Vector3.zero;
            this.inside = inside;
            this.mass = mass;
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

        // Link neighbors
        for (int x = 0; x < resolution.x; x++)
        {
            for (int y = 0; y < resolution.y; y++)
            {
                for (int z = 0; z < resolution.z; z++)
                {
                    Point p = grid[x, y, z];
                    if (x > 0) p.left = grid[x - 1, y, z];
                    if (x < resolution.x - 1) p.right = grid[x + 1, y, z];
                    if (y > 0) p.down = grid[x, y - 1, z];
                    if (y < resolution.y - 1) p.up = grid[x, y + 1, z];
                    if (z > 0) p.back = grid[x, y, z - 1];
                    if (z < resolution.z - 1) p.forward = grid[x, y, z + 1];
                }
            }
        }
    }

    /*bool PointInsideMesh(Vector3 point)
    {
        // Cast in 6 axis-aligned directions to improve robustness
        Vector3[] directions = new Vector3[] {
            Vector3.right, Vector3.left,
            Vector3.up, Vector3.down,
            Vector3.forward, Vector3.back
        };

        Mesh mesh = meshFilter.sharedMesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        int totalHits = 0;

        foreach (Vector3 dir in directions)
        {
            int hitCount = 0;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v0 = meshFilter.transform.TransformPoint(vertices[triangles[i]]);
                Vector3 v1 = meshFilter.transform.TransformPoint(vertices[triangles[i + 1]]);
                Vector3 v2 = meshFilter.transform.TransformPoint(vertices[triangles[i + 2]]);

                if (RayIntersectsTriangle(point, dir, v0, v1, v2))
                    hitCount++;
            }
            totalHits += hitCount;
        }

        return totalHits % 2 == 1; // Odd hits = inside
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
            if (!p.inside) continue; // Skip non-simulated points

            p.force = Vector3.zero;
            p.force += Vector3.up * gravity * p.mass;

            // Volume-based spring forces between neighbor triplets
            if (p.up != null && p.up.inside && p.forward != null && p.forward.inside)
                AddVolumeForce(p, p.up, p.forward);
            if (p.up != null && p.up.inside && p.right != null && p.right.inside)
                AddVolumeForce(p, p.up, p.right);
            if (p.up != null && p.up.inside && p.left != null && p.left.inside)
                AddVolumeForce(p, p.up, p.left);
            if (p.up != null && p.up.inside && p.back != null && p.back.inside)
                AddVolumeForce(p, p.up, p.back);
            if (p.forward != null && p.forward.inside && p.right != null && p.right.inside)
                AddVolumeForce(p, p.forward, p.right);
            if (p.forward != null && p.forward.inside && p.left != null && p.left.inside)
                AddVolumeForce(p, p.forward, p.left);
            if (p.back != null && p.back.inside && p.right != null && p.right.inside)
                AddVolumeForce(p, p.back, p.right);
            if (p.back != null && p.back.inside && p.left != null && p.left.inside)
                AddVolumeForce(p, p.back, p.left);
            if (p.down != null && p.down.inside && p.forward != null && p.forward.inside)
                AddVolumeForce(p, p.down, p.forward);
            if (p.down != null && p.down.inside && p.right != null && p.right.inside)
                AddVolumeForce(p, p.down, p.right);
            if (p.down != null && p.down.inside && p.left != null && p.left.inside)
                AddVolumeForce(p, p.down, p.left);
            if (p.down != null && p.down.inside && p.back != null && p.back.inside)
                AddVolumeForce(p, p.down, p.back);

            // Spring forces
            if (p.up != null && p.up.inside && p.inside) ApplySpringForce(p, p.up);
            if (p.down != null && p.down.inside && p.inside) ApplySpringForce(p, p.down);
            if (p.left != null && p.left.inside && p.inside) ApplySpringForce(p, p.left);
            if (p.right != null && p.right.inside && p.inside) ApplySpringForce(p, p.right);
            if (p.forward != null && p.forward.inside && p.inside) ApplySpringForce(p, p.forward);
            if (p.back != null && p.back.inside && p.inside) ApplySpringForce(p, p.back);
        }
    }

    /*void AddVolumeForce(Point p0, Point p1, Point p2)
    {
        Vector3 v1 = p1.position - p0.position;
        Vector3 v2 = p2.position - p0.position;
        Vector3 normal = Vector3.Cross(v1, v2);

        float currentVolume = Vector3.Dot(normal, Vector3.up);
        float restVolume = spacing * spacing; // Flat 2D area approximation

        float delta = currentVolume - restVolume;
        Vector3 force = -volumeStiffness * delta * normal.normalized;

        p0.force += force;
        p1.force -= force * 0.5f;
        p2.force -= force * 0.5f;
    }

    void AddVolumeForce(Point p0, Point p1, Point p2)
    {
        Vector3 v1 = p1.position - p0.position;
        Vector3 v2 = p2.position - p0.position;
        Vector3 normal = Vector3.Cross(v1, v2);

        float currentVolume = normal.magnitude * 0.5f;             // Actual triangle area
        float restVolume = spacing * spacing * 0.5f;       // Area between points at rest

        float delta = currentVolume - restVolume;
        Vector3 force = -volumeStiffness * delta * normal.normalized;

        p0.force += force / 3f;
        p1.force += force / 3f;
        p2.force += force / 3f;
    }

    void AddVolumeForce(Point p0, Point p1, Point p2)
    {
        Vector3 restV1 = p1.position - p0.position;
        Vector3 restV2 = p2.position - p0.position;

        Vector3 currentV1 = p1.position - p0.position;
        Vector3 currentV2 = p2.position - p0.position;

        float currentVolume = Vector3.Dot(Vector3.Cross(currentV1, currentV2), Vector3.up);
        float restVolume = spacing * spacing;

        float delta = currentVolume - restVolume;
        Vector3 forceDir = Vector3.Cross(currentV1, currentV2).normalized;
        Vector3 force = -volumeStiffness * delta * forceDir;

        p0.force += force;
        p1.force -= force * 0.5f;
        p2.force -= force * 0.5f;
    }*/

    void AddVolumeForce(Point p0, Point p1, Point p2)
    {
        // Current triangle vectors
        Vector3 v1 = p1.position - p0.position;
        Vector3 v2 = p2.position - p0.position;
        Vector3 normal = Vector3.Cross(v1, v2);

        // Current area (0.5 * ||v1 x v2||)
        float currentArea = normal.magnitude * 0.5f;

        // Rest area for an isosceles right triangle with side length 'spacing'
        float restArea = 0.5f * spacing * spacing;

        // Area difference
        float delta = currentArea - restArea;

        // Compute force direction based on area gradient
        Vector3 n = normal.normalized;
        Vector3 force0 = volumeStiffness * delta * Vector3.Cross(p2.position - p1.position, n);
        Vector3 force1 = volumeStiffness * delta * Vector3.Cross(p0.position - p2.position, n);
        Vector3 force2 = volumeStiffness * delta * Vector3.Cross(p1.position - p0.position, n);

        // Apply forces symmetrically
        p0.force += force0;
        p1.force += force1;
        p2.force += force2;
    }

    void ApplySpringForce(Point a, Point b)
    {
        Vector3 delta = b.position - a.position;
        float currentLength = delta.magnitude;
        float restLength = spacing;
        float diff = currentLength - restLength;

        if (Mathf.Abs(diff) > 1e-4f * restLength)
        {
            Vector3 direction = delta.normalized;
            Vector3 force = springStiffness * diff * direction;

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
            p.velocity = (p.velocity + acceleration * dt) * damping;
            p.position += p.velocity * dt;

            // Simple collision handling with world plane at y = 0
            if (p.position.y < 0f)
            {
                p.position.y = 0f;
                if (p.velocity.y < 0f)
                    p.velocity.y *= -0.3f; // Bounce with damping
            }
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
