// SoftbodyVolumeGrid.cs
// Attach this script to an empty GameObject.
// Assign a target mesh to deform and the MeshCollider that defines its volume.

using System.Collections.Generic;
using UnityEngine;

public class SoftbodySpringGridV1 : MonoBehaviour
{
    [Header("Grid Settings")]
    public Vector3Int resolution = new Vector3Int(10, 10, 10); // Grid resolution (X, Y, Z)
    public float spacing = 0.1f;                               // Distance between points
    private float diagonalSpacing;

    [Header("Physics")]
    public float mass = 1f;
    public float gravity = -9.81f;
    public float damping = 0.1f;
    public float volumeStiffness = 0.1f;
    public float springStiffness = 20f;
    public float collisionStiffness = 500f;
    public float speed = 10f;

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

        public bool middle;

        // Neighbor references (axis-aligned only)
        public Point up, down, left, right, forward, back,
            upforward, downforward, leftforward, rightforward, upleft, upright;

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
        diagonalSpacing = spacing * Mathf.Sqrt(2f);
        InitGrid();
    }

    /*void Update()
    {
        ApplyForces();
        Integrate(Time.deltaTime);
    }*/
    
    private int slowUpdate;
    void FixedUpdate()
    {
        if (slowUpdate > speed)
        {
            ApplyForces();
            Integrate(Time.deltaTime);
            slowUpdate = 0;
        }
        else
            slowUpdate++;
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
                    if (x > 0) 
                        p.left = grid[x - 1, y, z];
                    if (x < resolution.x - 1) 
                        p.right = grid[x + 1, y, z];
                    if (y > 0) 
                        p.down = grid[x, y - 1, z];
                    if (y < resolution.y - 1) 
                        p.up = grid[x, y + 1, z];
                    if (z > 0) 
                        p.back = grid[x, y, z - 1];
                    if (z < resolution.z - 1) 
                        p.forward = grid[x, y, z + 1];

                    if (z < resolution.z - 1 && y < resolution.y - 1)
                        p.upforward = grid[x, y + 1, z + 1];
                    if (z < resolution.z - 1 && y > 0)
                        p.downforward = grid[x, y - 1, z + 1];
                    if (z < resolution.z - 1 && x > 0)
                        p.leftforward = grid[x - 1, y, z + 1];
                    if (z < resolution.z - 1 && x < resolution.x - 1)
                        p.rightforward = grid[x + 1, y, z + 1];
                    if (y < resolution.y - 1 && x > 0)
                        p.upleft = grid[x - 1, y + 1, z];
                    if (y < resolution.y - 1 && x < resolution.x - 1)
                        p.upright = grid[x + 1, y + 1, z];



                    if (x == resolution.x / 2 && y == resolution.y / 2 && z == resolution.z / 2)
                        p.middle = true;
                }
            }
        }
    }

    bool PointInsideMesh(Vector3 point)
    {
        // Custom ray-mesh intersection test
        Mesh mesh = meshFilter.sharedMesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        int hitCount = 0;
        Vector3 dir = Random.onUnitSphere;

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
            /*if (p.up != null && p.up.inside && p.forward != null && p.forward.inside)
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
                AddVolumeForce(p, p.down, p.back);*/

            // Spring forces
            if (p.up != null && p.up.inside && p.inside) 
                ApplySpringForce(p, p.up, spacing);
            if (p.right != null && p.right.inside && p.inside) 
                ApplySpringForce(p, p.right, spacing);
            if (p.forward != null && p.forward.inside && p.inside) 
                ApplySpringForce(p, p.forward, spacing);

            if (p.upforward != null && p.upforward.inside && p.inside) 
                ApplySpringForce(p, p.upforward, diagonalSpacing);
            if (p.downforward != null && p.downforward.inside && p.inside) 
                ApplySpringForce(p, p.downforward, diagonalSpacing);
            if (p.leftforward != null && p.leftforward.inside && p.inside) 
                ApplySpringForce(p, p.leftforward, diagonalSpacing);
            if (p.rightforward != null && p.rightforward.inside && p.inside) 
                ApplySpringForce(p, p.rightforward, diagonalSpacing);
            if (p.upleft != null && p.upleft.inside && p.inside) 
                ApplySpringForce(p, p.upleft, diagonalSpacing);
            if (p.upright != null && p.upright.inside && p.inside) 
                ApplySpringForce(p, p.upright, diagonalSpacing);
        }
    }

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

    void ApplySpringForce(Point a, Point b, float restLength)
    {
        Vector3 delta = b.position - a.position;
        float currentLength = delta.magnitude;

        float diff = currentLength - restLength;

        if (Mathf.Abs(diff) > 1e-4f * restLength)
        {
            Vector3 direction = delta.normalized;
            Vector3 springForce = springStiffness * diff * direction;
            Vector3 relativeVelocity = b.velocity - a.velocity;
            float velocityAlongSpring = Vector3.Dot(relativeVelocity, direction);
            Vector3 dampingForce = -damping * velocityAlongSpring * direction;
            Vector3 force = springForce + dampingForce;
            a.force += force;
            b.force -= force;
        }
    }

    void Integrate(float dt)
    {
        foreach (var p in grid)
        {
            if (!p.inside ) continue;
            //if (p.middle) continue;

            Vector3 acceleration = p.force / p.mass;
            p.velocity = (p.velocity + acceleration * dt);
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

                if (p.upforward != null && p.upforward.inside)
                    Gizmos.DrawLine(p.position, p.upforward.position);
                if (p.downforward != null && p.downforward.inside)
                    Gizmos.DrawLine(p.position, p.downforward.position);
                if (p.leftforward != null && p.leftforward.inside)
                    Gizmos.DrawLine(p.position, p.leftforward.position);
                if (p.rightforward != null && p.rightforward.inside)
                    Gizmos.DrawLine(p.position, p.rightforward.position);
                if (p.upleft != null && p.upleft.inside)
                    Gizmos.DrawLine(p.position, p.upleft.position);
                if (p.upright != null && p.upright.inside)
                    Gizmos.DrawLine(p.position, p.upright.position);
            }
        }
        DrawDebugVectors();
    }
    void DrawDebugVectors()
    {
        if (grid == null) return;

        foreach (var p in grid)
        {
            if (p == null || !p.inside) continue;

            // Velocity (green)
            Gizmos.color = Color.green;
            Gizmos.DrawLine(p.position, p.position + p.velocity);

            // Acceleration (red) = force / mass
            Vector3 acceleration = p.force / p.mass;
            Gizmos.color = Color.red;
            Gizmos.DrawLine(p.position, p.position + acceleration);
        }
    }
}
