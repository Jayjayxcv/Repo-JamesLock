// SoftbodyVolumeGrid.cs
// Attach this script to an empty GameObject.
// Assign a target mesh to deform and the collider that defines its volume.

using System.Collections.Generic;
using UnityEngine;

public class SoftbodyVolumeGrid : MonoBehaviour
{
    [Header("Grid Settings")]
    public Vector3Int resolution = new Vector3Int(10, 10, 10); // Grid resolution (X, Y, Z)
    public float spacing = 0.5f;                               // Distance between points

    [Header("Physics")]
    public float mass = 1f;
    public float gravity = -9.81f;
    public float damping = 0.98f;

    [Header("Mesh Settings")]
    public MeshFilter meshFilter;         // The mesh to deform
    public Collider volumeCollider;       // The mesh volume to determine inside/outside

    class Point
    {
        public Vector3 position;
        public Vector3 velocity;
        public Vector3 force;
        public float mass;
        public bool inside;

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

        // Center grid around collider bounds
        Bounds bounds = volumeCollider.bounds;
        gridOrigin = bounds.center - 0.5f * Vector3.Scale(spacing * Vector3.one, (Vector3)(resolution - Vector3Int.one));

        for (int x = 0; x < resolution.x; x++)
        {
            for (int y = 0; y < resolution.y; y++)
            {
                for (int z = 0; z < resolution.z; z++)
                {
                    Vector3 pos = gridOrigin + new Vector3(x, y, z) * spacing;

                    // Check if point is inside the collider
                    Vector3 closest = volumeCollider.ClosestPoint(pos);
                    bool isInside = Vector3.Distance(closest, pos) < 0.001f;

                    grid[x, y, z] = new Point(pos, isInside, mass);
                }
            }
        }
    }

    void ApplyForces()
    {
        foreach (var p in grid)
        {
            if (!p.inside) continue; // Only simulate inside points

            p.force = Vector3.zero; // Reset force

            // Gravity
            p.force += Vector3.up * gravity * p.mass;

            // TODO: Volume constraints with neighbors
            // Placeholder for force based on neighboring points and volume
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

            // TODO: Collision response (keep inside collider or push out)
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
            }
        }
    }
}
