using System.Collections.Generic;
using UnityEngine;

public class SoftbodySphere : MonoBehaviour
{
    public int resolution = 6;                  // Number of points per axis
    public float radius = 1f;                   // Radius of softbody
    public float stiffness = 0.2f;              // Spring stiffness
    public float pointMass = 1f;
    //public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public Vector3 gravity = new Vector3(0, 0f, 0);
    public float damping = 0.98f;

    class Point
    {
        public Vector3 position;
        public Vector3 prevPosition;

        public Point(Vector3 pos)
        {
            position = pos;
            prevPosition = pos;
        }

        public void VerletIntegrate(Vector3 acceleration, float dt, float damping)
        {
            Vector3 temp = position;
            position += (position - prevPosition) * damping + acceleration * dt * dt;
            prevPosition = temp;
        }
    }

    struct Spring
    {
        public int a, b;
        public float restLength;

        public Spring(int a, int b, float restLength)
        {
            this.a = a;
            this.b = b;
            this.restLength = restLength;
        }
    }

    List<Point> points;
    List<Spring> springs;

    void Start()
    {
        points = new List<Point>();
        springs = new List<Spring>();

        // Fill the sphere with points
        for (int x = -resolution; x <= resolution; x++)
        {
            for (int y = -resolution; y <= resolution; y++)
            {
                for (int z = -resolution; z <= resolution; z++)
                {
                    Vector3 pos = new Vector3(x, y, z) / resolution * radius;
                    if (pos.magnitude <= radius)
                    {
                        Point p = new Point(transform.position + pos);
                        points.Add(p);
                    }
                }
            }
        }

        // Add springs between nearby points
        float maxDist = radius * 2f / resolution * 1.1f;
        for (int i = 0; i < points.Count; i++)
        {
            for (int j = i + 1; j < points.Count; j++)
            {
                float dist = (points[i].position - points[j].position).magnitude;
                if (dist < maxDist)
                {
                    springs.Add(new Spring(i, j, dist));
                }
            }
        }
    }

    void Update()
    {
        float dt = Time.deltaTime;

        // Step 1: Verlet integration
        foreach (var p in points)
        {
            p.VerletIntegrate(gravity, dt, damping);
        }

        // Step 2: Apply spring constraints
        for (int i = 0; i < 5; i++) // Iteration count
        {
            foreach (var s in springs)
            {
                Point pa = points[s.a];
                Point pb = points[s.b];

                Vector3 delta = pb.position - pa.position;
                float dist = delta.magnitude;
                float diff = (dist - s.restLength) / dist;

                Vector3 offset = delta * 0.5f * stiffness * diff;

                pa.position += offset;
                pb.position -= offset;
            }
        }

        // Step 3: Collision with plane at y = 0
        foreach (var p in points)
        {
            if (p.position.y < 0f)
            {
                p.position.y = 0f;
            }
        }

        // Step 4: Debug draw
        foreach (var p in points)
        {
            Debug.DrawRay(p.position, Vector3.up * 0.05f, Color.cyan);
        }

        foreach (var s in springs)
        {
            Debug.DrawLine(points[s.a].position, points[s.b].position, Color.gray);
        }
    }
}
