using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter))]
public class MeshDeformerXai : MonoBehaviour
{
    [SerializeField] private Vector3 originPoint; // Starting point for point distribution
    [SerializeField] private int pointCount = 100; // Number of points to distribute
    [SerializeField] private float influenceRadius = 1f; // Radius of influence for each point
    [SerializeField] private float deformationStrength = 0.1f; // Strength of deformation

    private MeshFilter meshFilter;
    private Vector3[] originalVertices;
    private Vector3[] deformedVertices;
    private List<ControlPoint> controlPoints;
    private Dictionary<int, List<PointInfluence>> vertexInfluences;

    // Struct to hold control point data
    private struct ControlPoint
    {
        public Vector3 originalPosition;
        public Vector3 currentPosition;

        public ControlPoint(Vector3 position)
        {
            originalPosition = position;
            currentPosition = position;
        }
    }

    // Struct to store influence of a control point on a vertex
    private struct PointInfluence
    {
        public int pointIndex;
        public float weight;
    }

    void Start()
    {
        meshFilter = GetComponent<MeshFilter>();
        originalVertices = meshFilter.mesh.vertices;
        deformedVertices = new Vector3[originalVertices.Length];
        controlPoints = new List<ControlPoint>();
        vertexInfluences = new Dictionary<int, List<PointInfluence>>();

        InitializePoints();
        BindPointsToVertices();
    }

    void Update()
    {
        // Example: Randomly move control points for demonstration
        for (int i = 0; i < controlPoints.Count; i++)
        {
            ControlPoint point = controlPoints[i];
            point.currentPosition = point.originalPosition + Random.insideUnitSphere * deformationStrength * Time.deltaTime;
            controlPoints[i] = point;
        }

        UpdateMesh();
    }

    // Distribute points inside the mesh starting from originPoint
    void InitializePoints()
    {
        Bounds bounds = meshFilter.mesh.bounds;
        Vector3 meshCenter = transform.TransformPoint(bounds.center);
        Vector3 meshExtents = bounds.extents;

        // Ensure the origin point is inside the mesh bounds
        originPoint = transform.InverseTransformPoint(originPoint);
        if (!bounds.Contains(originPoint))
        {
            Debug.LogWarning("Origin point is outside mesh bounds. Clamping to bounds.");
            originPoint = bounds.ClosestPoint(originPoint);
        }

        // Convert origin point back to world space for distribution
        Vector3 worldOrigin = transform.TransformPoint(originPoint);

        // Generate points (simple uniform distribution for now)
        for (int i = 0; i < pointCount; i++)
        {
            // Random point within bounds, starting from origin
            Vector3 offset = Random.insideUnitSphere * Mathf.Max(meshExtents.x, meshExtents.y, meshExtents.z);
            Vector3 point = worldOrigin + offset;

            // Transform to local space and check if inside mesh
            Vector3 localPoint = transform.InverseTransformPoint(point);
            if (IsPointInsideMesh(localPoint))
            {
                controlPoints.Add(new ControlPoint(localPoint));
            }
        }

        Debug.Log($"Generated {controlPoints.Count} control points inside mesh.");
    }

    // Check if a point is inside the mesh using raycasting
    bool IsPointInsideMesh(Vector3 localPoint)
    {
        Vector3 worldPoint = transform.TransformPoint(localPoint);
        int hitCount = 0;
        Vector3 rayDirection = Vector3.right; // Arbitrary direction for raycasting

        Ray ray = new Ray(worldPoint, rayDirection);
        RaycastHit[] hits = Physics.RaycastAll(ray, Mathf.Infinity);

        foreach (var hit in hits)
        {
            if (hit.collider.gameObject == gameObject)
            {
                hitCount++;
            }
        }

        // Odd number of hits means the point is inside
        return hitCount % 2 == 1;
    }

    // Bind control points to mesh vertices based on distance
    void BindPointsToVertices()
    {
        for (int i = 0; i < originalVertices.Length; i++)
        {
            Vector3 vertex = originalVertices[i];
            List<PointInfluence> influences = new List<PointInfluence>();

            for (int j = 0; j < controlPoints.Count; j++)
            {
                float distance = Vector3.Distance(vertex, controlPoints[j].originalPosition);
                if (distance < influenceRadius)
                {
                    float weight = 1f / (1f + distance * distance); // Inverse distance weighting
                    influences.Add(new PointInfluence { pointIndex = j, weight = weight });
                }
            }

            vertexInfluences[i] = influences;
        }
    }

    // Update mesh vertices based on control point movements
    void UpdateMesh()
    {
        for (int i = 0; i < originalVertices.Length; i++)
        {
            Vector3 displacement = Vector3.zero;
            float totalWeight = 0f;

            if (vertexInfluences.ContainsKey(i))
            {
                foreach (var influence in vertexInfluences[i])
                {
                    Vector3 pointDisplacement = controlPoints[influence.pointIndex].currentPosition - controlPoints[influence.pointIndex].originalPosition;
                    displacement += pointDisplacement * influence.weight;
                    totalWeight += influence.weight;
                }
            }

            // Apply weighted displacement
            if (totalWeight > 0)
            {
                deformedVertices[i] = originalVertices[i] + displacement / totalWeight;
            }
            else
            {
                deformedVertices[i] = originalVertices[i];
            }
        }

        meshFilter.mesh.vertices = deformedVertices;
        meshFilter.mesh.RecalculateNormals();
    }

    // Optional: Visualize control points in the editor
    void OnDrawGizmos()
    {
        if (controlPoints == null) return;

        Gizmos.color = Color.red;
        foreach (var point in controlPoints)
        {
            Gizmos.DrawSphere(transform.TransformPoint(point.currentPosition), 0.05f);
        }
    }
}