using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Exclusion-zone geometry and obstacle-aware path routing.
    /// </summary>
    public static class Obstacles
    {
        /// <summary>
        /// Shared empty obstacle field.
        /// </summary>
        static public readonly ObstacleField EMPTY_OBSTACLES = new ObstacleField();

        /// <summary>
        /// Computes signed polygon area.
        /// </summary>
        /// <param name="Ring">Polygon ring.</param>
        /// <returns>Signed area.</returns>
        static public Double SignedArea
            (
            IReadOnlyList<(Double X, Double Y)> Ring
            )
        {
            // Positive sign indicates counter-clockwise winding.
            Double Area = 0.0;
            Int32 N = Ring.Count;
            for (Int32 Index = 0; Index < N; Index += 1)
            {
                (Double X1, Double Y1) = Ring[Index];
                (Double X2, Double Y2) = Ring[(Index + 1) % N];
                Area += (X1 * Y2) - (X2 * Y1);
            }

            return 0.5 * Area;
        }

        /// <summary>
        /// Returns a CCW-ordered copy of a polygon ring.
        /// </summary>
        /// <param name="Ring">Input ring.</param>
        /// <returns>CCW ring.</returns>
        static public List<(Double X, Double Y)> AsCcw
            (
            IReadOnlyList<(Double X, Double Y)> Ring
            )
        {
            // Normalize ring winding so outward normals are consistent.
            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>(Ring);
            if ((Out.Count >= 3) && (SignedArea(Out) < 0.0))
            {
                Out.Reverse();
            }

            return Out;
        }

        /// <summary>
        /// Ray-cast point-in-ring test.
        /// </summary>
        /// <param name="X">Point x.</param>
        /// <param name="Y">Point y.</param>
        /// <param name="Ring">Polygon ring.</param>
        /// <returns>Whether point is inside.</returns>
        static public Boolean PointInRing
            (
            Double X,
            Double Y,
            IReadOnlyList<(Double X, Double Y)> Ring
            )
        {
            // Boundary is treated as outside to permit perimeter travel edges.
            Boolean Inside = false;
            Int32 N = Ring.Count;
            Int32 J = N - 1;
            for (Int32 I = 0; I < N; I += 1)
            {
                (Double Xi, Double Yi) = Ring[I];
                (Double Xj, Double Yj) = Ring[J];
                if ((Yi > Y) != (Yj > Y))
                {
                    Double XAt = ((Xj - Xi) * (Y - Yi) / ((Yj - Yi) + 1e-18)) + Xi;
                    if (X < XAt)
                    {
                        Inside = !Inside;
                    }
                }

                J = I;
            }

            return Inside;
        }

        /// <summary>
        /// Convex miter offset of a CCW polygon.
        /// </summary>
        /// <param name="Ring">CCW input ring.</param>
        /// <param name="Clearance">Offset clearance.</param>
        /// <returns>Buffered ring.</returns>
        static public List<(Double X, Double Y)> MiterBuffer
            (
            List<(Double X, Double Y)> Ring,
            Double Clearance
            )
        {
            // Build an outward clearance polygon around each exclusion.
            Int32 N = Ring.Count;
            if ((N < 3) || (Clearance <= 0.0))
            {
                return new List<(Double X, Double Y)>(Ring);
            }

            List<(Double X, Double Y)> Normals = new List<(Double X, Double Y)>();
            for (Int32 Index = 0; Index < N; Index += 1)
            {
                (Double X1, Double Y1) = Ring[Index];
                (Double X2, Double Y2) = Ring[(Index + 1) % N];
                Double Ex = X2 - X1;
                Double Ey = Y2 - Y1;
                Double Length = Math.Sqrt((Ex * Ex) + (Ey * Ey));
                if (Length <= 1e-18)
                {
                    Length = 1e-18;
                }

                Normals.Add((Ey / Length, -Ex / Length));
            }

            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>();
            for (Int32 Index = 0; Index < N; Index += 1)
            {
                (Double X, Double Y) NPrev = Normals[(Index - 1 + N) % N];
                (Double X, Double Y) NCur = Normals[Index];
                Double Mx = NPrev.X + NCur.X;
                Double My = NPrev.Y + NCur.Y;
                Double MLen = Math.Sqrt((Mx * Mx) + (My * My));
                if (MLen < 1e-9)
                {
                    Mx = NCur.X;
                    My = NCur.Y;
                    MLen = 1.0;
                }

                Mx /= MLen;
                My /= MLen;
                Double CosHalf = (Mx * NCur.X) + (My * NCur.Y);
                Double Scale = Clearance / Math.Max(CosHalf, 0.35);
                (Double X, Double Y) P = Ring[Index];
                Out.Add((P.X + (Mx * Scale), P.Y + (My * Scale)));
            }

            return Out;
        }

        /// <summary>
        /// Cheap segment-vs-AABB overlap test.
        /// </summary>
        /// <param name="A">Segment start.</param>
        /// <param name="B">Segment end.</param>
        /// <param name="Box">AABB as (minx,miny,maxx,maxy).</param>
        /// <returns>Whether segment can intersect the box.</returns>
        static public Boolean SegmentIntersectsAabb
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            (Double MinX, Double MinY, Double MaxX, Double MaxY) Box
            )
        {
            // Reject lines lying fully outside a slab before expensive checks.
            if (((A.X < Box.MinX) && (B.X < Box.MinX)) || ((A.X > Box.MaxX) && (B.X > Box.MaxX)))
            {
                return false;
            }

            if (((A.Y < Box.MinY) && (B.Y < Box.MinY)) || ((A.Y > Box.MaxY) && (B.Y > Box.MaxY)))
            {
                return false;
            }

            return true;
        }
    }

    /// <summary>
    /// One no-go polygon and its clearance-buffered geometry.
    /// </summary>
    public sealed class ExclusionZone
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="ExclusionZone"/> class.
        /// </summary>
        /// <param name="Ring">Original ring.</param>
        /// <param name="BufferRing">Buffered ring.</param>
        /// <param name="Bbox">Buffered bounding box.</param>
        public ExclusionZone
            (
            List<(Double X, Double Y)> Ring,
            List<(Double X, Double Y)> BufferRing,
            (Double MinX, Double MinY, Double MaxX, Double MaxY) Bbox
            )
        {
            // Store original and buffered geometry for fast blocking tests.
            this.Ring = Ring;
            this.BufferRing = BufferRing;
            this.Bbox = Bbox;
        }

        /// <summary>
        /// Gets original CCW ring.
        /// </summary>
        public List<(Double X, Double Y)> Ring { get; }

        /// <summary>
        /// Gets clearance-buffered ring.
        /// </summary>
        public List<(Double X, Double Y)> BufferRing { get; }

        /// <summary>
        /// Gets buffered ring bounding box.
        /// </summary>
        public (Double MinX, Double MinY, Double MaxX, Double MaxY) Bbox { get; }

        /// <summary>
        /// Creates a zone from an unbuffered ring.
        /// </summary>
        /// <param name="Ring">Input polygon ring.</param>
        /// <param name="Clearance">Buffer clearance.</param>
        /// <returns>Buffered exclusion zone.</returns>
        static public ExclusionZone FromRing
            (
            IReadOnlyList<(Double X, Double Y)> Ring,
            Double Clearance
            )
        {
            // Normalize winding, buffer outward, and cache bbox.
            List<(Double X, Double Y)> Ccw = Obstacles.AsCcw(Ring);
            List<(Double X, Double Y)> Buffer = Obstacles.MiterBuffer(Ccw, Clearance);
            Double MinX = Double.PositiveInfinity;
            Double MinY = Double.PositiveInfinity;
            Double MaxX = Double.NegativeInfinity;
            Double MaxY = Double.NegativeInfinity;
            foreach ((Double X, Double Y) Point in Buffer)
            {
                MinX = Math.Min(MinX, Point.X);
                MinY = Math.Min(MinY, Point.Y);
                MaxX = Math.Max(MaxX, Point.X);
                MaxY = Math.Max(MaxY, Point.Y);
            }

            return new ExclusionZone(Ccw, Buffer, (MinX, MinY, MaxX, MaxY));
        }

        /// <summary>
        /// Tests whether a straight segment crosses buffered interior.
        /// </summary>
        /// <param name="A">Segment start.</param>
        /// <param name="B">Segment end.</param>
        /// <param name="Samples">Number of interior samples.</param>
        /// <returns>Whether the segment is blocked.</returns>
        public Boolean Blocks
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            Int32 Samples = 24
            )
        {
            // Reject quickly with bbox, then sample interior points.
            if (!Obstacles.SegmentIntersectsAabb(A, B, Bbox))
            {
                return false;
            }

            for (Int32 K = 1; K < Samples; K += 1)
            {
                Double T = K / (Double)Samples;
                Double X = A.X + ((B.X - A.X) * T);
                Double Y = A.Y + ((B.Y - A.Y) * T);
                if (Obstacles.PointInRing(X, Y, BufferRing))
                {
                    return true;
                }
            }

            return false;
        }
    }

    /// <summary>
    /// Obstacle field with visibility graph and Dijkstra routing.
    /// </summary>
    public sealed class ObstacleField
    {
        private List<(Double X, Double Y)> _Nodes;
        private List<List<(Int32 Node, Double Weight)>> _Adj;

        /// <summary>
        /// Initializes a new instance of the <see cref="ObstacleField"/> class.
        /// </summary>
        /// <param name="Zones">Exclusion zones.</param>
        public ObstacleField
            (
            IEnumerable<ExclusionZone>? Zones = null
            )
        {
            // Build graph once so repeated path queries stay cheap.
            this.Zones = (Zones == null) ? new List<ExclusionZone>() : new List<ExclusionZone>(Zones);
            _Nodes = new List<(Double X, Double Y)>();
            _Adj = new List<List<(Int32 Node, Double Weight)>>();
            BuildGraph();
        }

        /// <summary>
        /// Gets exclusion zones.
        /// </summary>
        public List<ExclusionZone> Zones { get; }

        /// <summary>
        /// Gets a value indicating whether the field has active zones.
        /// </summary>
        public Boolean Active
        {
            get
            {
                // Empty fields should short-circuit all expensive routing.
                return Zones.Count > 0;
            }
        }

        /// <summary>
        /// Creates an obstacle field from raw rings.
        /// </summary>
        /// <param name="Rings">Raw polygon rings.</param>
        /// <param name="Clearance">Buffer clearance.</param>
        /// <returns>Built obstacle field.</returns>
        static public ObstacleField FromRings
            (
            IEnumerable<IReadOnlyList<(Double X, Double Y)>> Rings,
            Double Clearance
            )
        {
            // Buffer each valid ring, then build one shared visibility graph.
            List<ExclusionZone> Zones = new List<ExclusionZone>();
            foreach (IReadOnlyList<(Double X, Double Y)> Ring in Rings)
            {
                if (Ring.Count >= 3)
                {
                    Zones.Add(ExclusionZone.FromRing(Ring, Clearance));
                }
            }

            return new ObstacleField(Zones);
        }

        /// <summary>
        /// Creates an obstacle field from already buffered zones.
        /// </summary>
        /// <param name="Zones">Buffered exclusion zones.</param>
        /// <returns>Built obstacle field.</returns>
        static public ObstacleField FromZones
            (
            IEnumerable<ExclusionZone> Zones
            )
        {
            // Reuse existing zone geometry as-is.
            return new ObstacleField(Zones);
        }

        /// <summary>
        /// Tests whether the straight segment between two points is blocked.
        /// </summary>
        /// <param name="A">Segment start.</param>
        /// <param name="B">Segment end.</param>
        /// <returns>Whether blocked.</returns>
        public Boolean Blocked
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Any zone blocking implies path detour is needed.
            foreach (ExclusionZone Zone in Zones)
            {
                if (Zone.Blocks(A, B))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Finds shortest obstacle-avoiding polyline from A to B.
        /// </summary>
        /// <param name="A">Start point.</param>
        /// <param name="B">End point.</param>
        /// <returns>Polyline including start and end points.</returns>
        public List<(Double X, Double Y)> Path
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Short-circuit common straight-through legs.
            if (!Active || !Blocked(A, B))
            {
                return new List<(Double X, Double Y)> { A, B };
            }

            Int32 M = _Nodes.Count;
            Int32 Ai = M;
            Int32 Bi = M + 1;
            List<List<(Int32 Node, Double Weight)>> Adj = CloneAdjacency(_Adj);
            Adj.Add(new List<(Int32 Node, Double Weight)>());
            Adj.Add(new List<(Int32 Node, Double Weight)>());

            for (Int32 K = 0; K < M; K += 1)
            {
                if (Visible(A, _Nodes[K]))
                {
                    Double DistValue = Distance(A, _Nodes[K]);
                    Adj[Ai].Add((K, DistValue));
                    Adj[K].Add((Ai, DistValue));
                }

                if (Visible(B, _Nodes[K]))
                {
                    Double DistValue = Distance(B, _Nodes[K]);
                    Adj[Bi].Add((K, DistValue));
                    Adj[K].Add((Bi, DistValue));
                }
            }

            if (Visible(A, B))
            {
                Double DistValue = Distance(A, B);
                Adj[Ai].Add((Bi, DistValue));
                Adj[Bi].Add((Ai, DistValue));
            }

            (Int32[] Prev, Double[] Dist) = Dijkstra(Adj, Ai, Bi);
            if (Double.IsInfinity(Dist[Bi]))
            {
                return new List<(Double X, Double Y)> { A, B };
            }

            List<Int32> Chain = new List<Int32> { Bi };
            while (Chain[Chain.Count - 1] != Ai)
            {
                Int32 P = Prev[Chain[Chain.Count - 1]];
                if (P < 0)
                {
                    return new List<(Double X, Double Y)> { A, B };
                }

                Chain.Add(P);
            }

            Chain.Reverse();
            List<(Double X, Double Y)> AllPoints = new List<(Double X, Double Y)>(_Nodes);
            AllPoints.Add(A);
            AllPoints.Add(B);
            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>();
            foreach (Int32 Index in Chain)
            {
                Out.Add(AllPoints[Index]);
            }

            return Out;
        }

        /// <summary>
        /// Returns shortest obstacle-aware path length.
        /// </summary>
        /// <param name="A">Start point.</param>
        /// <param name="B">End point.</param>
        /// <returns>Path length.</returns>
        public Double PathLength
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Distance-only accessor used heavily in routing loops.
            if (!Active || !Blocked(A, B))
            {
                return Distance(A, B);
            }

            List<(Double X, Double Y)> Poly = Path(A, B);
            Double Sum = 0.0;
            for (Int32 Index = 0; Index < (Poly.Count - 1); Index += 1)
            {
                Sum += Distance(Poly[Index], Poly[Index + 1]);
            }

            return Sum;
        }

        /// <summary>
        /// Runs Dijkstra shortest path on adjacency list graph.
        /// </summary>
        /// <param name="Adj">Adjacency list.</param>
        /// <param name="Src">Source node index.</param>
        /// <param name="Dst">Destination node index.</param>
        /// <returns>Predecessors and distances arrays.</returns>
        static public (Int32[] Prev, Double[] Dist) Dijkstra
            (
            List<List<(Int32 Node, Double Weight)>> Adj,
            Int32 Src,
            Int32 Dst
            )
        {
            // Classic Dijkstra with lazy-priority queue updates.
            Int32 N = Adj.Count;
            Double[] Dist = new Double[N];
            Int32[] Prev = new Int32[N];
            for (Int32 Index = 0; Index < N; Index += 1)
            {
                Dist[Index] = Double.PositiveInfinity;
                Prev[Index] = -1;
            }

            Dist[Src] = 0.0;
            PriorityQueue<(Int32 Node, Double Dist), Double> Queue = new PriorityQueue<(Int32 Node, Double Dist), Double>();
            Queue.Enqueue((Src, 0.0), 0.0);
            while (Queue.Count > 0)
            {
                (Int32 Node, Double Dist) Current = Queue.Dequeue();
                if (Current.Dist > Dist[Current.Node])
                {
                    continue;
                }

                if (Current.Node == Dst)
                {
                    break;
                }

                foreach ((Int32 Node, Double Weight) Edge in Adj[Current.Node])
                {
                    Double NextDist = Current.Dist + Edge.Weight;
                    if (NextDist < Dist[Edge.Node])
                    {
                        Dist[Edge.Node] = NextDist;
                        Prev[Edge.Node] = Current.Node;
                        Queue.Enqueue((Edge.Node, NextDist), NextDist);
                    }
                }
            }

            return (Prev, Dist);
        }

        /// <summary>
        /// Builds visibility graph over buffered vertices.
        /// </summary>
        private void BuildGraph
            (
            )
        {
            // Connect all mutually visible buffered vertices once.
            _Nodes = new List<(Double X, Double Y)>();
            foreach (ExclusionZone Zone in Zones)
            {
                foreach ((Double X, Double Y) Vertex in Zone.BufferRing)
                {
                    _Nodes.Add(Vertex);
                }
            }

            Int32 M = _Nodes.Count;
            _Adj = new List<List<(Int32 Node, Double Weight)>>();
            for (Int32 Index = 0; Index < M; Index += 1)
            {
                _Adj.Add(new List<(Int32 Node, Double Weight)>());
            }

            for (Int32 I = 0; I < M; I += 1)
            {
                for (Int32 J = I + 1; J < M; J += 1)
                {
                    if (Visible(_Nodes[I], _Nodes[J]))
                    {
                        Double Weight = Distance(_Nodes[I], _Nodes[J]);
                        _Adj[I].Add((J, Weight));
                        _Adj[J].Add((I, Weight));
                    }
                }
            }
        }

        /// <summary>
        /// Tests if direct line between two points crosses no zone interior.
        /// </summary>
        /// <param name="A">First point.</param>
        /// <param name="B">Second point.</param>
        /// <returns>Whether points are mutually visible.</returns>
        private Boolean Visible
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Visibility is blocked by any zone interior crossing.
            foreach (ExclusionZone Zone in Zones)
            {
                if (Zone.Blocks(A, B))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Computes Euclidean distance.
        /// </summary>
        /// <param name="A">First point.</param>
        /// <param name="B">Second point.</param>
        /// <returns>Distance.</returns>
        static private Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Centralized helper avoids duplicate sqrt expressions.
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Clones adjacency list for temporary query graph expansion.
        /// </summary>
        /// <param name="Source">Source adjacency list.</param>
        /// <returns>Cloned adjacency list.</returns>
        static private List<List<(Int32 Node, Double Weight)>> CloneAdjacency
            (
            List<List<(Int32 Node, Double Weight)>> Source
            )
        {
            // Per-query graph mutation must not alter cached base graph.
            List<List<(Int32 Node, Double Weight)>> Out = new List<List<(Int32 Node, Double Weight)>>();
            foreach (List<(Int32 Node, Double Weight)> Row in Source)
            {
                Out.Add(new List<(Int32 Node, Double Weight)>(Row));
            }

            return Out;
        }
    }
}
