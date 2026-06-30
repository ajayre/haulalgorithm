using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Gif;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

namespace HaulBuilder
{
    /// <summary>
    /// Ports center-pivot scheduling, detour modeling, and rendering from <c>landlevel.pivot</c>.
    /// </summary>
    public static class Pivot
    {
        /// <summary>
        /// UTM WGS-84 equatorial radius.
        /// </summary>
        private const Double UTM_A = 6378137.0;

        /// <summary>
        /// UTM WGS-84 flattening.
        /// </summary>
        private const Double UTM_F = (1.0 / 298.257223563);

        /// <summary>
        /// UTM scale at central meridian.
        /// </summary>
        private const Double UTM_K0 = 0.9996;

        /// <summary>
        /// Minimum radius fallback in grid-cell units.
        /// </summary>
        private const Double MIN_RADIUS_CELLS = 1.0;

        /// <summary>
        /// Small epsilon used for floating-point comparisons.
        /// </summary>
        private const Double EPSILON = 1e-9;

        /// <summary>
        /// Geometry and geodesy for one center-pivot installation.
        /// </summary>
        public sealed class PivotGeometry
        {
            /// <summary>
            /// Initializes a new geometry instance.
            /// </summary>
            /// <param name="centerX">Pivot anchor easting (UTM metres).</param>
            /// <param name="centerY">Pivot anchor northing (UTM metres).</param>
            /// <param name="radiusM">Arm radius (metres).</param>
            /// <param name="wheelbaseM">Arm width (metres).</param>
            /// <param name="clearanceM">Extra routing clearance (metres).</param>
            /// <param name="zone">UTM zone number.</param>
            /// <param name="isNorthern">True for northern hemisphere UTM northing.</param>
            public PivotGeometry
                (
                Double centerX,
                Double centerY,
                Double radiusM,
                Double wheelbaseM,
                Double clearanceM,
                Int32 zone,
                Boolean isNorthern
                )
            {
                // The geometry is immutable once solved so downstream routing and rendering stay consistent.
                this.CenterX = centerX;
                this.CenterY = centerY;
                this.RadiusM = radiusM;
                this.WheelbaseM = wheelbaseM;
                this.ClearanceM = clearanceM;
                this.Zone = zone;
                this.IsNorthern = isNorthern;
            }

            /// <summary>
            /// Gets pivot anchor easting.
            /// </summary>
            public Double CenterX { get; }

            /// <summary>
            /// Gets pivot anchor northing.
            /// </summary>
            public Double CenterY { get; }

            /// <summary>
            /// Gets arm radius in metres.
            /// </summary>
            public Double RadiusM { get; }

            /// <summary>
            /// Gets arm wheelbase width in metres.
            /// </summary>
            public Double WheelbaseM { get; }

            /// <summary>
            /// Gets additional collision clearance in metres.
            /// </summary>
            public Double ClearanceM { get; }

            /// <summary>
            /// Gets UTM zone.
            /// </summary>
            public Int32 Zone { get; }

            /// <summary>
            /// Gets a value indicating whether northing is northern-hemisphere UTM.
            /// </summary>
            public Boolean IsNorthern { get; }

            /// <summary>
            /// Converts the pivot center UTM coordinate to latitude/longitude.
            /// </summary>
            /// <returns>Tuple of latitude and longitude in degrees.</returns>
            public (Double Latitude, Double Longitude) CenterLatLon
                (
                )
            {
                // This mirrors Python center_latlon so exported metadata matches.
                return LatLonFromUtm(this.CenterX, this.CenterY, this.Zone, this.IsNorthern);
            }

            /// <summary>
            /// Computes arm tip UTM coordinate for a compass bearing.
            /// </summary>
            /// <param name="bearingDeg">Compass bearing (0=N, 90=E).</param>
            /// <returns>Tip easting/northing in UTM metres.</returns>
            public (Double X, Double Y) TipXy
                (
                Double bearingDeg
                )
            {
                // Compass convention uses east as sin and north as cos.
                Double Rad = DegreesToRadians(bearingDeg);
                Double X = this.CenterX + (this.RadiusM * Math.Sin(Rad));
                Double Y = this.CenterY + (this.RadiusM * Math.Cos(Rad));
                return (X, Y);
            }

            /// <summary>
            /// Computes arm tip latitude/longitude for a compass bearing.
            /// </summary>
            /// <param name="bearingDeg">Compass bearing (0=N, 90=E).</param>
            /// <returns>Tuple of latitude and longitude in degrees.</returns>
            public (Double Latitude, Double Longitude) TipLatLon
                (
                Double bearingDeg
                )
            {
                // Tip geodesy is derived from the same inverse UTM conversion as the center.
                (Double X, Double Y) Tip = this.TipXy(bearingDeg);
                return LatLonFromUtm(Tip.X, Tip.Y, this.Zone, this.IsNorthern);
            }
        }

        /// <summary>
        /// One held arm bearing over a contiguous pass range.
        /// </summary>
        public sealed class PivotPosition
        {
            /// <summary>
            /// Initializes one scheduled pivot position.
            /// </summary>
            /// <param name="index">1-based position sequence index.</param>
            /// <param name="firstPass">1-based first pass in this held run.</param>
            /// <param name="lastPass">1-based last pass in this held run.</param>
            /// <param name="bearingDeg">Held compass bearing in degrees.</param>
            /// <param name="moveDeg">Rotation from previous position in degrees.</param>
            /// <param name="conflicts">Number of passes in run that still conflict.</param>
            public PivotPosition
                (
                Int32 index,
                Int32 firstPass,
                Int32 lastPass,
                Double bearingDeg,
                Double moveDeg,
                Int32 conflicts
                )
            {
                // Positions are immutable snapshots so downstream files and images stay deterministic.
                this.Index = index;
                this.FirstPass = firstPass;
                this.LastPass = lastPass;
                this.BearingDeg = bearingDeg;
                this.MoveDeg = moveDeg;
                this.ConflictCount = conflicts;
            }

            /// <summary>
            /// Gets 1-based schedule index.
            /// </summary>
            public Int32 Index { get; }

            /// <summary>
            /// Gets 1-based first pass index.
            /// </summary>
            public Int32 FirstPass { get; }

            /// <summary>
            /// Gets 1-based last pass index.
            /// </summary>
            public Int32 LastPass { get; }

            /// <summary>
            /// Gets held compass bearing.
            /// </summary>
            public Double BearingDeg { get; }

            /// <summary>
            /// Gets move from previous position.
            /// </summary>
            public Double MoveDeg { get; }

            /// <summary>
            /// Gets count of conflicted passes in this run.
            /// </summary>
            public Int32 ConflictCount { get; }

            /// <summary>
            /// Gets number of passes covered by this position.
            /// </summary>
            public Int32 PassCount => (this.LastPass - this.FirstPass + 1);
        }

        /// <summary>
        /// Full center-pivot schedule and summary metrics.
        /// </summary>
        public sealed class PivotPlan
        {
            /// <summary>
            /// Initializes a new empty pivot plan.
            /// </summary>
            /// <param name="geometry">Solved pivot geometry.</param>
            /// <param name="binCount">Angular bin count for planning.</param>
            /// <param name="moveCost">Move cost in detour-equivalent passes.</param>
            public PivotPlan
                (
                PivotGeometry geometry,
                Int32 binCount,
                Double moveCost
                )
            {
                // The planner mutates collections while solving, then callers consume the finalized state.
                this.Geometry = geometry;
                this.Positions = new List<PivotPosition>();
                this.ConflictPasses = new List<Int32>();
                this.BearingByPass = Array.Empty<Double>();
                this.BinCount = binCount;
                this.MoveCost = moveCost;
            }

            /// <summary>
            /// Gets solved geometry.
            /// </summary>
            public PivotGeometry Geometry { get; }

            /// <summary>
            /// Gets contiguous held-bearing positions.
            /// </summary>
            public List<PivotPosition> Positions { get; }

            /// <summary>
            /// Gets total schedule rotation in degrees.
            /// </summary>
            public Double TotalRotationDeg { get; set; }

            /// <summary>
            /// Gets 1-based pass numbers whose chosen bearing still collides.
            /// </summary>
            public List<Int32> ConflictPasses { get; }

            /// <summary>
            /// Gets planning bin count.
            /// </summary>
            public Int32 BinCount { get; }

            /// <summary>
            /// Gets move cost used by dynamic programming.
            /// </summary>
            public Double MoveCost { get; }

            /// <summary>
            /// Gets held bearing per pass in route order.
            /// </summary>
            public Double[] BearingByPass { get; set; }

            /// <summary>
            /// Gets number of arm repositions.
            /// </summary>
            public Int32 MoveCount => Math.Max(0, this.Positions.Count - 1);

            /// <summary>
            /// Gets number of detoured/conflicted passes.
            /// </summary>
            public Int32 DetourCount => this.ConflictPasses.Count;
        }

        /// <summary>
        /// One cut/fill cell entry with applied depth.
        /// </summary>
        public sealed class PassCell
        {
            /// <summary>
            /// Initializes one pass cell.
            /// </summary>
            /// <param name="bx">Cell x index.</param>
            /// <param name="by">Cell y index.</param>
            /// <param name="depthM">Applied depth in metres.</param>
            public PassCell
                (
                Int32 bx,
                Int32 by,
                Double depthM
                )
            {
                // The tuple fields are split for easy interop from serializers and tests.
                this.Bx = bx;
                this.By = by;
                this.DepthM = depthM;
            }

            /// <summary>
            /// Gets cell x index.
            /// </summary>
            public Int32 Bx { get; }

            /// <summary>
            /// Gets cell y index.
            /// </summary>
            public Int32 By { get; }

            /// <summary>
            /// Gets depth in metres.
            /// </summary>
            public Double DepthM { get; }
        }

        /// <summary>
        /// Pass shape required by pivot scheduling and rendering.
        /// </summary>
        public sealed class Pass
        {
            /// <summary>
            /// Initializes a pass.
            /// </summary>
            /// <param name="cutCells">Cut cells with depths.</param>
            /// <param name="fillCells">Fill cells with depths.</param>
            /// <param name="cutEndXy">Cut sweep end coordinate.</param>
            /// <param name="fillStartXy">Fill sweep start coordinate.</param>
            public Pass
                (
                IEnumerable<PassCell> cutCells,
                IEnumerable<PassCell> fillCells,
                (Double X, Double Y) cutEndXy,
                (Double X, Double Y) fillStartXy
                )
            {
                // Pivot logic only consumes disturbed cells and haul leg endpoints.
                this.CutCells = new List<PassCell>(cutCells);
                this.FillCells = new List<PassCell>(fillCells);
                this.CutEndXy = cutEndXy;
                this.FillStartXy = fillStartXy;
            }

            /// <summary>
            /// Gets cut cells.
            /// </summary>
            public List<PassCell> CutCells { get; }

            /// <summary>
            /// Gets fill cells.
            /// </summary>
            public List<PassCell> FillCells { get; }

            /// <summary>
            /// Gets cut sweep end coordinate.
            /// </summary>
            public (Double X, Double Y) CutEndXy { get; }

            /// <summary>
            /// Gets fill sweep start coordinate.
            /// </summary>
            public (Double X, Double Y) FillStartXy { get; }
        }

        /// <summary>
        /// Grid state used by pivot geometry, occupancy, and rendering.
        /// </summary>
        public sealed class Grid
        {
            /// <summary>
            /// Initializes a grid snapshot.
            /// </summary>
            /// <param name="gridSizeM">Cell edge length in metres.</param>
            /// <param name="minX">World minimum easting.</param>
            /// <param name="minY">World minimum northing.</param>
            /// <param name="zone">UTM zone.</param>
            /// <param name="isNorthern">UTM hemisphere flag.</param>
            /// <param name="existing">Existing elevation matrix [by, bx].</param>
            /// <param name="target">Target elevation matrix [by, bx].</param>
            /// <param name="valid">Valid mask matrix [by, bx].</param>
            public Grid
                (
                Double gridSizeM,
                Double minX,
                Double minY,
                Int32 zone,
                Boolean isNorthern,
                Double[,] existing,
                Double[,] target,
                Boolean[,] valid
                )
            {
                // The grid dimensions are derived once so all methods share one coordinate frame.
                this.GridSizeM = gridSizeM;
                this.MinX = minX;
                this.MinY = minY;
                this.Zone = zone;
                this.IsNorthern = isNorthern;
                this.Existing = existing;
                this.Target = target;
                this.Valid = valid;
                this.Height = existing.GetLength(0);
                this.Width = existing.GetLength(1);
            }

            /// <summary>
            /// Gets grid cell edge length in metres.
            /// </summary>
            public Double GridSizeM { get; }

            /// <summary>
            /// Gets grid width in cells.
            /// </summary>
            public Int32 Width { get; }

            /// <summary>
            /// Gets grid height in cells.
            /// </summary>
            public Int32 Height { get; }

            /// <summary>
            /// Gets minimum world easting.
            /// </summary>
            public Double MinX { get; }

            /// <summary>
            /// Gets minimum world northing.
            /// </summary>
            public Double MinY { get; }

            /// <summary>
            /// Gets UTM zone.
            /// </summary>
            public Int32 Zone { get; }

            /// <summary>
            /// Gets a value indicating whether northing is northern-hemisphere UTM.
            /// </summary>
            public Boolean IsNorthern { get; }

            /// <summary>
            /// Gets existing elevation matrix [by, bx].
            /// </summary>
            public Double[,] Existing { get; }

            /// <summary>
            /// Gets target elevation matrix [by, bx].
            /// </summary>
            public Double[,] Target { get; }

            /// <summary>
            /// Gets valid field mask matrix [by, bx].
            /// </summary>
            public Boolean[,] Valid { get; }

            /// <summary>
            /// Computes world center coordinate of a grid cell.
            /// </summary>
            /// <param name="bx">Cell x index.</param>
            /// <param name="by">Cell y index.</param>
            /// <returns>Cell center world coordinate.</returns>
            public (Double X, Double Y) CellCenterXy
                (
                Int32 bx,
                Int32 by
                )
            {
                // Cell centers align with Python's (index + 0.5) formulation.
                Double X = this.MinX + ((bx + 0.5) * this.GridSizeM);
                Double Y = this.MinY + ((by + 0.5) * this.GridSizeM);
                return (X, Y);
            }

            /// <summary>
            /// Computes existing-minus-target delta.
            /// </summary>
            /// <returns>Delta matrix [by, bx].</returns>
            public Double[,] Delta
                (
                )
            {
                // Delta is computed on demand so renderers always reflect current arrays.
                Double[,] Result = new Double[this.Height, this.Width];
                for (Int32 By = 0; By < this.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < this.Width; Bx += 1)
                    {
                        Result[By, Bx] = (this.Existing[By, Bx] - this.Target[By, Bx]);
                    }
                }

                return Result;
            }
        }

        /// <summary>
        /// One row-equivalent of the pivot CSV export.
        /// </summary>
        public sealed class PivotCsvRow
        {
            /// <summary>
            /// Gets or sets position index.
            /// </summary>
            public Int32 Position { get; set; }

            /// <summary>
            /// Gets or sets first pass in run.
            /// </summary>
            public Int32 FirstPass { get; set; }

            /// <summary>
            /// Gets or sets last pass in run.
            /// </summary>
            public Int32 LastPass { get; set; }

            /// <summary>
            /// Gets or sets number of passes in run.
            /// </summary>
            public Int32 PassCount { get; set; }

            /// <summary>
            /// Gets or sets held bearing in degrees.
            /// </summary>
            public Double BearingDeg { get; set; }

            /// <summary>
            /// Gets or sets 16-point cardinal direction.
            /// </summary>
            public String Cardinal { get; set; } = String.Empty;

            /// <summary>
            /// Gets or sets move from previous position in degrees.
            /// </summary>
            public Double MoveDeg { get; set; }

            /// <summary>
            /// Gets or sets tip latitude.
            /// </summary>
            public Double TipLat { get; set; }

            /// <summary>
            /// Gets or sets tip longitude.
            /// </summary>
            public Double TipLon { get; set; }

            /// <summary>
            /// Gets or sets conflict count in run.
            /// </summary>
            public Int32 ConflictCount { get; set; }
        }

        /// <summary>
        /// Structured CSV-equivalent payload for callers that do not want file IO.
        /// </summary>
        public sealed class PivotCsvData
        {
            /// <summary>
            /// Initializes a new structured CSV payload.
            /// </summary>
            public PivotCsvData
                (
                )
            {
                // Metadata and rows are split so callers can map to JSON or tabular formats directly.
                this.Metadata = new Dictionary<String, String>(StringComparer.Ordinal);
                this.Rows = new List<PivotCsvRow>();
            }

            /// <summary>
            /// Gets key/value metadata that would have been CSV header comments.
            /// </summary>
            public Dictionary<String, String> Metadata { get; }

            /// <summary>
            /// Gets tabular rows that mirror CSV data lines.
            /// </summary>
            public List<PivotCsvRow> Rows { get; }
        }

        /// <summary>
        /// One exclusion zone polygon with buffered ring.
        /// </summary>
        public sealed class ExclusionZone
        {
            /// <summary>
            /// Initializes a buffered exclusion zone.
            /// </summary>
            /// <param name="ring">Original CCW ring.</param>
            /// <param name="bufferRing">Buffered CCW ring.</param>
            public ExclusionZone
                (
                IReadOnlyList<(Double X, Double Y)> ring,
                IReadOnlyList<(Double X, Double Y)> bufferRing
                )
            {
                // Bounding box is cached for fast segment reject checks in repeated routing calls.
                this.Ring = ring;
                this.BufferRing = bufferRing;

                Double MinX = Double.PositiveInfinity;
                Double MinY = Double.PositiveInfinity;
                Double MaxX = Double.NegativeInfinity;
                Double MaxY = Double.NegativeInfinity;
                foreach ((Double X, Double Y) Point in bufferRing)
                {
                    MinX = Math.Min(MinX, Point.X);
                    MinY = Math.Min(MinY, Point.Y);
                    MaxX = Math.Max(MaxX, Point.X);
                    MaxY = Math.Max(MaxY, Point.Y);
                }

                this.BBox = (MinX, MinY, MaxX, MaxY);
            }

            /// <summary>
            /// Gets original ring.
            /// </summary>
            public IReadOnlyList<(Double X, Double Y)> Ring { get; }

            /// <summary>
            /// Gets buffered ring.
            /// </summary>
            public IReadOnlyList<(Double X, Double Y)> BufferRing { get; }

            /// <summary>
            /// Gets axis-aligned box of buffered ring.
            /// </summary>
            public (Double MinX, Double MinY, Double MaxX, Double MaxY) BBox { get; }

            /// <summary>
            /// Builds one zone from a polygon ring and clearance.
            /// </summary>
            /// <param name="ring">Input ring vertices.</param>
            /// <param name="clearance">Clearance buffer in metres.</param>
            /// <returns>Buffered exclusion zone.</returns>
            public static ExclusionZone FromRing
                (
                IReadOnlyList<(Double X, Double Y)> ring,
                Double clearance
                )
            {
                // Rings are normalized CCW before miter buffering to keep outward normals stable.
                List<(Double X, Double Y)> Ccw = EnsureCcw(ring);
                List<(Double X, Double Y)> Buffered = MiterBuffer(Ccw, clearance);
                return new ExclusionZone(Ccw, Buffered);
            }

            /// <summary>
            /// Determines whether segment <paramref name="a"/> to <paramref name="b"/> crosses buffered interior.
            /// </summary>
            /// <param name="a">Start point.</param>
            /// <param name="b">End point.</param>
            /// <param name="samples">Interior samples along segment.</param>
            /// <returns>True when the segment passes through buffered interior.</returns>
            public Boolean Blocks
                (
                (Double X, Double Y) a,
                (Double X, Double Y) b,
                Int32 samples = 24
                )
            {
                // The two-stage test mirrors Python: AABB reject first, sampled interior test second.
                if ((!SegmentIntersectsAabb(a, b, this.BBox)))
                {
                    return false;
                }

                for (Int32 k = 1; k < samples; k += 1)
                {
                    Double T = ((Double)k / samples);
                    Double X = (a.X + ((b.X - a.X) * T));
                    Double Y = (a.Y + ((b.Y - a.Y) * T));
                    if (PointInRing(X, Y, this.BufferRing))
                    {
                        return true;
                    }
                }

                return false;
            }
        }

        /// <summary>
        /// Obstacle field with shortest-path routing around exclusion zones.
        /// </summary>
        public sealed class ObstacleField
        {
            /// <summary>
            /// Shared empty obstacle field.
            /// </summary>
            static public readonly ObstacleField EMPTY_OBSTACLES = new ObstacleField(Array.Empty<ExclusionZone>());

            /// <summary>
            /// Initializes obstacle field from zones.
            /// </summary>
            /// <param name="zones">Buffered exclusion zones.</param>
            public ObstacleField
                (
                IEnumerable<ExclusionZone> zones
                )
            {
                // Graph caches are built once because pivot detour checks can run on many passes.
                this.Zones = new List<ExclusionZone>(zones);
                this._Nodes = new List<(Double X, Double Y)>();
                this._Adj = new List<List<(Int32 Index, Double Weight)>>();
                this.BuildGraph();
            }

            /// <summary>
            /// Gets exclusion zones.
            /// </summary>
            public List<ExclusionZone> Zones { get; }

            /// <summary>
            /// Gets a value indicating whether any zones are active.
            /// </summary>
            public Boolean Active => (this.Zones.Count > 0);

            /// <summary>
            /// Builds a field from zones.
            /// </summary>
            /// <param name="zones">Input zones.</param>
            /// <returns>Obstacle field.</returns>
            static public ObstacleField FromZones
                (
                IEnumerable<ExclusionZone> zones
                )
            {
                // This helper keeps callers close to Python's ObstacleField.from_zones.
                return new ObstacleField(zones);
            }

            /// <summary>
            /// Tests whether a straight segment is blocked by any zone.
            /// </summary>
            /// <param name="a">Start point.</param>
            /// <param name="b">End point.</param>
            /// <returns>True if blocked.</returns>
            public Boolean Blocked
                (
                (Double X, Double Y) a,
                (Double X, Double Y) b
                )
            {
                // Segment checks are delegated to per-zone buffered tests.
                foreach (ExclusionZone Zone in this.Zones)
                {
                    if (Zone.Blocks(a, b))
                    {
                        return true;
                    }
                }

                return false;
            }

            /// <summary>
            /// Computes shortest path polyline from <paramref name="a"/> to <paramref name="b"/>.
            /// </summary>
            /// <param name="a">Start point.</param>
            /// <param name="b">End point.</param>
            /// <returns>Path points including endpoints.</returns>
            public List<(Double X, Double Y)> Path
                (
                (Double X, Double Y) a,
                (Double X, Double Y) b
                )
            {
                // A straight path is returned whenever no obstacle crossing exists.
                if ((!this.Active) || (!this.Blocked(a, b)))
                {
                    return new List<(Double X, Double Y)> { a, b };
                }

                Int32 NodeCount = this._Nodes.Count;
                Int32 AIndex = NodeCount;
                Int32 BIndex = (NodeCount + 1);
                List<List<(Int32 Index, Double Weight)>> Adj = CloneAdj(this._Adj);
                Adj.Add(new List<(Int32 Index, Double Weight)>());
                Adj.Add(new List<(Int32 Index, Double Weight)>());

                for (Int32 k = 0; k < NodeCount; k += 1)
                {
                    if (this.Visible(a, this._Nodes[k]))
                    {
                        Double Dist = Distance(a, this._Nodes[k]);
                        Adj[AIndex].Add((k, Dist));
                        Adj[k].Add((AIndex, Dist));
                    }

                    if (this.Visible(b, this._Nodes[k]))
                    {
                        Double Dist = Distance(b, this._Nodes[k]);
                        Adj[BIndex].Add((k, Dist));
                        Adj[k].Add((BIndex, Dist));
                    }
                }

                if (this.Visible(a, b))
                {
                    Double Dist = Distance(a, b);
                    Adj[AIndex].Add((BIndex, Dist));
                    Adj[BIndex].Add((AIndex, Dist));
                }

                (Int32[] Prev, Double[] Distances) = Dijkstra(Adj, AIndex, BIndex);
                if (Double.IsPositiveInfinity(Distances[BIndex]))
                {
                    return new List<(Double X, Double Y)> { a, b };
                }

                List<Int32> Chain = new List<Int32> { BIndex };
                while (Chain[Chain.Count - 1] != AIndex)
                {
                    Int32 P = Prev[Chain[Chain.Count - 1]];
                    if (P < 0)
                    {
                        return new List<(Double X, Double Y)> { a, b };
                    }

                    Chain.Add(P);
                }

                Chain.Reverse();
                List<(Double X, Double Y)> Full = new List<(Double X, Double Y)>(this._Nodes);
                Full.Add(a);
                Full.Add(b);

                List<(Double X, Double Y)> Result = new List<(Double X, Double Y)>();
                foreach (Int32 Index in Chain)
                {
                    Result.Add(Full[Index]);
                }

                return Result;
            }

            /// <summary>
            /// Computes shortest-path length from <paramref name="a"/> to <paramref name="b"/>.
            /// </summary>
            /// <param name="a">Start point.</param>
            /// <param name="b">End point.</param>
            /// <returns>Route length in metres.</returns>
            public Double PathLength
                (
                (Double X, Double Y) a,
                (Double X, Double Y) b
                )
            {
                // The fast path avoids graph work for the overwhelmingly common clear-leg case.
                if ((!this.Active) || (!this.Blocked(a, b)))
                {
                    return Distance(a, b);
                }

                List<(Double X, Double Y)> Points = this.Path(a, b);
                Double Length = 0.0;
                for (Int32 Iter = 0; Iter < (Points.Count - 1); Iter += 1)
                {
                    Length += Distance(Points[Iter], Points[Iter + 1]);
                }

                return Length;
            }

            /// <summary>
            /// Builds visibility graph between buffered vertices.
            /// </summary>
            private void BuildGraph
                (
                )
            {
                // The graph is symmetric because visibility edges are undirected.
                this._Nodes.Clear();
                this._Adj.Clear();
                foreach (ExclusionZone Zone in this.Zones)
                {
                    foreach ((Double X, Double Y) Vertex in Zone.BufferRing)
                    {
                        this._Nodes.Add(Vertex);
                    }
                }

                for (Int32 Iter = 0; Iter < this._Nodes.Count; Iter += 1)
                {
                    this._Adj.Add(new List<(Int32 Index, Double Weight)>());
                }

                for (Int32 A = 0; A < this._Nodes.Count; A += 1)
                {
                    for (Int32 B = (A + 1); B < this._Nodes.Count; B += 1)
                    {
                        if (this.Visible(this._Nodes[A], this._Nodes[B]))
                        {
                            Double Dist = Distance(this._Nodes[A], this._Nodes[B]);
                            this._Adj[A].Add((B, Dist));
                            this._Adj[B].Add((A, Dist));
                        }
                    }
                }
            }

            /// <summary>
            /// Tests straight visibility between two points.
            /// </summary>
            /// <param name="a">Start point.</param>
            /// <param name="b">End point.</param>
            /// <returns>True if no zone blocks interior.</returns>
            private Boolean Visible
                (
                (Double X, Double Y) a,
                (Double X, Double Y) b
                )
            {
                // Visibility is exactly "not blocked by any zone."
                return (!this.Blocked(a, b));
            }

            /// <summary>
            /// Cached vertices for zone buffers.
            /// </summary>
            private readonly List<(Double X, Double Y)> _Nodes;

            /// <summary>
            /// Cached visibility-graph adjacency.
            /// </summary>
            private readonly List<List<(Int32 Index, Double Weight)>> _Adj;
        }

        /// <summary>
        /// Computes pivot geometry anchored at field center and maximally fitting inside the field.
        /// </summary>
        /// <param name="grid">Grid to analyze.</param>
        /// <param name="wheelbaseFt">Arm wheelbase in feet.</param>
        /// <param name="clearanceFt">Arm clearance in feet.</param>
        /// <param name="centerXy">Optional explicit center override in world metres.</param>
        /// <returns>Solved pivot geometry.</returns>
        static public PivotGeometry ComputePivotGeometry
            (
            Grid grid,
            Double wheelbaseFt,
            Double clearanceFt,
            (Double X, Double Y)? centerXy = null
            )
        {
            // The Python implementation sizes radius against invalid cells and outer grid boundary.
            Double Size = grid.GridSizeM;
            Double CenterX;
            Double CenterY;

            if (centerXy.HasValue)
            {
                CenterX = centerXy.Value.X;
                CenterY = centerXy.Value.Y;
            }
            else
            {
                Double SumX = 0.0;
                Double SumY = 0.0;
                Int32 Count = 0;
                for (Int32 By = 0; By < grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < grid.Width; Bx += 1)
                    {
                        if (grid.Valid[By, Bx])
                        {
                            (Double X, Double Y) Center = grid.CellCenterXy(Bx, By);
                            SumX += Center.X;
                            SumY += Center.Y;
                            Count += 1;
                        }
                    }
                }

                if (Count <= 0)
                {
                    throw new InvalidOperationException("Grid has no valid cells.");
                }

                CenterX = (SumX / Count);
                CenterY = (SumY / Count);
            }

            Double RInvalid = Double.PositiveInfinity;
            for (Int32 By = 0; By < grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < grid.Width; Bx += 1)
                {
                    if ((!grid.Valid[By, Bx]))
                    {
                        (Double X, Double Y) C = grid.CellCenterXy(Bx, By);
                        Double D = Math.Sqrt(((C.X - CenterX) * (C.X - CenterX)) + ((C.Y - CenterY) * (C.Y - CenterY)));
                        if (D < RInvalid)
                        {
                            RInvalid = D;
                        }
                    }
                }
            }

            Double MaxX = (grid.MinX + (grid.Width * grid.GridSizeM));
            Double MaxY = (grid.MinY + (grid.Height * grid.GridSizeM));
            Double RBorder = Math.Min(
                Math.Min((CenterX - grid.MinX), (MaxX - CenterX)),
                Math.Min((CenterY - grid.MinY), (MaxY - CenterY)));

            Double WheelbaseM = (wheelbaseFt * Constants.FEET_TO_METERS);
            Double ClearanceM = (clearanceFt * Constants.FEET_TO_METERS);
            Double HalfDiag = (Size * 0.5 * Math.Sqrt(2.0));
            Double RawRadius = (Math.Min(RInvalid, RBorder) - HalfDiag - (WheelbaseM / 2.0));
            Double Radius = Math.Max(RawRadius, (Size * MIN_RADIUS_CELLS));

            return new PivotGeometry(
                CenterX,
                CenterY,
                Radius,
                WheelbaseM,
                ClearanceM,
                grid.Zone,
                grid.IsNorthern);
        }

        /// <summary>
        /// Plans a globally optimal pivot bearing schedule by dynamic programming.
        /// </summary>
        /// <param name="grid">Field grid.</param>
        /// <param name="orderedPasses">Passes in executed route order.</param>
        /// <param name="geometry">Pivot geometry.</param>
        /// <param name="nBins">Angular bin count.</param>
        /// <param name="moveCost">Move cost in detour-equivalent passes.</param>
        /// <returns>Computed pivot plan.</returns>
        static public PivotPlan PlanPivot
            (
            Grid grid,
            IReadOnlyList<Pass> orderedPasses,
            PivotGeometry geometry,
            Int32 nBins = 720,
            Double moveCost = 25.0
            )
        {
            // The DP trades off arm moves against pass detours exactly like the Python recurrence.
            Int32 PassCount = orderedPasses.Count;
            PivotPlan Plan = new PivotPlan(geometry, nBins, moveCost);
            if (PassCount <= 0)
            {
                return Plan;
            }

            Boolean[,] Blocked = new Boolean[PassCount, nBins];
            for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
            {
                Boolean[] Row = BlockedBins(grid, geometry, orderedPasses[PassIndex], nBins);
                for (Int32 Bin = 0; Bin < nBins; Bin += 1)
                {
                    Blocked[PassIndex, Bin] = Row[Bin];
                }
            }

            Double[,] Detour = new Double[PassCount, nBins];
            for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
            {
                for (Int32 Bin = 0; Bin < nBins; Bin += 1)
                {
                    Detour[PassIndex, Bin] = (Blocked[PassIndex, Bin] ? 1.0 : 0.0);
                }
            }

            Double[,] Dp = new Double[PassCount, nBins];
            for (Int32 Bin = 0; Bin < nBins; Bin += 1)
            {
                Dp[0, Bin] = Detour[0, Bin];
            }

            for (Int32 PassIndex = 1; PassIndex < PassCount; PassIndex += 1)
            {
                Double MinPrev = Double.PositiveInfinity;
                for (Int32 Bin = 0; Bin < nBins; Bin += 1)
                {
                    MinPrev = Math.Min(MinPrev, Dp[PassIndex - 1, Bin]);
                }

                for (Int32 Bin = 0; Bin < nBins; Bin += 1)
                {
                    Double Hold = Dp[PassIndex - 1, Bin];
                    Double Move = (MinPrev + moveCost);
                    Dp[PassIndex, Bin] = (Detour[PassIndex, Bin] + Math.Min(Hold, Move));
                }
            }

            Int32[] Chosen = new Int32[PassCount];
            Int32 LastBin = 0;
            Double LastScore = Double.PositiveInfinity;
            for (Int32 Bin = 0; Bin < nBins; Bin += 1)
            {
                if (Dp[PassCount - 1, Bin] < LastScore)
                {
                    LastScore = Dp[PassCount - 1, Bin];
                    LastBin = Bin;
                }
            }

            Chosen[PassCount - 1] = LastBin;
            for (Int32 PassIndex = (PassCount - 1); PassIndex >= 1; PassIndex -= 1)
            {
                Int32 Bin = Chosen[PassIndex];
                Double HoldScore = (Detour[PassIndex, Bin] + Dp[PassIndex - 1, Bin]);
                if (Math.Abs(Dp[PassIndex, Bin] - HoldScore) <= EPSILON)
                {
                    Chosen[PassIndex - 1] = Bin;
                }
                else
                {
                    Double MinPrev = Double.PositiveInfinity;
                    for (Int32 Prev = 0; Prev < nBins; Prev += 1)
                    {
                        MinPrev = Math.Min(MinPrev, Dp[PassIndex - 1, Prev]);
                    }

                    List<Int32> Candidates = new List<Int32>();
                    for (Int32 Prev = 0; Prev < nBins; Prev += 1)
                    {
                        if (Dp[PassIndex - 1, Prev] <= (MinPrev + EPSILON))
                        {
                            Candidates.Add(Prev);
                        }
                    }

                    Int32 Best = Candidates[0];
                    Int32 BestDist = CircularBinDistance(Best, Bin, nBins);
                    foreach (Int32 Candidate in Candidates)
                    {
                        Int32 Dist = CircularBinDistance(Candidate, Bin, nBins);
                        if (Dist < BestDist)
                        {
                            Best = Candidate;
                            BestDist = Dist;
                        }
                    }

                    Chosen[PassIndex - 1] = Best;
                }
            }

            Double BinWidth = (360.0 / nBins);
            Double[] Bearings = new Double[PassCount];
            for (Int32 Iter = 0; Iter < PassCount; Iter += 1)
            {
                Bearings[Iter] = ((Chosen[Iter] + 0.5) * BinWidth);
            }

            Plan.BearingByPass = Bearings;

            Double TotalRotation = 0.0;
            Double? PreviousBearing = null;
            Int32 Start = 0;
            Int32 PositionIndex = 1;
            for (Int32 Iter = 1; Iter <= PassCount; Iter += 1)
            {
                if ((Iter == PassCount) || (Chosen[Iter] != Chosen[Start]))
                {
                    Int32 Bin = Chosen[Start];
                    Double Bearing = ((Bin + 0.5) * BinWidth);
                    Double Move = (PreviousBearing.HasValue ? CircularDistanceDeg(Bearing, PreviousBearing.Value) : 0.0);
                    TotalRotation += Move;
                    Int32 ConflictCount = 0;
                    for (Int32 PassIndex = Start; PassIndex < Iter; PassIndex += 1)
                    {
                        if (Blocked[PassIndex, Bin])
                        {
                            ConflictCount += 1;
                        }
                    }

                    Plan.Positions.Add(new PivotPosition(
                        PositionIndex,
                        (Start + 1),
                        Iter,
                        Bearing,
                        Move,
                        ConflictCount));

                    PreviousBearing = Bearing;
                    Start = Iter;
                    PositionIndex += 1;
                }
            }

            Plan.TotalRotationDeg = TotalRotation;
            for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
            {
                Int32 Bin = Chosen[PassIndex];
                if (Blocked[PassIndex, Bin])
                {
                    Plan.ConflictPasses.Add(PassIndex + 1);
                }
            }

            return Plan;
        }

        /// <summary>
        /// Builds one arm rectangle ring at a held bearing.
        /// </summary>
        /// <param name="geometry">Pivot geometry.</param>
        /// <param name="bearingDeg">Compass bearing in degrees.</param>
        /// <returns>Rectangle ring vertices in world metres.</returns>
        static public List<(Double X, Double Y)> ArmRing
            (
            PivotGeometry geometry,
            Double bearingDeg
            )
        {
            // The slight tip overshoot prevents transit legs from clipping around the extreme arm end.
            Double Rad = DegreesToRadians(bearingDeg);
            Double Dx = Math.Sin(Rad);
            Double Dy = Math.Cos(Rad);
            Double Px = Math.Cos(Rad);
            Double Py = -Math.Sin(Rad);
            Double Half = (geometry.WheelbaseM / 2.0);
            (Double TipX, Double TipY) = geometry.TipXy(bearingDeg);
            TipX += (Dx * Half);
            TipY += (Dy * Half);

            return new List<(Double X, Double Y)>
            {
                (geometry.CenterX + (Px * Half), geometry.CenterY + (Py * Half)),
                (TipX + (Px * Half), TipY + (Py * Half)),
                (TipX - (Px * Half), TipY - (Py * Half)),
                (geometry.CenterX - (Px * Half), geometry.CenterY - (Py * Half)),
            };
        }

        /// <summary>
        /// Creates per-pass obstacle fields for arm positions held by the plan.
        /// </summary>
        /// <param name="geometry">Pivot geometry.</param>
        /// <param name="plan">Pivot plan.</param>
        /// <param name="baseObstacles">Optional static obstacles to merge.</param>
        /// <param name="clearanceM">Routing clearance used to buffer arm ring.</param>
        /// <returns>Dictionary keyed by 1-based pass number.</returns>
        static public Dictionary<Int32, ObstacleField> BuildArmObstacles
            (
            PivotGeometry geometry,
            PivotPlan plan,
            ObstacleField? baseObstacles = null,
            Double clearanceM = 0.0
            )
        {
            // Every pass in a held run maps to the same obstacle field because the arm does not move within that run.
            Dictionary<Int32, ObstacleField> ByPass = new Dictionary<Int32, ObstacleField>();
            List<ExclusionZone> BaseZones = new List<ExclusionZone>();
            if (baseObstacles is not null)
            {
                BaseZones.AddRange(baseObstacles.Zones);
            }

            foreach (PivotPosition Position in plan.Positions)
            {
                List<(Double X, Double Y)> Ring = ArmRing(geometry, Position.BearingDeg);
                ExclusionZone Zone = ExclusionZone.FromRing(Ring, clearanceM);
                List<ExclusionZone> Zones = new List<ExclusionZone>(BaseZones) { Zone };
                ObstacleField Field = ObstacleField.FromZones(Zones);
                for (Int32 PassNo = Position.FirstPass; PassNo <= Position.LastPass; PassNo += 1)
                {
                    ByPass[PassNo] = Field;
                }
            }

            return ByPass;
        }

        /// <summary>
        /// Measures loaded-haul detours induced by arm obstacles per pass.
        /// </summary>
        /// <param name="orderedPasses">Passes in route order.</param>
        /// <param name="armObstacles">Per-pass obstacle mapping.</param>
        /// <param name="minDetourM">Minimum routed extra length to count as detour.</param>
        /// <returns>Tuple: routed count, extra haul metres, under-arm count, routed passes.</returns>
        static public (Int32 Routed, Double ExtraHaulM, Int32 UnderArm, List<Int32> RoutedPasses) PivotDetourStats
            (
            IReadOnlyList<Pass> orderedPasses,
            IReadOnlyDictionary<Int32, ObstacleField> armObstacles,
            Double minDetourM = 1e-3
            )
        {
            // The logic separates true reroutes from endpoints that begin/end under the arm and cannot be rerouted.
            Int32 Routed = 0;
            Int32 UnderArm = 0;
            Double Extra = 0.0;
            List<Int32> Passes = new List<Int32>();

            for (Int32 Iter = 0; Iter < orderedPasses.Count; Iter += 1)
            {
                Int32 PassNo = (Iter + 1);
                if ((!armObstacles.TryGetValue(PassNo, out ObstacleField? Field)) || (Field is null) || (!Field.Active))
                {
                    continue;
                }

                Pass Current = orderedPasses[Iter];
                (Double X, Double Y) A = Current.CutEndXy;
                (Double X, Double Y) B = Current.FillStartXy;
                if ((!Field.Blocked(A, B)))
                {
                    continue;
                }

                Double Delta = (Field.PathLength(A, B) - Distance(A, B));
                if (Delta > minDetourM)
                {
                    Routed += 1;
                    Extra += Delta;
                    Passes.Add(PassNo);
                }
                else
                {
                    UnderArm += 1;
                }
            }

            return (Routed, Extra, UnderArm, Passes);
        }

        /// <summary>
        /// Renders one PNG per pivot position showing field state at each position start.
        /// </summary>
        /// <param name="grid">Grid state and metadata.</param>
        /// <param name="orderedPasses">Passes in route order.</param>
        /// <param name="plan">Pivot schedule.</param>
        /// <param name="outDir">Output directory.</param>
        /// <param name="mode">Render mode: <c>cutfill</c> or <c>topology</c>.</param>
        /// <param name="pixelsPerBin">Pixels per grid cell.</param>
        /// <returns>Paths of per-position PNG files.</returns>
        static public List<String> RenderPivotPngs
            (
            Grid grid,
            IReadOnlyList<Pass> orderedPasses,
            PivotPlan plan,
            String outDir,
            String mode = "cutfill",
            Int32 pixelsPerBin = 6
            )
        {
            // The render snapshot is taken before each first pass in a position run, matching operator timing.
            String Folder = Path.GetFullPath(outDir);
            Directory.CreateDirectory(Folder);
            Int32 Ppb = Math.Max(1, pixelsPerBin);

            foreach (String Old in Directory.EnumerateFiles(Folder, "pivot_*.png"))
            {
                File.Delete(Old);
            }

            Double[,] Existing = Clone(grid.Existing);
            Double[,] Target = grid.Target;
            Boolean[,] Valid = grid.Valid;
            (Double VMin, Double VMax) = ComputeModeRange(Existing, Target, Valid, mode);

            Dictionary<Int32, PivotPosition> Starts = new Dictionary<Int32, PivotPosition>();
            Dictionary<Int32, Double?> PrevBearing = new Dictionary<Int32, Double?>();
            Double? Prev = null;
            foreach (PivotPosition Position in plan.Positions)
            {
                Int32 StartIndex = (Position.FirstPass - 1);
                Starts[StartIndex] = Position;
                PrevBearing[StartIndex] = Prev;
                Prev = Position.BearingDeg;
            }

            List<String> Paths = new List<String>();
            for (Int32 Iter = 0; Iter < orderedPasses.Count; Iter += 1)
            {
                if (Starts.TryGetValue(Iter, out PivotPosition? Position) && (Position is not null))
                {
                    using Image<Rgba32> Image = BuildBaseImage(Existing, Target, Valid, mode, Ppb, VMin, VMax);
                    List<Pass> RunPasses = orderedPasses
                        .Skip(Position.FirstPass - 1)
                        .Take(Position.PassCount)
                        .ToList();

                    DrawRunOps(Image, grid, RunPasses, Ppb);
                    DrawPivot(Image, grid, plan.Geometry, Position.BearingDeg, Ppb, PrevBearing[Iter]);

                    (Double CLat, Double CLon) = plan.Geometry.CenterLatLon();
                    DrawBanner(Image, new List<String>
                    {
                        String.Format(
                            CultureInfo.InvariantCulture,
                            "Pivot position {0}/{1}  bearing {2:F1} deg ({3})",
                            Position.Index,
                            plan.Positions.Count,
                            Position.BearingDeg,
                            Cardinal(Position.BearingDeg)),
                        String.Format(
                            CultureInfo.InvariantCulture,
                            "passes {0}-{1} ({2})  move {3:F1} deg{4}",
                            Position.FirstPass,
                            Position.LastPass,
                            Position.PassCount,
                            Position.MoveDeg,
                            (Position.MoveDeg == 0.0 ? "  [START]" : String.Empty)),
                        String.Format(
                            CultureInfo.InvariantCulture,
                            "radius {0:F0} ft  anchor {1:F5}, {2:F5}{3}",
                            (plan.Geometry.RadiusM / Constants.FEET_TO_METERS),
                            CLat,
                            CLon,
                            (Position.ConflictCount > 0 ? String.Format(CultureInfo.InvariantCulture, "  conflicts {0}", Position.ConflictCount) : String.Empty)),
                    }, Ppb);

                    String FileName = String.Format(
                        CultureInfo.InvariantCulture,
                        "pivot_{0:D3}_pass{1:D5}.png",
                        Position.Index,
                        Position.FirstPass);
                    String OutputPath = Path.Combine(Folder, FileName);
                    Image.Save(OutputPath);
                    Paths.Add(OutputPath);
                }

                Pass Current = orderedPasses[Iter];
                foreach (PassCell Cell in Current.CutCells)
                {
                    Existing[Cell.By, Cell.Bx] -= Cell.DepthM;
                }

                foreach (PassCell Cell in Current.FillCells)
                {
                    Existing[Cell.By, Cell.Bx] += Cell.DepthM;
                }
            }

            RenderOverview(grid, plan, Folder, Ppb);
            return Paths;
        }

        /// <summary>
        /// Builds structured CSV-equivalent pivot export data.
        /// </summary>
        /// <param name="plan">Pivot plan.</param>
        /// <returns>Structured metadata and row payload.</returns>
        static public PivotCsvData WritePivotCsv
            (
            PivotPlan plan
            )
        {
            // This preserves Python CSV content while returning typed data for API use.
            PivotCsvData Data = new PivotCsvData();
            PivotGeometry Geometry = plan.Geometry;
            (Double CLat, Double CLon) = Geometry.CenterLatLon();
            Data.Metadata["anchor_lat"] = CLat.ToString("F7", CultureInfo.InvariantCulture);
            Data.Metadata["anchor_lon"] = CLon.ToString("F7", CultureInfo.InvariantCulture);
            Data.Metadata["radius_ft"] = (Geometry.RadiusM / Constants.FEET_TO_METERS).ToString("F2", CultureInfo.InvariantCulture);
            Data.Metadata["wheelbase_ft"] = (Geometry.WheelbaseM / Constants.FEET_TO_METERS).ToString("F2", CultureInfo.InvariantCulture);
            Data.Metadata["moves"] = plan.MoveCount.ToString(CultureInfo.InvariantCulture);
            Data.Metadata["total_rotation_deg"] = plan.TotalRotationDeg.ToString("F2", CultureInfo.InvariantCulture);
            Data.Metadata["detoured_passes"] = plan.DetourCount.ToString(CultureInfo.InvariantCulture);
            Data.Metadata["move_cost_passes"] = plan.MoveCost.ToString("G", CultureInfo.InvariantCulture);

            foreach (PivotPosition Position in plan.Positions)
            {
                (Double TLat, Double TLon) = Geometry.TipLatLon(Position.BearingDeg);
                Data.Rows.Add(new PivotCsvRow
                {
                    Position = Position.Index,
                    FirstPass = Position.FirstPass,
                    LastPass = Position.LastPass,
                    PassCount = Position.PassCount,
                    BearingDeg = Position.BearingDeg,
                    Cardinal = Cardinal(Position.BearingDeg),
                    MoveDeg = Position.MoveDeg,
                    TipLat = TLat,
                    TipLon = TLon,
                    ConflictCount = Position.ConflictCount,
                });
            }

            return Data;
        }

        /// <summary>
        /// Animates leveling progress with pivot moves and optional GIF/frame outputs.
        /// </summary>
        /// <param name="grid">Grid state and metadata.</param>
        /// <param name="orderedPasses">Passes in route order.</param>
        /// <param name="plan">Pivot schedule.</param>
        /// <param name="gifPath">Optional GIF output path.</param>
        /// <param name="outDir">Optional frame directory.</param>
        /// <param name="stride">Render stride in applied passes.</param>
        /// <param name="pixelsPerBin">Pixels per grid cell.</param>
        /// <param name="mode">Render mode: <c>cutfill</c> or <c>topology</c>.</param>
        /// <param name="gifMs">Default frame hold in milliseconds.</param>
        /// <returns>Number of emitted frames.</returns>
        static public Int32 AnimatePivotLeveling
            (
            Grid grid,
            IReadOnlyList<Pass> orderedPasses,
            PivotPlan plan,
            String? gifPath = null,
            String? outDir = null,
            Int32 stride = 15,
            Int32 pixelsPerBin = 6,
            String mode = "cutfill",
            Int32 gifMs = 90
            )
        {
            // Move frames are emitted before their triggering pass so the arm reposition timing matches the planner.
            if ((gifPath is null) && (outDir is null))
            {
                return 0;
            }

            Int32 PassCount = orderedPasses.Count;
            if (PassCount <= 0)
            {
                return 0;
            }

            Int32 Ppb = Math.Max(1, pixelsPerBin);
            Double[,] Existing = Clone(grid.Existing);
            Boolean[,] Valid = grid.Valid;
            Double[,] Target = grid.Target;
            (Double VMin, Double VMax) = ComputeModeRange(Existing, Target, Valid, mode);

            Dictionary<Int32, (Int32 MoveNumber, Double PreviousBearing)> MoveInfo = new Dictionary<Int32, (Int32 MoveNumber, Double PreviousBearing)>();
            for (Int32 Iter = 0; Iter < plan.Positions.Count; Iter += 1)
            {
                PivotPosition Position = plan.Positions[Iter];
                if (Position.MoveDeg > 0.0)
                {
                    MoveInfo[Position.FirstPass] = (Iter, plan.Positions[Iter - 1].BearingDeg);
                }
            }

            Double InitialTotal = 0.0;
            Double[,] Delta = grid.Delta();
            for (Int32 By = 0; By < grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < grid.Width; Bx += 1)
                {
                    if (grid.Valid[By, Bx])
                    {
                        InitialTotal += Math.Abs(Delta[By, Bx]);
                    }
                }
            }

            if (outDir is not null)
            {
                Directory.CreateDirectory(Path.GetFullPath(outDir));
            }

            List<Image<Rgba32>> GifFrames = new List<Image<Rgba32>>();
            List<Int32> Durations = new List<Int32>();
            Int32 FrameNo = 0;

            Action<Image<Rgba32>, Int32> Emit = (Image<Rgba32> Image, Int32 HoldMs) =>
            {
                // Frames can be sent to disk, GIF, or both, from one render pass.
                if (outDir is not null)
                {
                    String FramePath = Path.Combine(Path.GetFullPath(outDir), String.Format(CultureInfo.InvariantCulture, "frame_{0:D5}.png", FrameNo));
                    Image.Save(FramePath);
                }

                if (gifPath is not null)
                {
                    Image<Rgba32> Frame = Image.Clone();
                    GifFrames.Add(Frame);
                    Durations.Add(HoldMs);
                }

                FrameNo += 1;
            };

            Func<Double> PercentDone = () =>
            {
                // Progress is measured by absolute remaining cut/fill magnitude relative to initial magnitude.
                if (InitialTotal <= 0.0)
                {
                    return 100.0;
                }

                Double Remaining = 0.0;
                for (Int32 By = 0; By < grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < grid.Width; Bx += 1)
                    {
                        if (Valid[By, Bx])
                        {
                            Remaining += Math.Abs(Existing[By, Bx] - Target[By, Bx]);
                        }
                    }
                }

                Double Done = (100.0 * (1.0 - (Remaining / InitialTotal)));
                return Math.Max(0.0, Math.Min(100.0, Done));
            };

            using (Image<Rgba32> Start = BuildBaseImage(Existing, Target, Valid, mode, Ppb, VMin, VMax))
            {
                DrawPivot(Start, grid, plan.Geometry, plan.BearingByPass[0], Ppb, null, new Rgba32(10, 10, 10, 255));
                DrawBanner(Start, new List<String>
                {
                    String.Format(CultureInfo.InvariantCulture, "Leveling start - pass 0/{0}  (0% to grade)", PassCount),
                    String.Format(CultureInfo.InvariantCulture, "pivot bearing {0:F0} deg ({1})  radius {2:F0} ft", plan.BearingByPass[0], Cardinal(plan.BearingByPass[0]), (plan.Geometry.RadiusM / Constants.FEET_TO_METERS)),
                    String.Format(CultureInfo.InvariantCulture, "plan: {0} positions, {1} moves, {2} detoured passes", plan.Positions.Count, plan.MoveCount, plan.DetourCount),
                }, Ppb);
                Emit(Start, 1500);
            }

            for (Int32 PassNo = 1; PassNo <= PassCount; PassNo += 1)
            {
                if (MoveInfo.TryGetValue(PassNo, out (Int32 MoveNumber, Double PreviousBearing) Move))
                {
                    Double NewBearing = plan.BearingByPass[PassNo - 1];
                    using Image<Rgba32> MoveImage = BuildBaseImage(Existing, Target, Valid, mode, Ppb, VMin, VMax);
                    DrawRunOps(MoveImage, grid, new List<Pass> { orderedPasses[PassNo - 1] }, Ppb);
                    DrawPivot(MoveImage, grid, plan.Geometry, NewBearing, Ppb, Move.PreviousBearing, new Rgba32(220, 0, 0, 255));
                    DrawBanner(MoveImage, new List<String>
                    {
                        String.Format(CultureInfo.InvariantCulture, "PIVOT MOVE {0}/{1}  before pass {2}/{3}  ({4:F0}% to grade)", Move.MoveNumber, plan.MoveCount, PassNo, PassCount, PercentDone()),
                        String.Format(CultureInfo.InvariantCulture, "rotate {0:F0} -> {1:F0} deg ({2:F0} deg, {3} -> {4})", Move.PreviousBearing, NewBearing, CircularDistanceDeg(NewBearing, Move.PreviousBearing), Cardinal(Move.PreviousBearing), Cardinal(NewBearing)),
                        "red = new arm position, grey = previous",
                    }, Ppb);
                    Emit(MoveImage, 900);
                }

                Pass Current = orderedPasses[PassNo - 1];
                foreach (PassCell Cell in Current.CutCells)
                {
                    Existing[Cell.By, Cell.Bx] -= Cell.DepthM;
                }

                foreach (PassCell Cell in Current.FillCells)
                {
                    Existing[Cell.By, Cell.Bx] += Cell.DepthM;
                }

                if (((PassNo % stride) == 0) || (PassNo == PassCount))
                {
                    Double Bearing = plan.BearingByPass[PassNo - 1];
                    using Image<Rgba32> StepImage = BuildBaseImage(Existing, Target, Valid, mode, Ppb, VMin, VMax);
                    DrawRunOps(StepImage, grid, new List<Pass> { Current }, Ppb);
                    DrawPivot(StepImage, grid, plan.Geometry, Bearing, Ppb, null, new Rgba32(10, 10, 10, 255));
                    DrawBanner(StepImage, new List<String>
                    {
                        String.Format(CultureInfo.InvariantCulture, "Leveling pass {0}/{1}  ({2:F0}% to grade)", PassNo, PassCount, PercentDone()),
                        String.Format(CultureInfo.InvariantCulture, "pivot bearing {0:F0} deg ({1})  position {2}/{3}", Bearing, Cardinal(Bearing), PositionOfPass(plan, PassNo), plan.Positions.Count),
                        "orange = current haul (machine at work)",
                    }, Ppb);
                    Emit(StepImage, gifMs);
                }
            }

            if ((gifPath is not null) && (GifFrames.Count > 0))
            {
                Durations[Durations.Count - 1] = 5000;
                using Image<Rgba32> Gif = GifFrames[0].Clone();
                SetGifFrameDelay(Gif.Frames.RootFrame, Durations[0]);
                for (Int32 Iter = 1; Iter < GifFrames.Count; Iter += 1)
                {
                    ImageFrame<Rgba32> Added = Gif.Frames.AddFrame(GifFrames[Iter].Frames.RootFrame);
                    SetGifFrameDelay(Added, Durations[Iter]);
                }

                Gif.Metadata.GetGifMetadata().RepeatCount = 0;
                Gif.Save(Path.GetFullPath(gifPath), new GifEncoder());
            }

            foreach (Image<Rgba32> Frame in GifFrames)
            {
                Frame.Dispose();
            }

            return FrameNo;
        }

        /// <summary>
        /// Gathers all work points disturbed by one pass.
        /// </summary>
        /// <param name="grid">Grid for cell-center mapping.</param>
        /// <param name="pass">Pass to sample.</param>
        /// <returns>World points disturbed by cut/fill cells and haul segment.</returns>
        private static List<(Double X, Double Y)> GatherWorkPoints
            (
            Grid grid,
            Pass pass
            )
        {
            // Sampling the loaded haul leg captures the long transit where arm conflicts are most likely.
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            foreach (PassCell Cell in pass.CutCells)
            {
                Points.Add(grid.CellCenterXy(Cell.Bx, Cell.By));
            }

            foreach (PassCell Cell in pass.FillCells)
            {
                Points.Add(grid.CellCenterXy(Cell.Bx, Cell.By));
            }

            (Double AX, Double AY) = pass.CutEndXy;
            (Double BX, Double BY) = pass.FillStartXy;
            Double Segment = Math.Sqrt(((BX - AX) * (BX - AX)) + ((BY - AY) * (BY - AY)));
            Int32 Samples = Math.Max(1, (Int32)(Segment / grid.GridSizeM));
            for (Int32 k = 0; k <= Samples; k += 1)
            {
                Double T = ((Double)k / Samples);
                Points.Add((AX + ((BX - AX) * T), AY + ((BY - AY) * T)));
            }

            return Points;
        }

        /// <summary>
        /// Computes blocked angular bins for one pass.
        /// </summary>
        /// <param name="grid">Grid metadata.</param>
        /// <param name="geometry">Pivot geometry.</param>
        /// <param name="pass">Pass to test.</param>
        /// <param name="nBins">Angular bins.</param>
        /// <returns>Boolean array marking blocked bins.</returns>
        private static Boolean[] BlockedBins
            (
            Grid grid,
            PivotGeometry geometry,
            Pass pass,
            Int32 nBins
            )
        {
            // Each point blocks a bearing wedge based on arm half-width and finite cell extent.
            Boolean[] Blocked = new Boolean[nBins];
            Double HalfW = ((geometry.WheelbaseM / 2.0) + geometry.ClearanceM);
            Double CellR = (grid.GridSizeM * 0.5 * Math.Sqrt(2.0));
            Double BinW = ((2.0 * Math.PI) / nBins);

            foreach ((Double X, Double Y) Point in GatherWorkPoints(grid, pass))
            {
                Double Dx = (Point.X - geometry.CenterX);
                Double Dy = (Point.Y - geometry.CenterY);
                Double Radius = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Radius > (geometry.RadiusM + CellR))
                {
                    continue;
                }

                if (Radius < 1e-6)
                {
                    for (Int32 Bin = 0; Bin < nBins; Bin += 1)
                    {
                        Blocked[Bin] = true;
                    }

                    return Blocked;
                }

                Double Angle = PositiveMod(Math.Atan2(Dx, Dy), (2.0 * Math.PI));
                Double Delta = Math.Min(((HalfW + CellR) / Radius), Math.PI);
                Int32 ILo = (Int32)Math.Floor(((Angle - Delta) / BinW));
                Int32 IHi = (Int32)Math.Ceiling(((Angle + Delta) / BinW));
                for (Int32 Iter = ILo; Iter <= IHi; Iter += 1)
                {
                    Int32 Index = Mod(Iter, nBins);
                    Blocked[Index] = true;
                }
            }

            return Blocked;
        }

        /// <summary>
        /// Computes minimum circular angular distance.
        /// </summary>
        /// <param name="a">Angle in degrees.</param>
        /// <param name="b">Angle in degrees.</param>
        /// <returns>Circular distance in degrees.</returns>
        private static Double CircularDistanceDeg
            (
            Double a,
            Double b
            )
        {
            // Distances are wrapped to treat clockwise/counter-clockwise equally.
            Double D = PositiveMod(Math.Abs(a - b), 360.0);
            return Math.Min(D, (360.0 - D));
        }

        /// <summary>
        /// Computes minimum circular bin distance.
        /// </summary>
        /// <param name="a">First bin index.</param>
        /// <param name="b">Second bin index.</param>
        /// <param name="nBins">Total bin count.</param>
        /// <returns>Wrapped bin distance.</returns>
        private static Int32 CircularBinDistance
            (
            Int32 a,
            Int32 b,
            Int32 nBins
            )
        {
            // Wrapped bin distance mirrors the Python tie-breaker used in backtracking.
            Int32 D = Math.Abs(a - b);
            return Math.Min(D, (nBins - D));
        }

        /// <summary>
        /// Converts a bearing to 16-point cardinal label.
        /// </summary>
        /// <param name="bearingDeg">Bearing in degrees.</param>
        /// <returns>16-point cardinal code.</returns>
        private static String Cardinal
            (
            Double bearingDeg
            )
        {
            // Labels and indexing match Python's cardinal helper.
            String[] Labels = new[]
            {
                "N", "NNE", "NE", "ENE",
                "E", "ESE", "SE", "SSE",
                "S", "SSW", "SW", "WSW",
                "W", "WNW", "NW", "NNW",
            };
            Int32 Index = Mod((Int32)(((PositiveMod(bearingDeg, 360.0) / 22.5) + 0.5)), 16);
            return Labels[Index];
        }

        /// <summary>
        /// Draws pivot circle, anchor, and arm overlay.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="grid">Grid metadata.</param>
        /// <param name="geometry">Pivot geometry.</param>
        /// <param name="bearingDeg">Current held bearing.</param>
        /// <param name="ppb">Pixels per bin.</param>
        /// <param name="ghostBearing">Optional previous bearing to ghost.</param>
        /// <param name="armColor">Current arm color.</param>
        private static void DrawPivot
            (
            Image<Rgba32> image,
            Grid grid,
            PivotGeometry geometry,
            Double bearingDeg,
            Int32 ppb,
            Double? ghostBearing,
            Rgba32? armColor = null
            )
        {
            // The layered overlay keeps arm annotations visible without mutating base palette logic.
            Rgba32 Arm = (armColor ?? new Rgba32(10, 10, 10, 255));
            (Double Cx, Double Cy) = WorldToPixel(grid, geometry.CenterX, geometry.CenterY, ppb);
            Double RadiusPx = ((geometry.RadiusM / grid.GridSizeM) * ppb);
            Single ArmW = (Single)Math.Max(2, Math.Round((geometry.WheelbaseM / grid.GridSizeM) * ppb));
            DrawCircleOutline(image, Cx, Cy, RadiusPx, new Rgba32(20, 60, 160, 230), Math.Max(2, (ppb / 3)));

            if (ghostBearing.HasValue)
            {
                (Double GxW, Double GyW) = geometry.TipXy(ghostBearing.Value);
                (Double Gx, Double Gy) = WorldToPixel(grid, GxW, GyW, ppb);
                DrawLine(image, Cx, Cy, Gx, Gy, new Rgba32(120, 120, 120, 150), Math.Max(1, (Int32)Math.Round(ArmW)));
            }

            (Double TxW, Double TyW) = geometry.TipXy(bearingDeg);
            (Double Tx, Double Ty) = WorldToPixel(grid, TxW, TyW, ppb);
            DrawLine(image, Cx, Cy, Tx, Ty, Arm, Math.Max(1, (Int32)Math.Round(ArmW)));

            Double Tr = Math.Max(3.0, ArmW);
            DrawCircleFilled(image, Tx, Ty, Tr, new Rgba32(255, 215, 0, 255));
            DrawCircleOutline(image, Tx, Ty, Tr, new Rgba32(10, 10, 10, 255), Math.Max(1, (Int32)(ArmW / 3.0f)));

            Double Ar = Math.Max(3.0, ArmW);
            DrawCircleFilled(image, Cx, Cy, Ar, new Rgba32(10, 10, 10, 255));
            DrawCircleOutline(image, Cx, Cy, Ar, new Rgba32(255, 255, 255, 255), Math.Max(1, (Int32)(ArmW / 3.0f)));
        }

        /// <summary>
        /// Draws faint haul segments for a run.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="grid">Grid metadata.</param>
        /// <param name="passes">Passes to overlay.</param>
        /// <param name="ppb">Pixels per bin.</param>
        private static void DrawRunOps
            (
            Image<Rgba32> image,
            Grid grid,
            IReadOnlyList<Pass> passes,
            Int32 ppb
            )
        {
            // Orange overlays expose upcoming/active haul direction while preserving map readability.
            foreach (Pass Pass in passes)
            {
                (Double AX, Double AY) = WorldToPixel(grid, Pass.CutEndXy.X, Pass.CutEndXy.Y, ppb);
                (Double BX, Double BY) = WorldToPixel(grid, Pass.FillStartXy.X, Pass.FillStartXy.Y, ppb);
                DrawLine(image, AX, AY, BX, BY, new Rgba32(255, 120, 0, 110), Math.Max(1, (ppb / 3)));
            }
        }

        /// <summary>
        /// Draws a metadata banner in the top-left corner.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="lines">Banner text lines.</param>
        /// <param name="ppb">Pixels per bin.</param>
        private static void DrawBanner
            (
            Image<Rgba32> image,
            IReadOnlyList<String> lines,
            Int32 ppb
            )
        {
            // Banner layout is approximated from text length because this pure-ImageSharp port avoids font dependencies.
            Int32 LineHeight = Math.Max(16, ((ppb * 2) + 4));
            Int32 Pad = 8;
            Int32 MaxChars = 0;
            foreach (String Line in lines)
            {
                MaxChars = Math.Max(MaxChars, Line.Length);
            }

            Int32 BoxW = ((MaxChars * 8) + (Pad * 2));
            Int32 BoxH = ((Pad * 2) + (LineHeight * lines.Count));
            DrawRectangleFilled(image, 6, 6, BoxW, BoxH, new Rgba32(255, 255, 255, 235));
            DrawRectangleOutline(image, 6, 6, BoxW, BoxH, new Rgba32(0, 0, 0, 255), 1);
        }

        /// <summary>
        /// Renders overview PNG of all planned bearings on initial cut/fill map.
        /// </summary>
        /// <param name="grid">Grid metadata.</param>
        /// <param name="plan">Pivot plan.</param>
        /// <param name="outDir">Output directory.</param>
        /// <param name="ppb">Pixels per bin.</param>
        private static void RenderOverview
            (
            Grid grid,
            PivotPlan plan,
            String outDir,
            Int32 ppb
            )
        {
            // The overview compresses full schedule intent into one map for quick sanity checks.
            Double[,] Delta = grid.Delta();
            using Image<Rgba32> Image = DiffToImage(Delta, grid.Valid, ppb);
            FadeImage(Image, 0.45);

            (Double Cx, Double Cy) = WorldToPixel(grid, plan.Geometry.CenterX, plan.Geometry.CenterY, ppb);
            Double RadiusPx = ((plan.Geometry.RadiusM / grid.GridSizeM) * ppb);
            Single ArmW = (Single)Math.Max(2, Math.Round((plan.Geometry.WheelbaseM / grid.GridSizeM) * ppb));
            Int32 Count = plan.Positions.Count;
            DrawCircleOutline(Image, Cx, Cy, RadiusPx, new Rgba32(20, 60, 160, 230), Math.Max(2, (ppb / 3)));
            foreach (PivotPosition Position in plan.Positions)
            {
                Double Fraction = ((Double)(Position.Index - 1) / Math.Max(1, (Count - 1)));
                Rgba32 Color = new Rgba32((Byte)(40 + (200 * Fraction)), 40, (Byte)(220 - (180 * Fraction)), 235);
                (Double TxW, Double TyW) = plan.Geometry.TipXy(Position.BearingDeg);
                (Double Tx, Double Ty) = WorldToPixel(grid, TxW, TyW, ppb);
                DrawLine(Image, Cx, Cy, Tx, Ty, Color, Math.Max(1, (Int32)Math.Round(ArmW)));
            }

            Double Ar = Math.Max(3.0, ArmW);
            DrawCircleFilled(Image, Cx, Cy, Ar, new Rgba32(10, 10, 10, 255));
            DrawCircleOutline(Image, Cx, Cy, Ar, new Rgba32(255, 255, 255, 255), 1);

            DrawBanner(Image, new List<String>
            {
                String.Format(CultureInfo.InvariantCulture, "All pivot positions: {0} (moves {1}, total rotation {2:F0} deg)", Count, plan.MoveCount, plan.TotalRotationDeg),
                "arm color: blue=first -> red=last",
            }, ppb);
            Image.Save(Path.Combine(outDir, "pivot_overview.png"));
        }

        /// <summary>
        /// Builds base render image for current field state.
        /// </summary>
        /// <param name="existing">Existing elevation matrix.</param>
        /// <param name="target">Target elevation matrix.</param>
        /// <param name="valid">Valid mask matrix.</param>
        /// <param name="mode">Render mode.</param>
        /// <param name="ppb">Pixels per bin.</param>
        /// <param name="vmin">Minimum topology scale.</param>
        /// <param name="vmax">Maximum topology scale.</param>
        /// <returns>RGBA image.</returns>
        private static Image<Rgba32> BuildBaseImage
            (
            Double[,] existing,
            Double[,] target,
            Boolean[,] valid,
            String mode,
            Int32 ppb,
            Double vmin,
            Double vmax
            )
        {
            // Mode selection mirrors Python: cut/fill by default, topology when requested.
            if (String.Equals(mode, "topology", StringComparison.OrdinalIgnoreCase))
            {
                return ElevationToImage(existing, valid, ppb, vmin, vmax);
            }

            Int32 Height = existing.GetLength(0);
            Int32 Width = existing.GetLength(1);
            Double[,] Diff = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Diff[By, Bx] = (existing[By, Bx] - target[By, Bx]);
                }
            }

            return DiffToImage(Diff, valid, ppb);
        }

        /// <summary>
        /// Computes vmin/vmax for topology mode.
        /// </summary>
        /// <param name="existing">Existing elevations.</param>
        /// <param name="target">Target elevations.</param>
        /// <param name="valid">Valid mask.</param>
        /// <param name="mode">Render mode.</param>
        /// <returns>Minimum and maximum value pair.</returns>
        private static (Double VMin, Double VMax) ComputeModeRange
            (
            Double[,] existing,
            Double[,] target,
            Boolean[,] valid,
            String mode
            )
        {
            // Shared scale across frames is essential in topology mode so flattening is visually meaningful.
            if ((!String.Equals(mode, "topology", StringComparison.OrdinalIgnoreCase)))
            {
                return (0.0, 0.0);
            }

            Double Min = Double.PositiveInfinity;
            Double Max = Double.NegativeInfinity;
            Int32 Height = existing.GetLength(0);
            Int32 Width = existing.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (valid[By, Bx])
                    {
                        Min = Math.Min(Min, existing[By, Bx]);
                        Min = Math.Min(Min, target[By, Bx]);
                        Max = Math.Max(Max, existing[By, Bx]);
                        Max = Math.Max(Max, target[By, Bx]);
                    }
                }
            }

            if ((!Double.IsFinite(Min)) || (!Double.IsFinite(Max)))
            {
                return (0.0, 1.0);
            }

            return (Min, Max);
        }

        /// <summary>
        /// Renders cut/fill image from existing-minus-target deltas.
        /// </summary>
        /// <param name="diff">Delta matrix.</param>
        /// <param name="valid">Valid mask.</param>
        /// <param name="ppb">Pixels per bin.</param>
        /// <returns>RGBA image.</returns>
        private static Image<Rgba32> DiffToImage
            (
            Double[,] diff,
            Boolean[,] valid,
            Int32 ppb
            )
        {
            // Y is flipped so larger grid row indices render higher, matching Python images.
            Int32 Height = diff.GetLength(0);
            Int32 Width = diff.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);
            for (Int32 By = 0; By < Height; By += 1)
            {
                Int32 Y = (Height - 1 - By);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!valid[By, Bx])
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Image[Bx, Y] = CutFillColor(diff[By, Bx]);
                }
            }

            if (ppb > 1)
            {
                Image.Mutate(Context => Context.Resize((Width * ppb), (Height * ppb), KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Renders terrain-style image from absolute elevations.
        /// </summary>
        /// <param name="elev">Elevation matrix.</param>
        /// <param name="valid">Valid mask.</param>
        /// <param name="ppb">Pixels per bin.</param>
        /// <param name="vmin">Scale minimum.</param>
        /// <param name="vmax">Scale maximum.</param>
        /// <returns>RGBA image.</returns>
        private static Image<Rgba32> ElevationToImage
            (
            Double[,] elev,
            Boolean[,] valid,
            Int32 ppb,
            Double vmin,
            Double vmax
            )
        {
            // Linear interpolation across terrain stops keeps frame-to-frame color continuity.
            Double[] Stops = new[] { 0.0, 0.25, 0.5, 0.75, 1.0 };
            Rgba32[] Colors = new[]
            {
                new Rgba32(30, 90, 40, 255),
                new Rgba32(120, 170, 70, 255),
                new Rgba32(210, 200, 120, 255),
                new Rgba32(150, 110, 70, 255),
                new Rgba32(245, 245, 245, 255),
            };

            Int32 Height = elev.GetLength(0);
            Int32 Width = elev.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);
            Double Range = ((vmax - vmin) != 0.0 ? (vmax - vmin) : 1.0);
            for (Int32 By = 0; By < Height; By += 1)
            {
                Int32 Y = (Height - 1 - By);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!valid[By, Bx])
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Double T = ((elev[By, Bx] - vmin) / Range);
                    T = Math.Max(0.0, Math.Min(1.0, T));
                    Image[Bx, Y] = InterpolateStops(T, Stops, Colors);
                }
            }

            if (ppb > 1)
            {
                Image.Mutate(Context => Context.Resize((Width * ppb), (Height * ppb), KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Maps cut/fill delta to nine-band color.
        /// </summary>
        /// <param name="diff">Existing minus target delta.</param>
        /// <returns>Palette color.</returns>
        private static Rgba32 CutFillColor
            (
            Double diff
            )
        {
            // Thresholds are copied from Python render palette for visual parity.
            if (diff < -0.80) return new Rgba32(0x80, 0x00, 0x80, 255);
            if (diff < -0.50) return new Rgba32(0x9A, 0x31, 0xFF, 255);
            if (diff < -0.05) return new Rgba32(0x00, 0x66, 0xFF, 255);
            if (diff < -0.01) return new Rgba32(0x36, 0xF5, 0xF5, 255);
            if (diff <= 0.01) return new Rgba32(0x00, 0xCD, 0x00, 255);
            if (diff <= 0.05) return new Rgba32(0xFF, 0xFF, 0x00, 255);
            if (diff <= 0.50) return new Rgba32(0xFF, 0x80, 0x00, 255);
            if (diff <= 0.80) return new Rgba32(0xFF, 0x00, 0x00, 255);
            return new Rgba32(0xB4, 0x00, 0x00, 255);
        }

        /// <summary>
        /// Performs piecewise linear color interpolation.
        /// </summary>
        /// <param name="t">Normalized value in [0, 1].</param>
        /// <param name="stops">Monotonic stop positions.</param>
        /// <param name="colors">Stop colors.</param>
        /// <returns>Interpolated color.</returns>
        private static Rgba32 InterpolateStops
            (
            Double t,
            Double[] stops,
            Rgba32[] colors
            )
        {
            // Piecewise interpolation reproduces numpy interp behavior used by Python elevation rendering.
            if (t <= stops[0])
            {
                return colors[0];
            }

            if (t >= stops[stops.Length - 1])
            {
                return colors[colors.Length - 1];
            }

            for (Int32 Iter = 0; Iter < (stops.Length - 1); Iter += 1)
            {
                Double A = stops[Iter];
                Double B = stops[Iter + 1];
                if ((t >= A) && (t <= B))
                {
                    Double U = ((t - A) / (B - A));
                    Byte R = (Byte)Math.Round((colors[Iter].R + ((colors[Iter + 1].R - colors[Iter].R) * U)));
                    Byte G = (Byte)Math.Round((colors[Iter].G + ((colors[Iter + 1].G - colors[Iter].G) * U)));
                    Byte Bl = (Byte)Math.Round((colors[Iter].B + ((colors[Iter + 1].B - colors[Iter].B) * U)));
                    return new Rgba32(R, G, Bl, 255);
                }
            }

            return colors[colors.Length - 1];
        }

        /// <summary>
        /// Lightens image toward white by blend factor.
        /// </summary>
        /// <param name="image">Image to modify.</param>
        /// <param name="keep">Base-color retain factor.</param>
        private static void FadeImage
            (
            Image<Rgba32> image,
            Double keep
            )
        {
            // Fading mirrors Python overview rendering so arm lines dominate visual attention.
            Double White = (1.0 - keep);
            image.ProcessPixelRows(Accessor =>
            {
                for (Int32 Y = 0; Y < Accessor.Height; Y += 1)
                {
                    Span<Rgba32> Row = Accessor.GetRowSpan(Y);
                    for (Int32 X = 0; X < Row.Length; X += 1)
                    {
                        Rgba32 P = Row[X];
                        Byte R = (Byte)Math.Clamp((P.R * keep) + (255.0 * White), 0.0, 255.0);
                        Byte G = (Byte)Math.Clamp((P.G * keep) + (255.0 * White), 0.0, 255.0);
                        Byte B = (Byte)Math.Clamp((P.B * keep) + (255.0 * White), 0.0, 255.0);
                        Row[X] = new Rgba32(R, G, B, P.A);
                    }
                }
            });
        }

        /// <summary>
        /// Maps world coordinate to pixel coordinate with flipped Y axis.
        /// </summary>
        /// <param name="grid">Grid metadata.</param>
        /// <param name="x">World easting.</param>
        /// <param name="y">World northing.</param>
        /// <param name="ppb">Pixels per bin.</param>
        /// <returns>Pixel coordinate.</returns>
        private static (Double X, Double Y) WorldToPixel
            (
            Grid grid,
            Double x,
            Double y,
            Int32 ppb
            )
        {
            // The coordinate transform intentionally mirrors Python render._world_to_pixel.
            Double Px = (((x - grid.MinX) / grid.GridSizeM) * ppb);
            Double Py = ((grid.Height - ((y - grid.MinY) / grid.GridSizeM)) * ppb);
            return (Px, Py);
        }

        /// <summary>
        /// Returns one-based position index containing a given pass.
        /// </summary>
        /// <param name="plan">Pivot plan.</param>
        /// <param name="pass1Based">1-based pass number.</param>
        /// <returns>Position index.</returns>
        private static Int32 PositionOfPass
            (
            PivotPlan plan,
            Int32 pass1Based
            )
        {
            // A fallback to final position is retained for defensive parity with Python helper behavior.
            foreach (PivotPosition Position in plan.Positions)
            {
                if ((Position.FirstPass <= pass1Based) && (pass1Based <= Position.LastPass))
                {
                    return Position.Index;
                }
            }

            return plan.Positions.Count;
        }

        /// <summary>
        /// Applies GIF frame delay in milliseconds.
        /// </summary>
        /// <param name="frame">Frame to modify.</param>
        /// <param name="delayMs">Delay milliseconds.</param>
        private static void SetGifFrameDelay
            (
            ImageFrame<Rgba32> frame,
            Int32 delayMs
            )
        {
            // GIF stores delay in centiseconds, so milliseconds are rounded accordingly.
            Int32 Centiseconds = Math.Max(1, (Int32)Math.Round((delayMs / 10.0)));
            frame.Metadata.GetGifMetadata().FrameDelay = Centiseconds;
        }

        /// <summary>
        /// Clones a 2D double matrix.
        /// </summary>
        /// <param name="matrix">Source matrix.</param>
        /// <returns>Cloned matrix.</returns>
        private static Double[,] Clone
            (
            Double[,] matrix
            )
        {
            // Explicit clone keeps mutation inside render/animation methods isolated from caller state.
            Int32 Height = matrix.GetLength(0);
            Int32 Width = matrix.GetLength(1);
            Double[,] Copy = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Copy[By, Bx] = matrix[By, Bx];
                }
            }

            return Copy;
        }

        /// <summary>
        /// Converts UTM coordinate to latitude/longitude.
        /// </summary>
        /// <param name="easting">UTM easting.</param>
        /// <param name="northing">UTM northing.</param>
        /// <param name="zone">UTM zone number.</param>
        /// <param name="isNorthern">UTM hemisphere flag.</param>
        /// <returns>Latitude/longitude in degrees.</returns>
        private static (Double Latitude, Double Longitude) LatLonFromUtm
            (
            Double easting,
            Double northing,
            Int32 zone,
            Boolean isNorthern
            )
        {
            // This is a direct C# port of the inverse UTM math used by Python AGD support.
            Double B = (UTM_A * (1.0 - UTM_F));
            Double E2 = (1.0 - ((B * B) / (UTM_A * UTM_A)));
            Double EPrime2 = (E2 / (1.0 - E2));

            Double X = (easting - 500000.0);
            Double Y = (northing - (isNorthern ? 0.0 : 10000000.0));
            Double M = (Y / UTM_K0);
            Double Mu = (M / (UTM_A * (1.0 - (E2 / 4.0) - (3.0 * E2 * E2 / 64.0) - (5.0 * E2 * E2 * E2 / 256.0))));
            Double E1 = ((1.0 - Math.Sqrt(1.0 - E2)) / (1.0 + Math.Sqrt(1.0 - E2)));

            Double Phi1 =
                Mu
                + ((3.0 * E1 / 2.0) - (27.0 * E1 * E1 * E1 / 32.0)) * Math.Sin(2.0 * Mu)
                + ((21.0 * E1 * E1 / 16.0) - (55.0 * Math.Pow(E1, 4) / 32.0)) * Math.Sin(4.0 * Mu)
                + (151.0 * E1 * E1 * E1 / 96.0) * Math.Sin(6.0 * Mu)
                + (1097.0 * Math.Pow(E1, 4) / 512.0) * Math.Sin(8.0 * Mu);

            Double SinP = Math.Sin(Phi1);
            Double CosP = Math.Cos(Phi1);
            Double TanP = Math.Tan(Phi1);
            Double N1 = (UTM_A / Math.Sqrt(1.0 - (E2 * SinP * SinP)));
            Double T1 = (TanP * TanP);
            Double C1 = (EPrime2 * CosP * CosP);
            Double R1 = (UTM_A * (1.0 - E2) / Math.Pow(1.0 - (E2 * SinP * SinP), 1.5));
            Double D = (X / (N1 * UTM_K0));

            Double LatRad = Phi1 - ((N1 * TanP / R1) *
                (
                    (D * D / 2.0)
                    - (5.0 + (3.0 * T1) + (10.0 * C1) - (4.0 * C1 * C1) - (9.0 * EPrime2)) * Math.Pow(D, 4) / 24.0
                    + (61.0 + (90.0 * T1) + (298.0 * C1) + (45.0 * T1 * T1) - (252.0 * EPrime2) - (3.0 * C1 * C1)) * Math.Pow(D, 6) / 720.0
                ));

            Double LonOffset = (
                D
                - (1.0 + (2.0 * T1) + C1) * Math.Pow(D, 3) / 6.0
                + (5.0 - (2.0 * C1) + (28.0 * T1) - (3.0 * C1 * C1) + (8.0 * EPrime2) + (24.0 * T1 * T1)) * Math.Pow(D, 5) / 120.0
                ) / CosP;

            Double LonOrigin = (((zone - 1) * 6) - 180 + 3);
            Double Latitude = RadiansToDegrees(LatRad);
            Double Longitude = (LonOrigin + RadiansToDegrees(LonOffset));
            return (Latitude, Longitude);
        }

        /// <summary>
        /// Ensures polygon ring has CCW winding.
        /// </summary>
        /// <param name="ring">Input ring.</param>
        /// <returns>CCW ring copy.</returns>
        private static List<(Double X, Double Y)> EnsureCcw
            (
            IReadOnlyList<(Double X, Double Y)> ring
            )
        {
            // CCW winding makes "right-side" outward normals consistent.
            List<(Double X, Double Y)> Result = new List<(Double X, Double Y)>(ring);
            if ((Result.Count >= 3) && (SignedArea(Result) < 0.0))
            {
                Result.Reverse();
            }

            return Result;
        }

        /// <summary>
        /// Computes polygon signed area.
        /// </summary>
        /// <param name="ring">Polygon ring.</param>
        /// <returns>Signed area.</returns>
        private static Double SignedArea
            (
            IReadOnlyList<(Double X, Double Y)> ring
            )
        {
            // Positive area indicates CCW winding.
            Double Area = 0.0;
            for (Int32 Iter = 0; Iter < ring.Count; Iter += 1)
            {
                (Double X1, Double Y1) = ring[Iter];
                (Double X2, Double Y2) = ring[(Iter + 1) % ring.Count];
                Area += ((X1 * Y2) - (X2 * Y1));
            }

            return (0.5 * Area);
        }

        /// <summary>
        /// Builds miter-offset outward buffer for convex CCW polygon.
        /// </summary>
        /// <param name="ring">CCW ring.</param>
        /// <param name="clearance">Buffer distance.</param>
        /// <returns>Buffered ring.</returns>
        private static List<(Double X, Double Y)> MiterBuffer
            (
            IReadOnlyList<(Double X, Double Y)> ring,
            Double clearance
            )
        {
            // Miter scale is clamped for sharp corners exactly as in Python implementation.
            if ((ring.Count < 3) || (clearance <= 0.0))
            {
                return new List<(Double X, Double Y)>(ring);
            }

            List<(Double NX, Double NY)> Normals = new List<(Double NX, Double NY)>();
            for (Int32 Iter = 0; Iter < ring.Count; Iter += 1)
            {
                (Double X1, Double Y1) = ring[Iter];
                (Double X2, Double Y2) = ring[(Iter + 1) % ring.Count];
                Double Ex = (X2 - X1);
                Double Ey = (Y2 - Y1);
                Double Len = Math.Sqrt((Ex * Ex) + (Ey * Ey));
                if (Len <= 1e-18)
                {
                    Len = 1e-18;
                }

                Normals.Add((Ey / Len, -Ex / Len));
            }

            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>();
            for (Int32 Iter = 0; Iter < ring.Count; Iter += 1)
            {
                (Double NX, Double NY) Prev = Normals[(Iter - 1 + ring.Count) % ring.Count];
                (Double NX, Double NY) Cur = Normals[Iter];
                Double Mx = (Prev.NX + Cur.NX);
                Double My = (Prev.NY + Cur.NY);
                Double MLen = Math.Sqrt((Mx * Mx) + (My * My));
                if (MLen < 1e-9)
                {
                    Mx = Cur.NX;
                    My = Cur.NY;
                    MLen = 1.0;
                }

                Mx /= MLen;
                My /= MLen;
                Double CosHalf = ((Mx * Cur.NX) + (My * Cur.NY));
                Double Scale = (clearance / Math.Max(CosHalf, 0.35));
                (Double Px, Double Py) = ring[Iter];
                Out.Add((Px + (Mx * Scale), Py + (My * Scale)));
            }

            return Out;
        }

        /// <summary>
        /// Tests point-in-ring with ray cast.
        /// </summary>
        /// <param name="x">Point x.</param>
        /// <param name="y">Point y.</param>
        /// <param name="ring">Ring vertices.</param>
        /// <returns>True if point is inside.</returns>
        private static Boolean PointInRing
            (
            Double x,
            Double y,
            IReadOnlyList<(Double X, Double Y)> ring
            )
        {
            // Boundary is treated as outside, consistent with Python's strict-ish test.
            Boolean Inside = false;
            Int32 J = (ring.Count - 1);
            for (Int32 Iter = 0; Iter < ring.Count; Iter += 1)
            {
                (Double Xi, Double Yi) = ring[Iter];
                (Double Xj, Double Yj) = ring[J];
                if ((Yi > y) != (Yj > y))
                {
                    Double XAt = (((Xj - Xi) * (y - Yi) / ((Yj - Yi) + 1e-18)) + Xi);
                    if (x < XAt)
                    {
                        Inside = (!Inside);
                    }
                }

                J = Iter;
            }

            return Inside;
        }

        /// <summary>
        /// Tests segment versus axis-aligned bounding box.
        /// </summary>
        /// <param name="a">Segment start.</param>
        /// <param name="b">Segment end.</param>
        /// <param name="box">Box tuple.</param>
        /// <returns>True if segment may intersect box.</returns>
        private static Boolean SegmentIntersectsAabb
            (
            (Double X, Double Y) a,
            (Double X, Double Y) b,
            (Double MinX, Double MinY, Double MaxX, Double MaxY) box
            )
        {
            // Slab rejects remove most clear segments before precise geometry work.
            if (((a.X < box.MinX) && (b.X < box.MinX)) || ((a.X > box.MaxX) && (b.X > box.MaxX)))
            {
                return false;
            }

            if (((a.Y < box.MinY) && (b.Y < box.MinY)) || ((a.Y > box.MaxY) && (b.Y > box.MaxY)))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Runs Dijkstra shortest path on adjacency list.
        /// </summary>
        /// <param name="adj">Adjacency list.</param>
        /// <param name="src">Source index.</param>
        /// <param name="dst">Target index.</param>
        /// <returns>Previous-node array and distance array.</returns>
        private static (Int32[] Prev, Double[] Dist) Dijkstra
            (
            IReadOnlyList<List<(Int32 Index, Double Weight)>> adj,
            Int32 src,
            Int32 dst
            )
        {
            // Priority-queue Dijkstra is used because edge weights are non-negative Euclidean lengths.
            Int32 Count = adj.Count;
            Int32[] Prev = Enumerable.Repeat(-1, Count).ToArray();
            Double[] Dist = Enumerable.Repeat(Double.PositiveInfinity, Count).ToArray();
            Dist[src] = 0.0;
            PriorityQueue<Int32, Double> Queue = new PriorityQueue<Int32, Double>();
            Queue.Enqueue(src, 0.0);

            while (Queue.Count > 0)
            {
                Queue.TryDequeue(out Int32 Node, out Double Cost);
                if (Cost > Dist[Node])
                {
                    continue;
                }

                if (Node == dst)
                {
                    break;
                }

                foreach ((Int32 Index, Double Weight) Edge in adj[Node])
                {
                    Double Next = (Cost + Edge.Weight);
                    if (Next < Dist[Edge.Index])
                    {
                        Dist[Edge.Index] = Next;
                        Prev[Edge.Index] = Node;
                        Queue.Enqueue(Edge.Index, Next);
                    }
                }
            }

            return (Prev, Dist);
        }

        /// <summary>
        /// Clones adjacency list.
        /// </summary>
        /// <param name="adj">Source adjacency list.</param>
        /// <returns>Deep-cloned adjacency list.</returns>
        private static List<List<(Int32 Index, Double Weight)>> CloneAdj
            (
            IReadOnlyList<List<(Int32 Index, Double Weight)>> adj
            )
        {
            // Cloning allows temporary endpoint nodes without mutating cached graph.
            List<List<(Int32 Index, Double Weight)>> Result = new List<List<(Int32 Index, Double Weight)>>();
            foreach (List<(Int32 Index, Double Weight)> Row in adj)
            {
                Result.Add(new List<(Int32 Index, Double Weight)>(Row));
            }

            return Result;
        }

        /// <summary>
        /// Computes Euclidean distance between two points.
        /// </summary>
        /// <param name="a">First point.</param>
        /// <param name="b">Second point.</param>
        /// <returns>Distance.</returns>
        private static Double Distance
            (
            (Double X, Double Y) a,
            (Double X, Double Y) b
            )
        {
            // Centralized distance helper keeps geometry formulas consistent.
            Double Dx = (a.X - b.X);
            Double Dy = (a.Y - b.Y);
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Converts degrees to radians.
        /// </summary>
        /// <param name="degrees">Angle in degrees.</param>
        /// <returns>Angle in radians.</returns>
        private static Double DegreesToRadians
            (
            Double degrees
            )
        {
            // Trigonometric helper keeps conversion intent explicit.
            return (degrees * Math.PI / 180.0);
        }

        /// <summary>
        /// Converts radians to degrees.
        /// </summary>
        /// <param name="radians">Angle in radians.</param>
        /// <returns>Angle in degrees.</returns>
        private static Double RadiansToDegrees
            (
            Double radians
            )
        {
            // Trigonometric helper keeps conversion intent explicit.
            return (radians * 180.0 / Math.PI);
        }

        /// <summary>
        /// Positive modulo for floating-point values.
        /// </summary>
        /// <param name="value">Value.</param>
        /// <param name="modulus">Modulus.</param>
        /// <returns>Positive remainder.</returns>
        private static Double PositiveMod
            (
            Double value,
            Double modulus
            )
        {
            // Positive modulo avoids negative wrap artifacts with bearings and angles.
            Double Result = (value % modulus);
            if (Result < 0.0)
            {
                Result += modulus;
            }

            return Result;
        }

        /// <summary>
        /// Positive modulo for integer values.
        /// </summary>
        /// <param name="value">Value.</param>
        /// <param name="modulus">Modulus.</param>
        /// <returns>Positive remainder.</returns>
        private static Int32 Mod
            (
            Int32 value,
            Int32 modulus
            )
        {
            // Positive integer modulo is used for wrapped bin indexing.
            Int32 Result = (value % modulus);
            if (Result < 0)
            {
                Result += modulus;
            }

            return Result;
        }

        /// <summary>
        /// Draws a thick line.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="x0">Start x.</param>
        /// <param name="y0">Start y.</param>
        /// <param name="x1">End x.</param>
        /// <param name="y1">End y.</param>
        /// <param name="color">Line color.</param>
        /// <param name="thickness">Line thickness.</param>
        private static void DrawLine
            (
            Image<Rgba32> image,
            Double x0,
            Double y0,
            Double x1,
            Double y1,
            Rgba32 color,
            Int32 thickness
            )
        {
            // Segment-distance rasterization keeps line width stable for any direction.
            Double MinX = (Math.Min(x0, x1) - thickness - 1.0);
            Double MaxX = (Math.Max(x0, x1) + thickness + 1.0);
            Double MinY = (Math.Min(y0, y1) - thickness - 1.0);
            Double MaxY = (Math.Max(y0, y1) + thickness + 1.0);
            Int32 X0 = Math.Max(0, (Int32)Math.Floor(MinX));
            Int32 X1 = Math.Min((image.Width - 1), (Int32)Math.Ceiling(MaxX));
            Int32 Y0 = Math.Max(0, (Int32)Math.Floor(MinY));
            Int32 Y1 = Math.Min((image.Height - 1), (Int32)Math.Ceiling(MaxY));

            Double Radius = (Math.Max(1, thickness) / 2.0);
            Double RadiusSq = (Radius * Radius);
            Double Dx = (x1 - x0);
            Double Dy = (y1 - y0);
            Double LenSq = ((Dx * Dx) + (Dy * Dy));
            if (LenSq <= EPSILON)
            {
                DrawCircleFilled(image, x0, y0, Radius, color);
                return;
            }

            for (Int32 Y = Y0; Y <= Y1; Y += 1)
            {
                for (Int32 X = X0; X <= X1; X += 1)
                {
                    Double T = ((((X - x0) * Dx) + ((Y - y0) * Dy)) / LenSq);
                    T = Math.Max(0.0, Math.Min(1.0, T));
                    Double Px = (x0 + (T * Dx));
                    Double Py = (y0 + (T * Dy));
                    Double DistSq = (((X - Px) * (X - Px)) + ((Y - Py) * (Y - Py)));
                    if (DistSq <= RadiusSq)
                    {
                        BlendPixel(image, X, Y, color);
                    }
                }
            }
        }

        /// <summary>
        /// Draws a filled circle.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="cx">Center x.</param>
        /// <param name="cy">Center y.</param>
        /// <param name="radius">Radius.</param>
        /// <param name="color">Fill color.</param>
        private static void DrawCircleFilled
            (
            Image<Rgba32> image,
            Double cx,
            Double cy,
            Double radius,
            Rgba32 color
            )
        {
            // Filled circles are used for anchor and tower markers.
            Int32 X0 = Math.Max(0, (Int32)Math.Floor(cx - radius));
            Int32 X1 = Math.Min((image.Width - 1), (Int32)Math.Ceiling(cx + radius));
            Int32 Y0 = Math.Max(0, (Int32)Math.Floor(cy - radius));
            Int32 Y1 = Math.Min((image.Height - 1), (Int32)Math.Ceiling(cy + radius));
            Double R2 = (radius * radius);
            for (Int32 Y = Y0; Y <= Y1; Y += 1)
            {
                for (Int32 X = X0; X <= X1; X += 1)
                {
                    Double Dx = (X - cx);
                    Double Dy = (Y - cy);
                    if (((Dx * Dx) + (Dy * Dy)) <= R2)
                    {
                        BlendPixel(image, X, Y, color);
                    }
                }
            }
        }

        /// <summary>
        /// Draws a circle outline.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="cx">Center x.</param>
        /// <param name="cy">Center y.</param>
        /// <param name="radius">Radius.</param>
        /// <param name="color">Outline color.</param>
        /// <param name="thickness">Outline thickness.</param>
        private static void DrawCircleOutline
            (
            Image<Rgba32> image,
            Double cx,
            Double cy,
            Double radius,
            Rgba32 color,
            Int32 thickness
            )
        {
            // Outline is rasterized as a radial band around the requested radius.
            Double Half = (Math.Max(1, thickness) / 2.0);
            Double Inner = Math.Max(0.0, (radius - Half));
            Double Outer = (radius + Half);
            Double Inner2 = (Inner * Inner);
            Double Outer2 = (Outer * Outer);
            Int32 X0 = Math.Max(0, (Int32)Math.Floor(cx - Outer));
            Int32 X1 = Math.Min((image.Width - 1), (Int32)Math.Ceiling(cx + Outer));
            Int32 Y0 = Math.Max(0, (Int32)Math.Floor(cy - Outer));
            Int32 Y1 = Math.Min((image.Height - 1), (Int32)Math.Ceiling(cy + Outer));
            for (Int32 Y = Y0; Y <= Y1; Y += 1)
            {
                for (Int32 X = X0; X <= X1; X += 1)
                {
                    Double Dx = (X - cx);
                    Double Dy = (Y - cy);
                    Double Dist2 = ((Dx * Dx) + (Dy * Dy));
                    if ((Dist2 >= Inner2) && (Dist2 <= Outer2))
                    {
                        BlendPixel(image, X, Y, color);
                    }
                }
            }
        }

        /// <summary>
        /// Draws a filled rectangle.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="x">Left pixel.</param>
        /// <param name="y">Top pixel.</param>
        /// <param name="width">Width.</param>
        /// <param name="height">Height.</param>
        /// <param name="color">Fill color.</param>
        private static void DrawRectangleFilled
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Int32 width,
            Int32 height,
            Rgba32 color
            )
        {
            // Rectangle fill supports translucent banner backgrounds.
            Int32 X0 = Math.Max(0, x);
            Int32 Y0 = Math.Max(0, y);
            Int32 X1 = Math.Min((image.Width - 1), (x + width));
            Int32 Y1 = Math.Min((image.Height - 1), (y + height));
            for (Int32 Y = Y0; Y <= Y1; Y += 1)
            {
                for (Int32 X = X0; X <= X1; X += 1)
                {
                    BlendPixel(image, X, Y, color);
                }
            }
        }

        /// <summary>
        /// Draws a rectangle outline.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="x">Left pixel.</param>
        /// <param name="y">Top pixel.</param>
        /// <param name="width">Width.</param>
        /// <param name="height">Height.</param>
        /// <param name="color">Outline color.</param>
        /// <param name="thickness">Outline thickness.</param>
        private static void DrawRectangleOutline
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Int32 width,
            Int32 height,
            Rgba32 color,
            Int32 thickness
            )
        {
            // Four edge lines produce the outline ring.
            DrawLine(image, x, y, (x + width), y, color, thickness);
            DrawLine(image, (x + width), y, (x + width), (y + height), color, thickness);
            DrawLine(image, (x + width), (y + height), x, (y + height), color, thickness);
            DrawLine(image, x, (y + height), x, y, color, thickness);
        }

        /// <summary>
        /// Blends one source pixel onto destination.
        /// </summary>
        /// <param name="image">Target image.</param>
        /// <param name="x">Pixel x.</param>
        /// <param name="y">Pixel y.</param>
        /// <param name="src">Source color.</param>
        private static void BlendPixel
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Rgba32 src
            )
        {
            // Source-over alpha blend mirrors the compositing behavior used by PIL overlays.
            if ((x < 0) || (y < 0) || (x >= image.Width) || (y >= image.Height))
            {
                return;
            }

            Rgba32 Dst = image[x, y];
            Double Sa = (src.A / 255.0);
            Double Da = (Dst.A / 255.0);
            Double OutA = (Sa + (Da * (1.0 - Sa)));
            if (OutA <= EPSILON)
            {
                image[x, y] = new Rgba32(0, 0, 0, 0);
                return;
            }

            Byte R = (Byte)Math.Clamp(Math.Round((((src.R * Sa) + (Dst.R * Da * (1.0 - Sa))) / OutA)), 0.0, 255.0);
            Byte G = (Byte)Math.Clamp(Math.Round((((src.G * Sa) + (Dst.G * Da * (1.0 - Sa))) / OutA)), 0.0, 255.0);
            Byte B = (Byte)Math.Clamp(Math.Round((((src.B * Sa) + (Dst.B * Da * (1.0 - Sa))) / OutA)), 0.0, 255.0);
            Byte A = (Byte)Math.Clamp(Math.Round((OutA * 255.0)), 0.0, 255.0);
            image[x, y] = new Rgba32(R, G, B, A);
        }
    }
}
