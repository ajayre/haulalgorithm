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
    static public class Pivot
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
        private const Double EPSILON = Constants.NUMERIC_COMPARISON_TOLERANCE;

        /// <summary>
        /// Geometry and geodesy for one center-pivot installation.
        /// </summary>
        public sealed class PivotGeometry
        {
            /// <summary>
            /// Initializes a new geometry instance.
            /// </summary>
            /// <param name="CenterX">Pivot anchor easting (UTM metres).</param>
            /// <param name="CenterY">Pivot anchor northing (UTM metres).</param>
            /// <param name="RadiusM">Arm radius (metres).</param>
            /// <param name="WheelbaseM">Arm width (metres).</param>
            /// <param name="ClearanceM">Extra routing clearance (metres).</param>
            /// <param name="Zone">UTM zone number.</param>
            /// <param name="IsNorthern">True for northern hemisphere UTM northing.</param>
            public PivotGeometry
                (
                Double CenterX,
                Double CenterY,
                Double RadiusM,
                Double WheelbaseM,
                Double ClearanceM,
                Int32 Zone,
                Boolean IsNorthern
                )
            {
                // The geometry is immutable once solved so downstream routing and rendering stay consistent.
                this.CenterX = CenterX;
                this.CenterY = CenterY;
                this.RadiusM = RadiusM;
                this.WheelbaseM = WheelbaseM;
                this.ClearanceM = ClearanceM;
                this.Zone = Zone;
                this.IsNorthern = IsNorthern;
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
            /// <param name="BearingDeg">Compass bearing (0=N, 90=E).</param>
            /// <returns>Tip easting/northing in UTM metres.</returns>
            public (Double X, Double Y) TipXy
                (
                Double BearingDeg
                )
            {
                // Compass convention uses east as sin and north as cos.
                Double Rad = DegreesToRadians(BearingDeg);
                Double X = this.CenterX + (this.RadiusM * Math.Sin(Rad));
                Double Y = this.CenterY + (this.RadiusM * Math.Cos(Rad));
                return (X, Y);
            }

            /// <summary>
            /// Computes arm tip latitude/longitude for a compass bearing.
            /// </summary>
            /// <param name="BearingDeg">Compass bearing (0=N, 90=E).</param>
            /// <returns>Tuple of latitude and longitude in degrees.</returns>
            public (Double Latitude, Double Longitude) TipLatLon
                (
                Double BearingDeg
                )
            {
                // Tip geodesy is derived from the same inverse UTM conversion as the center.
                (Double X, Double Y) Tip = this.TipXy(BearingDeg);
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
            /// <param name="Index">1-based position sequence index.</param>
            /// <param name="FirstPass">1-based first pass in this held run.</param>
            /// <param name="LastPass">1-based last pass in this held run.</param>
            /// <param name="BearingDeg">Held compass bearing in degrees.</param>
            /// <param name="MoveDeg">Rotation from previous position in degrees.</param>
            /// <param name="Conflicts">Number of passes in run that still conflict.</param>
            public PivotPosition
                (
                Int32 Index,
                Int32 FirstPass,
                Int32 LastPass,
                Double BearingDeg,
                Double MoveDeg,
                Int32 Conflicts
                )
            {
                // Positions are immutable snapshots so downstream files and images stay deterministic.
                this.Index = Index;
                this.FirstPass = FirstPass;
                this.LastPass = LastPass;
                this.BearingDeg = BearingDeg;
                this.MoveDeg = MoveDeg;
                this.ConflictCount = Conflicts;
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
            /// <param name="Geometry">Solved pivot geometry.</param>
            /// <param name="BinCount">Angular bin count for planning.</param>
            /// <param name="MoveCost">Move cost in detour-equivalent passes.</param>
            public PivotPlan
                (
                PivotGeometry Geometry,
                Int32 BinCount,
                Double MoveCost
                )
            {
                // The planner mutates collections while solving, then callers consume the finalized state.
                this.Geometry = Geometry;
                this.Positions = new List<PivotPosition>();
                this.ConflictPasses = new List<Int32>();
                this.BearingByPass = Array.Empty<Double>();
                this.BinCount = BinCount;
                this.MoveCost = MoveCost;
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
            /// <param name="Bx">Cell x index.</param>
            /// <param name="By">Cell y index.</param>
            /// <param name="DepthM">Applied depth in metres.</param>
            public PassCell
                (
                Int32 Bx,
                Int32 By,
                Double DepthM
                )
            {
                // The tuple fields are split for easy interop from serializers and tests.
                this.Bx = Bx;
                this.By = By;
                this.DepthM = DepthM;
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
            /// <param name="CutCells">Cut cells with depths.</param>
            /// <param name="FillCells">Fill cells with depths.</param>
            /// <param name="CutEndXy">Cut sweep end coordinate.</param>
            /// <param name="FillStartXy">Fill sweep start coordinate.</param>
            public Pass
                (
                IEnumerable<PassCell> CutCells,
                IEnumerable<PassCell> FillCells,
                (Double X, Double Y) CutEndXy,
                (Double X, Double Y) FillStartXy
                )
            {
                // Pivot logic only consumes disturbed cells and haul leg endpoints.
                this.CutCells = new List<PassCell>(CutCells);
                this.FillCells = new List<PassCell>(FillCells);
                this.CutEndXy = CutEndXy;
                this.FillStartXy = FillStartXy;
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
            /// <param name="GridSizeM">Cell edge length in metres.</param>
            /// <param name="MinX">World minimum easting.</param>
            /// <param name="MinY">World minimum northing.</param>
            /// <param name="Zone">UTM zone.</param>
            /// <param name="IsNorthern">UTM hemisphere flag.</param>
            /// <param name="Existing">Existing elevation matrix [by, bx].</param>
            /// <param name="Target">Target elevation matrix [by, bx].</param>
            /// <param name="Valid">Valid mask matrix [by, bx].</param>
            public Grid
                (
                Double GridSizeM,
                Double MinX,
                Double MinY,
                Int32 Zone,
                Boolean IsNorthern,
                Double[,] Existing,
                Double[,] Target,
                Boolean[,] Valid
                )
            {
                // The grid dimensions are derived once so all methods share one coordinate frame.
                this.GridSizeM = GridSizeM;
                this.MinX = MinX;
                this.MinY = MinY;
                this.Zone = Zone;
                this.IsNorthern = IsNorthern;
                this.Existing = Existing;
                this.Target = Target;
                this.Valid = Valid;
                this.Height = Existing.GetLength(0);
                this.Width = Existing.GetLength(1);
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
            /// <param name="Bx">Cell x index.</param>
            /// <param name="By">Cell y index.</param>
            /// <returns>Cell center world coordinate.</returns>
            public (Double X, Double Y) CellCenterXy
                (
                Int32 Bx,
                Int32 By
                )
            {
                // Cell centers align with Python's (index + 0.5) formulation.
                Double X = this.MinX + ((Bx + Constants.HALF_SCALE) * this.GridSizeM);
                Double Y = this.MinY + ((By + Constants.HALF_SCALE) * this.GridSizeM);
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
            /// <param name="Ring">Original CCW ring.</param>
            /// <param name="BufferRing">Buffered CCW ring.</param>
            public ExclusionZone
                (
                IReadOnlyList<(Double X, Double Y)> Ring,
                IReadOnlyList<(Double X, Double Y)> BufferRing
                )
            {
                // Bounding box is cached for fast segment reject checks in repeated routing calls.
                this.Ring = Ring;
                this.BufferRing = BufferRing;

                Double MinX = Double.PositiveInfinity;
                Double MinY = Double.PositiveInfinity;
                Double MaxX = Double.NegativeInfinity;
                Double MaxY = Double.NegativeInfinity;
                foreach ((Double X, Double Y) Point in BufferRing)
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
            /// <param name="Ring">Input ring vertices.</param>
            /// <param name="Clearance">Clearance buffer in metres.</param>
            /// <returns>Buffered exclusion zone.</returns>
            static public ExclusionZone FromRing
                (
                IReadOnlyList<(Double X, Double Y)> Ring,
                Double Clearance
                )
            {
                // Rings are normalized CCW before miter buffering to keep outward normals stable.
                List<(Double X, Double Y)> Ccw = EnsureCcw(Ring);
                List<(Double X, Double Y)> Buffered = MiterBuffer(Ccw, Clearance);
                return new ExclusionZone(Ccw, Buffered);
            }

            /// <summary>
            /// Determines whether segment <paramref name="A"/> to <paramref name="B"/> crosses buffered interior.
            /// </summary>
            /// <param name="A">Start point.</param>
            /// <param name="B">End point.</param>
            /// <param name="Samples">Interior samples along segment.</param>
            /// <returns>True when the segment passes through buffered interior.</returns>
            public Boolean Blocks
                (
                (Double X, Double Y) A,
                (Double X, Double Y) B,
                Int32 Samples = 24
                )
            {
                // The two-stage test mirrors Python: AABB reject first, sampled interior test second.
                if ((!SegmentIntersectsAabb(A, B, this.BBox)))
                {
                    return false;
                }

                for (Int32 k = 1; k < Samples; k += 1)
                {
                    Double T = ((Double)k / Samples);
                    Double X = (A.X + ((B.X - A.X) * T));
                    Double Y = (A.Y + ((B.Y - A.Y) * T));
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
            /// <param name="Zones">Buffered exclusion zones.</param>
            public ObstacleField
                (
                IEnumerable<ExclusionZone> Zones
                )
            {
                // Graph caches are built once because pivot detour checks can run on many passes.
                this.Zones = new List<ExclusionZone>(Zones);
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
            /// <param name="Zones">Input zones.</param>
            /// <returns>Obstacle field.</returns>
            static public ObstacleField FromZones
                (
                IEnumerable<ExclusionZone> Zones
                )
            {
                // This helper keeps callers close to Python's ObstacleField.from_zones.
                return new ObstacleField(Zones);
            }

            /// <summary>
            /// Tests whether a straight segment is blocked by any zone.
            /// </summary>
            /// <param name="A">Start point.</param>
            /// <param name="B">End point.</param>
            /// <returns>True if blocked.</returns>
            public Boolean Blocked
                (
                (Double X, Double Y) A,
                (Double X, Double Y) B
                )
            {
                // Segment checks are delegated to per-zone buffered tests.
                foreach (ExclusionZone Zone in this.Zones)
                {
                    if (Zone.Blocks(A, B))
                    {
                        return true;
                    }
                }

                return false;
            }

            /// <summary>
            /// Computes shortest path polyline from <paramref name="A"/> to <paramref name="B"/>.
            /// </summary>
            /// <param name="A">Start point.</param>
            /// <param name="B">End point.</param>
            /// <returns>Path points including endpoints.</returns>
            public List<(Double X, Double Y)> Path
                (
                (Double X, Double Y) A,
                (Double X, Double Y) B
                )
            {
                // A straight path is returned whenever no obstacle crossing exists.
                if ((!this.Active) || (!this.Blocked(A, B)))
                {
                    return new List<(Double X, Double Y)> { A, B };
                }

                Int32 NodeCount = this._Nodes.Count;
                Int32 AIndex = NodeCount;
                Int32 BIndex = (NodeCount + 1);
                List<List<(Int32 Index, Double Weight)>> Adj = CloneAdj(this._Adj);
                Adj.Add(new List<(Int32 Index, Double Weight)>());
                Adj.Add(new List<(Int32 Index, Double Weight)>());

                for (Int32 k = 0; k < NodeCount; k += 1)
                {
                    if (this.Visible(A, this._Nodes[k]))
                    {
                        Double Dist = Distance(A, this._Nodes[k]);
                        Adj[AIndex].Add((k, Dist));
                        Adj[k].Add((AIndex, Dist));
                    }

                    if (this.Visible(B, this._Nodes[k]))
                    {
                        Double Dist = Distance(B, this._Nodes[k]);
                        Adj[BIndex].Add((k, Dist));
                        Adj[k].Add((BIndex, Dist));
                    }
                }

                if (this.Visible(A, B))
                {
                    Double Dist = Distance(A, B);
                    Adj[AIndex].Add((BIndex, Dist));
                    Adj[BIndex].Add((AIndex, Dist));
                }

                (Int32[] Prev, Double[] Distances) = Dijkstra(Adj, AIndex, BIndex);
                if (Double.IsPositiveInfinity(Distances[BIndex]))
                {
                    return new List<(Double X, Double Y)> { A, B };
                }

                List<Int32> Chain = new List<Int32> { BIndex };
                while (Chain[Chain.Count - 1] != AIndex)
                {
                    Int32 P = Prev[Chain[Chain.Count - 1]];
                    if (P < 0)
                    {
                        return new List<(Double X, Double Y)> { A, B };
                    }

                    Chain.Add(P);
                }

                Chain.Reverse();
                List<(Double X, Double Y)> Full = new List<(Double X, Double Y)>(this._Nodes);
                Full.Add(A);
                Full.Add(B);

                List<(Double X, Double Y)> Result = new List<(Double X, Double Y)>();
                foreach (Int32 Index in Chain)
                {
                    Result.Add(Full[Index]);
                }

                return Result;
            }

            /// <summary>
            /// Computes shortest-path length from <paramref name="A"/> to <paramref name="B"/>.
            /// </summary>
            /// <param name="A">Start point.</param>
            /// <param name="B">End point.</param>
            /// <returns>Route length in metres.</returns>
            public Double PathLength
                (
                (Double X, Double Y) A,
                (Double X, Double Y) B
                )
            {
                // The fast path avoids graph work for the overwhelmingly common clear-leg case.
                if ((!this.Active) || (!this.Blocked(A, B)))
                {
                    return Distance(A, B);
                }

                List<(Double X, Double Y)> Points = this.Path(A, B);
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
            /// <param name="A">Start point.</param>
            /// <param name="B">End point.</param>
            /// <returns>True if no zone blocks interior.</returns>
            private Boolean Visible
                (
                (Double X, Double Y) A,
                (Double X, Double Y) B
                )
            {
                // Visibility is exactly "not blocked by any zone."
                return (!this.Blocked(A, B));
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
        /// <param name="Grid">Grid to analyze.</param>
        /// <param name="WheelbaseFt">Arm wheelbase in feet.</param>
        /// <param name="ClearanceFt">Arm clearance in feet.</param>
        /// <param name="CenterXy">Optional explicit center override in world metres.</param>
        /// <returns>Solved pivot geometry.</returns>
        static public PivotGeometry ComputePivotGeometry
            (
            Grid Grid,
            Double WheelbaseFt,
            Double ClearanceFt,
            (Double X, Double Y)? CenterXy = null
            )
        {
            // The Python implementation sizes radius against invalid cells and outer grid boundary.
            Double Size = Grid.GridSizeM;
            Double CenterX;
            Double CenterY;

            if (CenterXy.HasValue)
            {
                CenterX = CenterXy.Value.X;
                CenterY = CenterXy.Value.Y;
            }
            else
            {
                Double SumX = 0.0;
                Double SumY = 0.0;
                Int32 Count = 0;
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (Grid.Valid[By, Bx])
                        {
                            (Double X, Double Y) Center = Grid.CellCenterXy(Bx, By);
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
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if ((!Grid.Valid[By, Bx]))
                    {
                        (Double X, Double Y) C = Grid.CellCenterXy(Bx, By);
                        Double D = Math.Sqrt(((C.X - CenterX) * (C.X - CenterX)) + ((C.Y - CenterY) * (C.Y - CenterY)));
                        if (D < RInvalid)
                        {
                            RInvalid = D;
                        }
                    }
                }
            }

            Double MaxX = (Grid.MinX + (Grid.Width * Grid.GridSizeM));
            Double MaxY = (Grid.MinY + (Grid.Height * Grid.GridSizeM));
            Double RBorder = Math.Min(
                Math.Min((CenterX - Grid.MinX), (MaxX - CenterX)),
                Math.Min((CenterY - Grid.MinY), (MaxY - CenterY)));

            Double WheelbaseM = (WheelbaseFt * Constants.FEET_TO_METERS);
            Double ClearanceM = (ClearanceFt * Constants.FEET_TO_METERS);
            Double HalfDiag = (Size * Constants.HALF_SCALE * Math.Sqrt(2.0));
            Double RawRadius = (Math.Min(RInvalid, RBorder) - HalfDiag - (WheelbaseM / 2.0));
            Double Radius = Math.Max(RawRadius, (Size * MIN_RADIUS_CELLS));

            return new PivotGeometry(
                CenterX,
                CenterY,
                Radius,
                WheelbaseM,
                ClearanceM,
                Grid.Zone,
                Grid.IsNorthern);
        }

        /// <summary>
        /// Plans a guidance-first pivot bearing schedule by dynamic programming.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="OrderedPasses">Passes in executed route order.</param>
        /// <param name="Geometry">Pivot geometry.</param>
        /// <param name="NBins">Angular bin count.</param>
        /// <param name="MoveCost">Move cost in detour-equivalent passes.</param>
        /// <returns>Computed pivot plan.</returns>
        static public PivotPlan PlanPivot
            (
            Grid Grid,
            IReadOnlyList<Pass> OrderedPasses,
            PivotGeometry Geometry,
            Int32 NBins = 720,
            Double MoveCost = 25.0
            )
        {
            // The DP first minimizes arm/work conflicts, then minimizes reposition
            // cost among schedules with the same conflict count.
            Int32 PassCount = OrderedPasses.Count;
            PivotPlan Plan = new PivotPlan(Geometry, NBins, MoveCost);
            if (PassCount <= 0)
            {
                return Plan;
            }

            Boolean[,] Blocked = new Boolean[PassCount, NBins];
            for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
            {
                Boolean[] Row = BlockedBins(Grid, Geometry, OrderedPasses[PassIndex], NBins);
                for (Int32 Bin = 0; Bin < NBins; Bin += 1)
                {
                    Blocked[PassIndex, Bin] = Row[Bin];
                }
            }

            // Guidance safety is the primary objective.  One avoidable arm/work
            // conflict therefore costs more than moving at every pass; move cost
            // only selects the least disruptive schedule among equally clear
            // alternatives.  A conflict remains possible only when all bearings
            // are blocked for that pass.
            Double SafeMoveCost = Math.Max(0.0, MoveCost);
            Double ConflictPenalty =
                (SafeMoveCost * (PassCount + 1)) + 1.0;
            Double[,] Detour = new Double[PassCount, NBins];
            for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
            {
                for (Int32 Bin = 0; Bin < NBins; Bin += 1)
                {
                    Detour[PassIndex, Bin] = Blocked[PassIndex, Bin]
                        ? ConflictPenalty
                        : 0.0;
                }
            }

            Double[,] Dp = new Double[PassCount, NBins];
            for (Int32 Bin = 0; Bin < NBins; Bin += 1)
            {
                Dp[0, Bin] = Detour[0, Bin];
            }

            for (Int32 PassIndex = 1; PassIndex < PassCount; PassIndex += 1)
            {
                Double MinPrev = Double.PositiveInfinity;
                for (Int32 Bin = 0; Bin < NBins; Bin += 1)
                {
                    MinPrev = Math.Min(MinPrev, Dp[PassIndex - 1, Bin]);
                }

                for (Int32 Bin = 0; Bin < NBins; Bin += 1)
                {
                    Double Hold = Dp[PassIndex - 1, Bin];
                    Double Move = (MinPrev + SafeMoveCost);
                    Dp[PassIndex, Bin] = (Detour[PassIndex, Bin] + Math.Min(Hold, Move));
                }
            }

            Int32[] Chosen = new Int32[PassCount];
            Int32 LastBin = 0;
            Double LastScore = Double.PositiveInfinity;
            for (Int32 Bin = 0; Bin < NBins; Bin += 1)
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
                    for (Int32 Prev = 0; Prev < NBins; Prev += 1)
                    {
                        MinPrev = Math.Min(MinPrev, Dp[PassIndex - 1, Prev]);
                    }

                    List<Int32> Candidates = new List<Int32>();
                    for (Int32 Prev = 0; Prev < NBins; Prev += 1)
                    {
                        if (Dp[PassIndex - 1, Prev] <= (MinPrev + EPSILON))
                        {
                            Candidates.Add(Prev);
                        }
                    }

                    Int32 Best = Candidates[0];
                    Int32 BestDist = CircularBinDistance(Best, Bin, NBins);
                    foreach (Int32 Candidate in Candidates)
                    {
                        Int32 Dist = CircularBinDistance(Candidate, Bin, NBins);
                        if (Dist < BestDist)
                        {
                            Best = Candidate;
                            BestDist = Dist;
                        }
                    }

                    Chosen[PassIndex - 1] = Best;
                }
            }

            Double BinWidth = (Constants.DEGREES_PER_TURN / NBins);
            Double[] Bearings = new Double[PassCount];
            for (Int32 Iter = 0; Iter < PassCount; Iter += 1)
            {
                Bearings[Iter] = ((Chosen[Iter] + Constants.HALF_SCALE) * BinWidth);
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
                    Double Bearing = ((Bin + Constants.HALF_SCALE) * BinWidth);
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
        /// <param name="Geometry">Pivot geometry.</param>
        /// <param name="BearingDeg">Compass bearing in degrees.</param>
        /// <returns>Rectangle ring vertices in world metres.</returns>
        static public List<(Double X, Double Y)> ArmRing
            (
            PivotGeometry Geometry,
            Double BearingDeg
            )
        {
            // The slight tip overshoot prevents transit legs from clipping around the extreme arm end.
            Double Rad = DegreesToRadians(BearingDeg);
            Double Dx = Math.Sin(Rad);
            Double Dy = Math.Cos(Rad);
            Double Px = Math.Cos(Rad);
            Double Py = -Math.Sin(Rad);
            Double Half = (Geometry.WheelbaseM / 2.0);
            (Double TipX, Double TipY) = Geometry.TipXy(BearingDeg);
            TipX += (Dx * Half);
            TipY += (Dy * Half);

            return new List<(Double X, Double Y)>
            {
                (Geometry.CenterX + (Px * Half), Geometry.CenterY + (Py * Half)),
                (TipX + (Px * Half), TipY + (Py * Half)),
                (TipX - (Px * Half), TipY - (Py * Half)),
                (Geometry.CenterX - (Px * Half), Geometry.CenterY - (Py * Half)),
            };
        }

        /// <summary>
        /// Creates per-pass obstacle fields for arm positions held by the plan.
        /// </summary>
        /// <param name="Geometry">Pivot geometry.</param>
        /// <param name="Plan">Pivot plan.</param>
        /// <param name="BaseObstacles">Optional static obstacles to merge.</param>
        /// <param name="ClearanceM">Routing clearance used to buffer arm ring.</param>
        /// <returns>Dictionary keyed by 1-based pass number.</returns>
        static public Dictionary<Int32, ObstacleField> BuildArmObstacles
            (
            PivotGeometry Geometry,
            PivotPlan Plan,
            ObstacleField? BaseObstacles = null,
            Double ClearanceM = 0.0
            )
        {
            // Every pass in a held run maps to the same obstacle field because the arm does not move within that run.
            Dictionary<Int32, ObstacleField> ByPass = new Dictionary<Int32, ObstacleField>();
            List<ExclusionZone> BaseZones = new List<ExclusionZone>();
            if (BaseObstacles is not null)
            {
                BaseZones.AddRange(BaseObstacles.Zones);
            }

            foreach (PivotPosition Position in Plan.Positions)
            {
                List<(Double X, Double Y)> Ring = ArmRing(Geometry, Position.BearingDeg);
                ExclusionZone Zone = ExclusionZone.FromRing(Ring, ClearanceM);
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
        /// <param name="OrderedPasses">Passes in route order.</param>
        /// <param name="ArmObstacles">Per-pass obstacle mapping.</param>
        /// <param name="MinDetourM">Minimum routed extra length to count as detour.</param>
        /// <returns>Tuple: routed count, extra haul metres, under-arm count, routed passes.</returns>
        static public (Int32 Routed, Double ExtraHaulM, Int32 UnderArm, List<Int32> RoutedPasses) PivotDetourStats
            (
            IReadOnlyList<Pass> OrderedPasses,
            IReadOnlyDictionary<Int32, ObstacleField> ArmObstacles,
            Double MinDetourM = Constants.PATH_JOIN_COMPARISON_TOLERANCE
            )
        {
            // The logic separates true reroutes from endpoints that begin/end under the arm and cannot be rerouted.
            Int32 Routed = 0;
            Int32 UnderArm = 0;
            Double Extra = 0.0;
            List<Int32> Passes = new List<Int32>();

            for (Int32 Iter = 0; Iter < OrderedPasses.Count; Iter += 1)
            {
                Int32 PassNo = (Iter + 1);
                if ((!ArmObstacles.TryGetValue(PassNo, out ObstacleField? Field)) || (Field is null) || (!Field.Active))
                {
                    continue;
                }

                Pass Current = OrderedPasses[Iter];
                (Double X, Double Y) A = Current.CutEndXy;
                (Double X, Double Y) B = Current.FillStartXy;
                if ((!Field.Blocked(A, B)))
                {
                    continue;
                }

                Double Delta = (Field.PathLength(A, B) - Distance(A, B));
                if (Delta > MinDetourM)
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
        /// <param name="Grid">Grid state and metadata.</param>
        /// <param name="OrderedPasses">Passes in route order.</param>
        /// <param name="Plan">Pivot schedule.</param>
        /// <param name="OutDir">Output directory.</param>
        /// <param name="Mode">Render mode: <c>cutfill</c> or <c>topology</c>.</param>
        /// <param name="PixelsPerBin">Pixels per grid cell.</param>
        /// <returns>Paths of per-position PNG files.</returns>
        static public List<String> RenderPivotPngs
            (
            Grid Grid,
            IReadOnlyList<Pass> OrderedPasses,
            PivotPlan Plan,
            String OutDir,
            String Mode = "cutfill",
            Int32 PixelsPerBin = 6
            )
        {
            // The render snapshot is taken before each first pass in a position run, matching operator timing.
            String Folder = Path.GetFullPath(OutDir);
            Directory.CreateDirectory(Folder);
            Int32 Ppb = Math.Max(1, PixelsPerBin);

            foreach (String Old in Directory.EnumerateFiles(Folder, "pivot_*.png"))
            {
                File.Delete(Old);
            }

            Double[,] Existing = Clone(Grid.Existing);
            Double[,] Target = Grid.Target;
            Boolean[,] Valid = Grid.Valid;
            (Double VMin, Double VMax) = ComputeModeRange(Existing, Target, Valid, Mode);

            Dictionary<Int32, PivotPosition> Starts = new Dictionary<Int32, PivotPosition>();
            Dictionary<Int32, Double?> PrevBearing = new Dictionary<Int32, Double?>();
            Double? Prev = null;
            foreach (PivotPosition Position in Plan.Positions)
            {
                Int32 StartIndex = (Position.FirstPass - 1);
                Starts[StartIndex] = Position;
                PrevBearing[StartIndex] = Prev;
                Prev = Position.BearingDeg;
            }

            List<String> Paths = new List<String>();
            for (Int32 Iter = 0; Iter < OrderedPasses.Count; Iter += 1)
            {
                if (Starts.TryGetValue(Iter, out PivotPosition? Position) && (Position is not null))
                {
                    using Image<Rgba32> Image = BuildBaseImage(Existing, Target, Valid, Mode, Ppb, VMin, VMax);
                    List<Pass> RunPasses = OrderedPasses
                        .Skip(Position.FirstPass - 1)
                        .Take(Position.PassCount)
                        .ToList();

                    DrawRunOps(Image, Grid, RunPasses, Ppb);
                    DrawPivot(Image, Grid, Plan.Geometry, Position.BearingDeg, Ppb, PrevBearing[Iter]);

                    (Double CLat, Double CLon) = Plan.Geometry.CenterLatLon();
                    DrawBanner(Image, new List<String>
                    {
                        String.Format(
                            CultureInfo.InvariantCulture,
                            "Pivot position {0}/{1}  bearing {2:F1} deg ({3})",
                            Position.Index,
                            Plan.Positions.Count,
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
                            (Plan.Geometry.RadiusM / Constants.FEET_TO_METERS),
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

                Pass Current = OrderedPasses[Iter];
                foreach (PassCell Cell in Current.CutCells)
                {
                    Existing[Cell.By, Cell.Bx] -= Cell.DepthM;
                }

                foreach (PassCell Cell in Current.FillCells)
                {
                    Existing[Cell.By, Cell.Bx] += Cell.DepthM;
                }
            }

            RenderOverview(Grid, Plan, Folder, Ppb);
            return Paths;
        }

        /// <summary>
        /// Builds structured CSV-equivalent pivot export data.
        /// </summary>
        /// <param name="Plan">Pivot plan.</param>
        /// <returns>Structured metadata and row payload.</returns>
        static public PivotCsvData WritePivotCsv
            (
            PivotPlan Plan
            )
        {
            // This preserves Python CSV content while returning typed data for API use.
            PivotCsvData Data = new PivotCsvData();
            PivotGeometry Geometry = Plan.Geometry;
            (Double CLat, Double CLon) = Geometry.CenterLatLon();
            Data.Metadata["anchor_lat"] = CLat.ToString("F7", CultureInfo.InvariantCulture);
            Data.Metadata["anchor_lon"] = CLon.ToString("F7", CultureInfo.InvariantCulture);
            Data.Metadata["radius_ft"] = (Geometry.RadiusM / Constants.FEET_TO_METERS).ToString("F2", CultureInfo.InvariantCulture);
            Data.Metadata["wheelbase_ft"] = (Geometry.WheelbaseM / Constants.FEET_TO_METERS).ToString("F2", CultureInfo.InvariantCulture);
            Data.Metadata["moves"] = Plan.MoveCount.ToString(CultureInfo.InvariantCulture);
            Data.Metadata["total_rotation_deg"] = Plan.TotalRotationDeg.ToString("F2", CultureInfo.InvariantCulture);
            Data.Metadata["detoured_passes"] = Plan.DetourCount.ToString(CultureInfo.InvariantCulture);
            Data.Metadata["move_cost_passes"] = Plan.MoveCost.ToString("G", CultureInfo.InvariantCulture);

            foreach (PivotPosition Position in Plan.Positions)
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
        /// <param name="Grid">Grid state and metadata.</param>
        /// <param name="OrderedPasses">Passes in route order.</param>
        /// <param name="Plan">Pivot schedule.</param>
        /// <param name="GifPath">Optional GIF output path.</param>
        /// <param name="OutDir">Optional frame directory.</param>
        /// <param name="Stride">Render stride in applied passes.</param>
        /// <param name="PixelsPerBin">Pixels per grid cell.</param>
        /// <param name="Mode">Render mode: <c>cutfill</c> or <c>topology</c>.</param>
        /// <param name="GifMs">Default frame hold in milliseconds.</param>
        /// <returns>Number of emitted frames.</returns>
        static public Int32 AnimatePivotLeveling
            (
            Grid Grid,
            IReadOnlyList<Pass> OrderedPasses,
            PivotPlan Plan,
            String? GifPath = null,
            String? OutDir = null,
            Int32 Stride = 15,
            Int32 PixelsPerBin = 6,
            String Mode = "cutfill",
            Int32 GifMs = 90
            )
        {
            // Move frames are emitted before their triggering pass so the arm reposition timing matches the planner.
            if ((GifPath is null) && (OutDir is null))
            {
                return 0;
            }

            Int32 PassCount = OrderedPasses.Count;
            if (PassCount <= 0)
            {
                return 0;
            }

            Int32 Ppb = Math.Max(1, PixelsPerBin);
            Double[,] Existing = Clone(Grid.Existing);
            Boolean[,] Valid = Grid.Valid;
            Double[,] Target = Grid.Target;
            (Double VMin, Double VMax) = ComputeModeRange(Existing, Target, Valid, Mode);

            Dictionary<Int32, (Int32 MoveNumber, Double PreviousBearing)> MoveInfo = new Dictionary<Int32, (Int32 MoveNumber, Double PreviousBearing)>();
            for (Int32 Iter = 0; Iter < Plan.Positions.Count; Iter += 1)
            {
                PivotPosition Position = Plan.Positions[Iter];
                if (Position.MoveDeg > 0.0)
                {
                    MoveInfo[Position.FirstPass] = (Iter, Plan.Positions[Iter - 1].BearingDeg);
                }
            }

            Double InitialTotal = 0.0;
            Double[,] Delta = Grid.Delta();
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx])
                    {
                        InitialTotal += Math.Abs(Delta[By, Bx]);
                    }
                }
            }

            if (OutDir is not null)
            {
                Directory.CreateDirectory(Path.GetFullPath(OutDir));
            }

            List<Image<Rgba32>> GifFrames = new List<Image<Rgba32>>();
            List<Int32> Durations = new List<Int32>();
            Int32 FrameNo = 0;

            Action<Image<Rgba32>, Int32> Emit = (Image<Rgba32> Image, Int32 HoldMs) =>
            {
                // Frames can be sent to disk, GIF, or both, from one render pass.
                if (OutDir is not null)
                {
                    String FramePath = Path.Combine(Path.GetFullPath(OutDir), String.Format(CultureInfo.InvariantCulture, "frame_{0:D5}.png", FrameNo));
                    Image.Save(FramePath);
                }

                if (GifPath is not null)
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
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
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

            using (Image<Rgba32> Start = BuildBaseImage(Existing, Target, Valid, Mode, Ppb, VMin, VMax))
            {
                DrawPivot(Start, Grid, Plan.Geometry, Plan.BearingByPass[0], Ppb, null, new Rgba32(10, 10, 10, Constants.MAX_COLOR_CHANNEL));
                DrawBanner(Start, new List<String>
                {
                    String.Format(CultureInfo.InvariantCulture, "Leveling start - pass 0/{0}  (0% to grade)", PassCount),
                    String.Format(CultureInfo.InvariantCulture, "pivot bearing {0:F0} deg ({1})  radius {2:F0} ft", Plan.BearingByPass[0], Cardinal(Plan.BearingByPass[0]), (Plan.Geometry.RadiusM / Constants.FEET_TO_METERS)),
                    String.Format(CultureInfo.InvariantCulture, "plan: {0} positions, {1} moves, {2} detoured passes", Plan.Positions.Count, Plan.MoveCount, Plan.DetourCount),
                }, Ppb);
                Emit(Start, 1500);
            }

            for (Int32 PassNo = 1; PassNo <= PassCount; PassNo += 1)
            {
                if (MoveInfo.TryGetValue(PassNo, out (Int32 MoveNumber, Double PreviousBearing) Move))
                {
                    Double NewBearing = Plan.BearingByPass[PassNo - 1];
                    using Image<Rgba32> MoveImage = BuildBaseImage(Existing, Target, Valid, Mode, Ppb, VMin, VMax);
                    DrawRunOps(MoveImage, Grid, new List<Pass> { OrderedPasses[PassNo - 1] }, Ppb);
                    DrawPivot(MoveImage, Grid, Plan.Geometry, NewBearing, Ppb, Move.PreviousBearing, new Rgba32(220, 0, 0, Constants.MAX_COLOR_CHANNEL));
                    DrawBanner(MoveImage, new List<String>
                    {
                        String.Format(CultureInfo.InvariantCulture, "PIVOT MOVE {0}/{1}  before pass {2}/{3}  ({4:F0}% to grade)", Move.MoveNumber, Plan.MoveCount, PassNo, PassCount, PercentDone()),
                        String.Format(CultureInfo.InvariantCulture, "rotate {0:F0} -> {1:F0} deg ({2:F0} deg, {3} -> {4})", Move.PreviousBearing, NewBearing, CircularDistanceDeg(NewBearing, Move.PreviousBearing), Cardinal(Move.PreviousBearing), Cardinal(NewBearing)),
                        "red = new arm position, grey = previous",
                    }, Ppb);
                    Emit(MoveImage, 900);
                }

                Pass Current = OrderedPasses[PassNo - 1];
                foreach (PassCell Cell in Current.CutCells)
                {
                    Existing[Cell.By, Cell.Bx] -= Cell.DepthM;
                }

                foreach (PassCell Cell in Current.FillCells)
                {
                    Existing[Cell.By, Cell.Bx] += Cell.DepthM;
                }

                if (((PassNo % Stride) == 0) || (PassNo == PassCount))
                {
                    Double Bearing = Plan.BearingByPass[PassNo - 1];
                    using Image<Rgba32> StepImage = BuildBaseImage(Existing, Target, Valid, Mode, Ppb, VMin, VMax);
                    DrawRunOps(StepImage, Grid, new List<Pass> { Current }, Ppb);
                    DrawPivot(StepImage, Grid, Plan.Geometry, Bearing, Ppb, null, new Rgba32(10, 10, 10, Constants.MAX_COLOR_CHANNEL));
                    DrawBanner(StepImage, new List<String>
                    {
                        String.Format(CultureInfo.InvariantCulture, "Leveling pass {0}/{1}  ({2:F0}% to grade)", PassNo, PassCount, PercentDone()),
                        String.Format(CultureInfo.InvariantCulture, "pivot bearing {0:F0} deg ({1})  position {2}/{3}", Bearing, Cardinal(Bearing), PositionOfPass(Plan, PassNo), Plan.Positions.Count),
                        "orange = current haul (machine at work)",
                    }, Ppb);
                    Emit(StepImage, GifMs);
                }
            }

            if ((GifPath is not null) && (GifFrames.Count > 0))
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
                Gif.Save(Path.GetFullPath(GifPath), new GifEncoder());
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
        /// <param name="Grid">Grid for cell-center mapping.</param>
        /// <param name="Pass">Pass to sample.</param>
        /// <returns>World points disturbed by cut/fill cells and haul segment.</returns>
        private static List<(Double X, Double Y)> GatherWorkPoints
            (
            Grid Grid,
            Pass Pass
            )
        {
            // Sampling the loaded haul leg captures the long transit where arm conflicts are most likely.
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            foreach (PassCell Cell in Pass.CutCells)
            {
                Points.Add(Grid.CellCenterXy(Cell.Bx, Cell.By));
            }

            foreach (PassCell Cell in Pass.FillCells)
            {
                Points.Add(Grid.CellCenterXy(Cell.Bx, Cell.By));
            }

            (Double AX, Double AY) = Pass.CutEndXy;
            (Double BX, Double BY) = Pass.FillStartXy;
            Double Segment = Math.Sqrt(((BX - AX) * (BX - AX)) + ((BY - AY) * (BY - AY)));
            Int32 Samples = Math.Max(1, (Int32)(Segment / Grid.GridSizeM));
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
        /// <param name="Grid">Grid metadata.</param>
        /// <param name="Geometry">Pivot geometry.</param>
        /// <param name="Pass">Pass to test.</param>
        /// <param name="NBins">Angular bins.</param>
        /// <returns>Boolean array marking blocked bins.</returns>
        private static Boolean[] BlockedBins
            (
            Grid Grid,
            PivotGeometry Geometry,
            Pass Pass,
            Int32 NBins
            )
        {
            // Each point blocks a bearing wedge based on arm half-width and finite cell extent.
            Boolean[] Blocked = new Boolean[NBins];
            Double HalfW = ((Geometry.WheelbaseM / 2.0) + Geometry.ClearanceM);
            Double CellR = (Grid.GridSizeM * Constants.HALF_SCALE * Math.Sqrt(2.0));
            Double BinW = ((2.0 * Math.PI) / NBins);

            foreach ((Double X, Double Y) Point in GatherWorkPoints(Grid, Pass))
            {
                Double Dx = (Point.X - Geometry.CenterX);
                Double Dy = (Point.Y - Geometry.CenterY);
                Double Radius = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Radius > (Geometry.RadiusM + CellR))
                {
                    continue;
                }

                if (Radius < Constants.GEOMETRY_COMPARISON_TOLERANCE)
                {
                    for (Int32 Bin = 0; Bin < NBins; Bin += 1)
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
                    Int32 Index = Mod(Iter, NBins);
                    Blocked[Index] = true;
                }
            }

            return Blocked;
        }

        /// <summary>
        /// Computes minimum circular angular distance.
        /// </summary>
        /// <param name="A">Angle in degrees.</param>
        /// <param name="B">Angle in degrees.</param>
        /// <returns>Circular distance in degrees.</returns>
        private static Double CircularDistanceDeg
            (
            Double A,
            Double B
            )
        {
            // Distances are wrapped to treat clockwise/counter-clockwise equally.
            Double D = PositiveMod(Math.Abs(A - B), Constants.DEGREES_PER_TURN);
            return Math.Min(D, (Constants.DEGREES_PER_TURN - D));
        }

        /// <summary>
        /// Computes minimum circular bin distance.
        /// </summary>
        /// <param name="A">First bin index.</param>
        /// <param name="B">Second bin index.</param>
        /// <param name="NBins">Total bin count.</param>
        /// <returns>Wrapped bin distance.</returns>
        private static Int32 CircularBinDistance
            (
            Int32 A,
            Int32 B,
            Int32 NBins
            )
        {
            // Wrapped bin distance mirrors the Python tie-breaker used in backtracking.
            Int32 D = Math.Abs(A - B);
            return Math.Min(D, (NBins - D));
        }

        /// <summary>
        /// Converts a bearing to 16-point cardinal label.
        /// </summary>
        /// <param name="BearingDeg">Bearing in degrees.</param>
        /// <returns>16-point cardinal code.</returns>
        private static String Cardinal
            (
            Double BearingDeg
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
            Int32 Index = Mod((Int32)(((PositiveMod(BearingDeg, Constants.DEGREES_PER_TURN) / 22.5) + Constants.HALF_SCALE)), 16);
            return Labels[Index];
        }

        /// <summary>
        /// Draws pivot circle, anchor, and arm overlay.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="Grid">Grid metadata.</param>
        /// <param name="Geometry">Pivot geometry.</param>
        /// <param name="BearingDeg">Current held bearing.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        /// <param name="GhostBearing">Optional previous bearing to ghost.</param>
        /// <param name="ArmColor">Current arm color.</param>
        private static void DrawPivot
            (
            Image<Rgba32> Image,
            Grid Grid,
            PivotGeometry Geometry,
            Double BearingDeg,
            Int32 Ppb,
            Double? GhostBearing,
            Rgba32? ArmColor = null
            )
        {
            // The layered overlay keeps arm annotations visible without mutating base palette logic.
            Rgba32 Arm = (ArmColor ?? new Rgba32(10, 10, 10, Constants.MAX_COLOR_CHANNEL));
            (Double Cx, Double Cy) = WorldToPixel(Grid, Geometry.CenterX, Geometry.CenterY, Ppb);
            Double RadiusPx = ((Geometry.RadiusM / Grid.GridSizeM) * Ppb);
            Single ArmW = (Single)Math.Max(2, Math.Round((Geometry.WheelbaseM / Grid.GridSizeM) * Ppb));
            DrawCircleOutline(Image, Cx, Cy, RadiusPx, new Rgba32(20, 60, 160, 230), Math.Max(2, (Ppb / 3)));

            if (GhostBearing.HasValue)
            {
                (Double GxW, Double GyW) = Geometry.TipXy(GhostBearing.Value);
                (Double Gx, Double Gy) = WorldToPixel(Grid, GxW, GyW, Ppb);
                DrawLine(Image, Cx, Cy, Gx, Gy, new Rgba32(120, 120, 120, 150), Math.Max(1, (Int32)Math.Round(ArmW)));
            }

            (Double TxW, Double TyW) = Geometry.TipXy(BearingDeg);
            (Double Tx, Double Ty) = WorldToPixel(Grid, TxW, TyW, Ppb);
            DrawLine(Image, Cx, Cy, Tx, Ty, Arm, Math.Max(1, (Int32)Math.Round(ArmW)));

            Double Tr = Math.Max(3.0, ArmW);
            DrawCircleFilled(Image, Tx, Ty, Tr, new Rgba32(Constants.MAX_COLOR_CHANNEL, 215, 0, Constants.MAX_COLOR_CHANNEL));
            DrawCircleOutline(Image, Tx, Ty, Tr, new Rgba32(10, 10, 10, Constants.MAX_COLOR_CHANNEL), Math.Max(1, (Int32)(ArmW / 3.0f)));

            Double Ar = Math.Max(3.0, ArmW);
            DrawCircleFilled(Image, Cx, Cy, Ar, new Rgba32(10, 10, 10, Constants.MAX_COLOR_CHANNEL));
            DrawCircleOutline(Image, Cx, Cy, Ar, new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL), Math.Max(1, (Int32)(ArmW / 3.0f)));
        }

        /// <summary>
        /// Draws faint haul segments for a run.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="Grid">Grid metadata.</param>
        /// <param name="Passes">Passes to overlay.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        private static void DrawRunOps
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            Int32 Ppb
            )
        {
            // Orange overlays expose upcoming/active haul direction while preserving map readability.
            foreach (Pass Pass in Passes)
            {
                (Double AX, Double AY) = WorldToPixel(Grid, Pass.CutEndXy.X, Pass.CutEndXy.Y, Ppb);
                (Double BX, Double BY) = WorldToPixel(Grid, Pass.FillStartXy.X, Pass.FillStartXy.Y, Ppb);
                DrawLine(Image, AX, AY, BX, BY, new Rgba32(Constants.MAX_COLOR_CHANNEL, 120, 0, 110), Math.Max(1, (Ppb / 3)));
            }
        }

        /// <summary>
        /// Draws a metadata banner in the top-left corner.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="Lines">Banner text lines.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        private static void DrawBanner
            (
            Image<Rgba32> Image,
            IReadOnlyList<String> Lines,
            Int32 Ppb
            )
        {
            // Banner layout is approximated from text length because this pure-ImageSharp port avoids font dependencies.
            Int32 LineHeight = Math.Max(16, ((Ppb * 2) + 4));
            Int32 Pad = 8;
            Int32 MaxChars = 0;
            foreach (String Line in Lines)
            {
                MaxChars = Math.Max(MaxChars, Line.Length);
            }

            Int32 BoxW = ((MaxChars * 8) + (Pad * 2));
            Int32 BoxH = ((Pad * 2) + (LineHeight * Lines.Count));
            DrawRectangleFilled(Image, 6, 6, BoxW, BoxH, new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, 235));
            DrawRectangleOutline(Image, 6, 6, BoxW, BoxH, new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL), 1);
        }

        /// <summary>
        /// Renders overview PNG of all planned bearings on initial cut/fill map.
        /// </summary>
        /// <param name="Grid">Grid metadata.</param>
        /// <param name="Plan">Pivot plan.</param>
        /// <param name="OutDir">Output directory.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        private static void RenderOverview
            (
            Grid Grid,
            PivotPlan Plan,
            String OutDir,
            Int32 Ppb
            )
        {
            // The overview compresses full schedule intent into one map for quick sanity checks.
            Double[,] Delta = Grid.Delta();
            using Image<Rgba32> Image = DiffToImage(Delta, Grid.Valid, Ppb);
            FadeImage(Image, 0.45);

            (Double Cx, Double Cy) = WorldToPixel(Grid, Plan.Geometry.CenterX, Plan.Geometry.CenterY, Ppb);
            Double RadiusPx = ((Plan.Geometry.RadiusM / Grid.GridSizeM) * Ppb);
            Single ArmW = (Single)Math.Max(2, Math.Round((Plan.Geometry.WheelbaseM / Grid.GridSizeM) * Ppb));
            Int32 Count = Plan.Positions.Count;
            DrawCircleOutline(Image, Cx, Cy, RadiusPx, new Rgba32(20, 60, 160, 230), Math.Max(2, (Ppb / 3)));
            foreach (PivotPosition Position in Plan.Positions)
            {
                Double Fraction = ((Double)(Position.Index - 1) / Math.Max(1, (Count - 1)));
                Rgba32 Color = new Rgba32((Byte)(40 + (200 * Fraction)), 40, (Byte)(220 - (180 * Fraction)), 235);
                (Double TxW, Double TyW) = Plan.Geometry.TipXy(Position.BearingDeg);
                (Double Tx, Double Ty) = WorldToPixel(Grid, TxW, TyW, Ppb);
                DrawLine(Image, Cx, Cy, Tx, Ty, Color, Math.Max(1, (Int32)Math.Round(ArmW)));
            }

            Double Ar = Math.Max(3.0, ArmW);
            DrawCircleFilled(Image, Cx, Cy, Ar, new Rgba32(10, 10, 10, Constants.MAX_COLOR_CHANNEL));
            DrawCircleOutline(Image, Cx, Cy, Ar, new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL), 1);

            DrawBanner(Image, new List<String>
            {
                String.Format(CultureInfo.InvariantCulture, "All pivot positions: {0} (moves {1}, total rotation {2:F0} deg)", Count, Plan.MoveCount, Plan.TotalRotationDeg),
                "arm color: blue=first -> red=last",
            }, Ppb);
            Image.Save(Path.Combine(OutDir, "pivot_overview.png"));
        }

        /// <summary>
        /// Builds base render image for current field state.
        /// </summary>
        /// <param name="Existing">Existing elevation matrix.</param>
        /// <param name="Target">Target elevation matrix.</param>
        /// <param name="Valid">Valid mask matrix.</param>
        /// <param name="Mode">Render mode.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        /// <param name="Vmin">Minimum topology scale.</param>
        /// <param name="Vmax">Maximum topology scale.</param>
        /// <returns>RGBA image.</returns>
        private static Image<Rgba32> BuildBaseImage
            (
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Valid,
            String Mode,
            Int32 Ppb,
            Double Vmin,
            Double Vmax
            )
        {
            // Mode selection mirrors Python: cut/fill by default, topology when requested.
            if (String.Equals(Mode, "topology", StringComparison.OrdinalIgnoreCase))
            {
                return ElevationToImage(Existing, Valid, Ppb, Vmin, Vmax);
            }

            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            Double[,] Diff = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Diff[By, Bx] = (Existing[By, Bx] - Target[By, Bx]);
                }
            }

            return DiffToImage(Diff, Valid, Ppb);
        }

        /// <summary>
        /// Computes vmin/vmax for topology mode.
        /// </summary>
        /// <param name="Existing">Existing elevations.</param>
        /// <param name="Target">Target elevations.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="Mode">Render mode.</param>
        /// <returns>Minimum and maximum value pair.</returns>
        private static (Double VMin, Double VMax) ComputeModeRange
            (
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Valid,
            String Mode
            )
        {
            // Shared scale across frames is essential in topology mode so flattening is visually meaningful.
            if ((!String.Equals(Mode, "topology", StringComparison.OrdinalIgnoreCase)))
            {
                return (0.0, 0.0);
            }

            Double Min = Double.PositiveInfinity;
            Double Max = Double.NegativeInfinity;
            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (Valid[By, Bx])
                    {
                        Min = Math.Min(Min, Existing[By, Bx]);
                        Min = Math.Min(Min, Target[By, Bx]);
                        Max = Math.Max(Max, Existing[By, Bx]);
                        Max = Math.Max(Max, Target[By, Bx]);
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
        /// <param name="Diff">Delta matrix.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        /// <returns>RGBA image.</returns>
        private static Image<Rgba32> DiffToImage
            (
            Double[,] Diff,
            Boolean[,] Valid,
            Int32 Ppb
            )
        {
            // Y is flipped so larger grid row indices render higher, matching Python images.
            Int32 Height = Diff.GetLength(0);
            Int32 Width = Diff.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);
            for (Int32 By = 0; By < Height; By += 1)
            {
                Int32 Y = (Height - 1 - By);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Valid[By, Bx])
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Image[Bx, Y] = CutFillColor(Diff[By, Bx]);
                }
            }

            if (Ppb > 1)
            {
                Image.Mutate(Context => Context.Resize((Width * Ppb), (Height * Ppb), KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Renders terrain-style image from absolute elevations.
        /// </summary>
        /// <param name="Elev">Elevation matrix.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        /// <param name="Vmin">Scale minimum.</param>
        /// <param name="Vmax">Scale maximum.</param>
        /// <returns>RGBA image.</returns>
        private static Image<Rgba32> ElevationToImage
            (
            Double[,] Elev,
            Boolean[,] Valid,
            Int32 Ppb,
            Double Vmin,
            Double Vmax
            )
        {
            // Linear interpolation across terrain stops keeps frame-to-frame color continuity.
            Double[] Stops = new[] { 0.0, Constants.QUARTER_SCALE, Constants.HALF_SCALE, 0.75, 1.0 };
            Rgba32[] Colors = new[]
            {
                new Rgba32(30, 90, 40, Constants.MAX_COLOR_CHANNEL),
                new Rgba32(120, 170, 70, Constants.MAX_COLOR_CHANNEL),
                new Rgba32(210, 200, 120, Constants.MAX_COLOR_CHANNEL),
                new Rgba32(150, 110, 70, Constants.MAX_COLOR_CHANNEL),
                new Rgba32(245, 245, 245, Constants.MAX_COLOR_CHANNEL),
            };

            Int32 Height = Elev.GetLength(0);
            Int32 Width = Elev.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);
            Double Range = ((Vmax - Vmin) != 0.0 ? (Vmax - Vmin) : 1.0);
            for (Int32 By = 0; By < Height; By += 1)
            {
                Int32 Y = (Height - 1 - By);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Valid[By, Bx])
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Double T = ((Elev[By, Bx] - Vmin) / Range);
                    T = Math.Max(0.0, Math.Min(1.0, T));
                    Image[Bx, Y] = InterpolateStops(T, Stops, Colors);
                }
            }

            if (Ppb > 1)
            {
                Image.Mutate(Context => Context.Resize((Width * Ppb), (Height * Ppb), KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Maps cut/fill delta to nine-band color.
        /// </summary>
        /// <param name="Diff">Existing minus target delta.</param>
        /// <returns>Palette color.</returns>
        private static Rgba32 CutFillColor
            (
            Double Diff
            )
        {
            // Thresholds are copied from Python render palette for visual parity.
            if (Diff < -0.80) return new Rgba32(0x80, 0x00, 0x80, Constants.MAX_COLOR_CHANNEL);
            if (Diff < -Constants.HALF_SCALE) return new Rgba32(0x9A, 0x31, Constants.BYTE_MASK, Constants.MAX_COLOR_CHANNEL);
            if (Diff < -0.05) return new Rgba32(0x00, 0x66, Constants.BYTE_MASK, Constants.MAX_COLOR_CHANNEL);
            if (Diff < -0.01) return new Rgba32(0x36, 0xF5, 0xF5, Constants.MAX_COLOR_CHANNEL);
            if (Diff <= 0.01) return new Rgba32(0x00, 0xCD, 0x00, Constants.MAX_COLOR_CHANNEL);
            if (Diff <= 0.05) return new Rgba32(Constants.BYTE_MASK, Constants.BYTE_MASK, 0x00, Constants.MAX_COLOR_CHANNEL);
            if (Diff <= Constants.HALF_SCALE) return new Rgba32(Constants.BYTE_MASK, 0x80, 0x00, Constants.MAX_COLOR_CHANNEL);
            if (Diff <= 0.80) return new Rgba32(Constants.BYTE_MASK, 0x00, 0x00, Constants.MAX_COLOR_CHANNEL);
            return new Rgba32(0xB4, 0x00, 0x00, Constants.MAX_COLOR_CHANNEL);
        }

        /// <summary>
        /// Performs piecewise linear color interpolation.
        /// </summary>
        /// <param name="T">Normalized value in [0, 1].</param>
        /// <param name="Stops">Monotonic stop positions.</param>
        /// <param name="Colors">Stop colors.</param>
        /// <returns>Interpolated color.</returns>
        private static Rgba32 InterpolateStops
            (
            Double T,
            Double[] Stops,
            Rgba32[] Colors
            )
        {
            // Piecewise interpolation reproduces numpy interp behavior used by Python elevation rendering.
            if (T <= Stops[0])
            {
                return Colors[0];
            }

            if (T >= Stops[Stops.Length - 1])
            {
                return Colors[Colors.Length - 1];
            }

            for (Int32 Iter = 0; Iter < (Stops.Length - 1); Iter += 1)
            {
                Double A = Stops[Iter];
                Double B = Stops[Iter + 1];
                if ((T >= A) && (T <= B))
                {
                    Double U = ((T - A) / (B - A));
                    Byte R = (Byte)Math.Round((Colors[Iter].R + ((Colors[Iter + 1].R - Colors[Iter].R) * U)));
                    Byte G = (Byte)Math.Round((Colors[Iter].G + ((Colors[Iter + 1].G - Colors[Iter].G) * U)));
                    Byte Bl = (Byte)Math.Round((Colors[Iter].B + ((Colors[Iter + 1].B - Colors[Iter].B) * U)));
                    return new Rgba32(R, G, Bl, Constants.MAX_COLOR_CHANNEL);
                }
            }

            return Colors[Colors.Length - 1];
        }

        /// <summary>
        /// Lightens image toward white by blend factor.
        /// </summary>
        /// <param name="Image">Image to modify.</param>
        /// <param name="Keep">Base-color retain factor.</param>
        private static void FadeImage
            (
            Image<Rgba32> Image,
            Double Keep
            )
        {
            // Fading mirrors Python overview rendering so arm lines dominate visual attention.
            Double White = (1.0 - Keep);
            Image.ProcessPixelRows(Accessor =>
            {
                for (Int32 Y = 0; Y < Accessor.Height; Y += 1)
                {
                    Span<Rgba32> Row = Accessor.GetRowSpan(Y);
                    for (Int32 X = 0; X < Row.Length; X += 1)
                    {
                        Rgba32 P = Row[X];
                        Byte R = (Byte)Math.Clamp((P.R * Keep) + (Constants.MAX_COLOR_CHANNEL * White), 0.0, Constants.MAX_COLOR_CHANNEL);
                        Byte G = (Byte)Math.Clamp((P.G * Keep) + (Constants.MAX_COLOR_CHANNEL * White), 0.0, Constants.MAX_COLOR_CHANNEL);
                        Byte B = (Byte)Math.Clamp((P.B * Keep) + (Constants.MAX_COLOR_CHANNEL * White), 0.0, Constants.MAX_COLOR_CHANNEL);
                        Row[X] = new Rgba32(R, G, B, P.A);
                    }
                }
            });
        }

        /// <summary>
        /// Maps world coordinate to pixel coordinate with flipped Y axis.
        /// </summary>
        /// <param name="Grid">Grid metadata.</param>
        /// <param name="X">World easting.</param>
        /// <param name="Y">World northing.</param>
        /// <param name="Ppb">Pixels per bin.</param>
        /// <returns>Pixel coordinate.</returns>
        private static (Double X, Double Y) WorldToPixel
            (
            Grid Grid,
            Double X,
            Double Y,
            Int32 Ppb
            )
        {
            // The coordinate transform intentionally mirrors Python render._world_to_pixel.
            Double Px = (((X - Grid.MinX) / Grid.GridSizeM) * Ppb);
            Double Py = ((Grid.Height - ((Y - Grid.MinY) / Grid.GridSizeM)) * Ppb);
            return (Px, Py);
        }

        /// <summary>
        /// Returns one-based position index containing a given pass.
        /// </summary>
        /// <param name="Plan">Pivot plan.</param>
        /// <param name="Pass1Based">1-based pass number.</param>
        /// <returns>Position index.</returns>
        private static Int32 PositionOfPass
            (
            PivotPlan Plan,
            Int32 Pass1Based
            )
        {
            // A fallback to final position is retained for defensive parity with Python helper behavior.
            foreach (PivotPosition Position in Plan.Positions)
            {
                if ((Position.FirstPass <= Pass1Based) && (Pass1Based <= Position.LastPass))
                {
                    return Position.Index;
                }
            }

            return Plan.Positions.Count;
        }

        /// <summary>
        /// Applies GIF frame delay in milliseconds.
        /// </summary>
        /// <param name="Frame">Frame to modify.</param>
        /// <param name="DelayMs">Delay milliseconds.</param>
        private static void SetGifFrameDelay
            (
            ImageFrame<Rgba32> Frame,
            Int32 DelayMs
            )
        {
            // GIF stores delay in centiseconds, so milliseconds are rounded accordingly.
            Int32 Centiseconds = Math.Max(1, (Int32)Math.Round((DelayMs / 10.0)));
            Frame.Metadata.GetGifMetadata().FrameDelay = Centiseconds;
        }

        /// <summary>
        /// Clones a 2D double matrix.
        /// </summary>
        /// <param name="Matrix">Source matrix.</param>
        /// <returns>Cloned matrix.</returns>
        private static Double[,] Clone
            (
            Double[,] Matrix
            )
        {
            // Explicit clone keeps mutation inside render/animation methods isolated from caller state.
            Int32 Height = Matrix.GetLength(0);
            Int32 Width = Matrix.GetLength(1);
            Double[,] Copy = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Copy[By, Bx] = Matrix[By, Bx];
                }
            }

            return Copy;
        }

        /// <summary>
        /// Converts UTM coordinate to latitude/longitude.
        /// </summary>
        /// <param name="Easting">UTM easting.</param>
        /// <param name="Northing">UTM northing.</param>
        /// <param name="Zone">UTM zone number.</param>
        /// <param name="IsNorthern">UTM hemisphere flag.</param>
        /// <returns>Latitude/longitude in degrees.</returns>
        private static (Double Latitude, Double Longitude) LatLonFromUtm
            (
            Double Easting,
            Double Northing,
            Int32 Zone,
            Boolean IsNorthern
            )
        {
            // This is a direct C# port of the inverse UTM math used by Python AGD support.
            Double B = (UTM_A * (1.0 - UTM_F));
            Double E2 = (1.0 - ((B * B) / (UTM_A * UTM_A)));
            Double EPrime2 = (E2 / (1.0 - E2));

            Double X = (Easting - 500000.0);
            Double Y = (Northing - (IsNorthern ? 0.0 : 10000000.0));
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

            Double LonOrigin = (((Zone - 1) * 6) - 180 + 3);
            Double Latitude = RadiansToDegrees(LatRad);
            Double Longitude = (LonOrigin + RadiansToDegrees(LonOffset));
            return (Latitude, Longitude);
        }

        /// <summary>
        /// Ensures polygon ring has CCW winding.
        /// </summary>
        /// <param name="Ring">Input ring.</param>
        /// <returns>CCW ring copy.</returns>
        private static List<(Double X, Double Y)> EnsureCcw
            (
            IReadOnlyList<(Double X, Double Y)> Ring
            )
        {
            // CCW winding makes "right-side" outward normals consistent.
            List<(Double X, Double Y)> Result = new List<(Double X, Double Y)>(Ring);
            if ((Result.Count >= 3) && (SignedArea(Result) < 0.0))
            {
                Result.Reverse();
            }

            return Result;
        }

        /// <summary>
        /// Computes polygon signed area.
        /// </summary>
        /// <param name="Ring">Polygon ring.</param>
        /// <returns>Signed area.</returns>
        private static Double SignedArea
            (
            IReadOnlyList<(Double X, Double Y)> Ring
            )
        {
            // Positive area indicates CCW winding.
            Double Area = 0.0;
            for (Int32 Iter = 0; Iter < Ring.Count; Iter += 1)
            {
                (Double X1, Double Y1) = Ring[Iter];
                (Double X2, Double Y2) = Ring[(Iter + 1) % Ring.Count];
                Area += ((X1 * Y2) - (X2 * Y1));
            }

            return (Constants.HALF_SCALE * Area);
        }

        /// <summary>
        /// Builds miter-offset outward buffer for convex CCW polygon.
        /// </summary>
        /// <param name="Ring">CCW ring.</param>
        /// <param name="Clearance">Buffer distance.</param>
        /// <returns>Buffered ring.</returns>
        private static List<(Double X, Double Y)> MiterBuffer
            (
            IReadOnlyList<(Double X, Double Y)> Ring,
            Double Clearance
            )
        {
            // Miter scale is clamped for sharp corners exactly as in Python implementation.
            if ((Ring.Count < 3) || (Clearance <= 0.0))
            {
                return new List<(Double X, Double Y)>(Ring);
            }

            List<(Double NX, Double NY)> Normals = new List<(Double NX, Double NY)>();
            for (Int32 Iter = 0; Iter < Ring.Count; Iter += 1)
            {
                (Double X1, Double Y1) = Ring[Iter];
                (Double X2, Double Y2) = Ring[(Iter + 1) % Ring.Count];
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
            for (Int32 Iter = 0; Iter < Ring.Count; Iter += 1)
            {
                (Double NX, Double NY) Prev = Normals[(Iter - 1 + Ring.Count) % Ring.Count];
                (Double NX, Double NY) Cur = Normals[Iter];
                Double Mx = (Prev.NX + Cur.NX);
                Double My = (Prev.NY + Cur.NY);
                Double MLen = Math.Sqrt((Mx * Mx) + (My * My));
                if (MLen < Constants.NUMERIC_COMPARISON_TOLERANCE)
                {
                    Mx = Cur.NX;
                    My = Cur.NY;
                    MLen = 1.0;
                }

                Mx /= MLen;
                My /= MLen;
                Double CosHalf = ((Mx * Cur.NX) + (My * Cur.NY));
                Double Scale = (Clearance / Math.Max(CosHalf, 0.35));
                (Double Px, Double Py) = Ring[Iter];
                Out.Add((Px + (Mx * Scale), Py + (My * Scale)));
            }

            return Out;
        }

        /// <summary>
        /// Tests point-in-ring with ray cast.
        /// </summary>
        /// <param name="X">Point x.</param>
        /// <param name="Y">Point y.</param>
        /// <param name="Ring">Ring vertices.</param>
        /// <returns>True if point is inside.</returns>
        private static Boolean PointInRing
            (
            Double X,
            Double Y,
            IReadOnlyList<(Double X, Double Y)> Ring
            )
        {
            // Boundary is treated as outside, consistent with Python's strict-ish test.
            Boolean Inside = false;
            Int32 J = (Ring.Count - 1);
            for (Int32 Iter = 0; Iter < Ring.Count; Iter += 1)
            {
                (Double Xi, Double Yi) = Ring[Iter];
                (Double Xj, Double Yj) = Ring[J];
                if ((Yi > Y) != (Yj > Y))
                {
                    Double XAt = (((Xj - Xi) * (Y - Yi) / ((Yj - Yi) + 1e-18)) + Xi);
                    if (X < XAt)
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
        /// <param name="A">Segment start.</param>
        /// <param name="B">Segment end.</param>
        /// <param name="Box">Box tuple.</param>
        /// <returns>True if segment may intersect box.</returns>
        private static Boolean SegmentIntersectsAabb
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            (Double MinX, Double MinY, Double MaxX, Double MaxY) Box
            )
        {
            // Slab rejects remove most clear segments before precise geometry work.
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

        /// <summary>
        /// Runs Dijkstra shortest path on adjacency list.
        /// </summary>
        /// <param name="Adj">Adjacency list.</param>
        /// <param name="Src">Source index.</param>
        /// <param name="Dst">Target index.</param>
        /// <returns>Previous-node array and distance array.</returns>
        private static (Int32[] Prev, Double[] Dist) Dijkstra
            (
            IReadOnlyList<List<(Int32 Index, Double Weight)>> Adj,
            Int32 Src,
            Int32 Dst
            )
        {
            // Priority-queue Dijkstra is used because edge weights are non-negative Euclidean lengths.
            Int32 Count = Adj.Count;
            Int32[] Prev = Enumerable.Repeat(-1, Count).ToArray();
            Double[] Dist = Enumerable.Repeat(Double.PositiveInfinity, Count).ToArray();
            Dist[Src] = 0.0;
            PriorityQueue<Int32, Double> Queue = new PriorityQueue<Int32, Double>();
            Queue.Enqueue(Src, 0.0);

            while (Queue.Count > 0)
            {
                Queue.TryDequeue(out Int32 Node, out Double Cost);
                if (Cost > Dist[Node])
                {
                    continue;
                }

                if (Node == Dst)
                {
                    break;
                }

                foreach ((Int32 Index, Double Weight) Edge in Adj[Node])
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
        /// <param name="Adj">Source adjacency list.</param>
        /// <returns>Deep-cloned adjacency list.</returns>
        private static List<List<(Int32 Index, Double Weight)>> CloneAdj
            (
            IReadOnlyList<List<(Int32 Index, Double Weight)>> Adj
            )
        {
            // Cloning allows temporary endpoint nodes without mutating cached graph.
            List<List<(Int32 Index, Double Weight)>> Result = new List<List<(Int32 Index, Double Weight)>>();
            foreach (List<(Int32 Index, Double Weight)> Row in Adj)
            {
                Result.Add(new List<(Int32 Index, Double Weight)>(Row));
            }

            return Result;
        }

        /// <summary>
        /// Computes Euclidean distance between two points.
        /// </summary>
        /// <param name="A">First point.</param>
        /// <param name="B">Second point.</param>
        /// <returns>Distance.</returns>
        private static Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Centralized distance helper keeps geometry formulas consistent.
            Double Dx = (A.X - B.X);
            Double Dy = (A.Y - B.Y);
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Converts degrees to radians.
        /// </summary>
        /// <param name="Degrees">Angle in degrees.</param>
        /// <returns>Angle in radians.</returns>
        private static Double DegreesToRadians
            (
            Double Degrees
            )
        {
            // Trigonometric helper keeps conversion intent explicit.
            return (Degrees * Math.PI / Constants.DEGREES_PER_HALF_TURN);
        }

        /// <summary>
        /// Converts radians to degrees.
        /// </summary>
        /// <param name="Radians">Angle in radians.</param>
        /// <returns>Angle in degrees.</returns>
        private static Double RadiansToDegrees
            (
            Double Radians
            )
        {
            // Trigonometric helper keeps conversion intent explicit.
            return (Radians * Constants.DEGREES_PER_HALF_TURN / Math.PI);
        }

        /// <summary>
        /// Positive modulo for floating-point values.
        /// </summary>
        /// <param name="Value">Value.</param>
        /// <param name="Modulus">Modulus.</param>
        /// <returns>Positive remainder.</returns>
        private static Double PositiveMod
            (
            Double Value,
            Double Modulus
            )
        {
            // Positive modulo avoids negative wrap artifacts with bearings and angles.
            Double Result = (Value % Modulus);
            if (Result < 0.0)
            {
                Result += Modulus;
            }

            return Result;
        }

        /// <summary>
        /// Positive modulo for integer values.
        /// </summary>
        /// <param name="Value">Value.</param>
        /// <param name="Modulus">Modulus.</param>
        /// <returns>Positive remainder.</returns>
        private static Int32 Mod
            (
            Int32 Value,
            Int32 Modulus
            )
        {
            // Positive integer modulo is used for wrapped bin indexing.
            Int32 Result = (Value % Modulus);
            if (Result < 0)
            {
                Result += Modulus;
            }

            return Result;
        }

        /// <summary>
        /// Draws a thick line.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="X0">Start x.</param>
        /// <param name="Y0">Start y.</param>
        /// <param name="X1">End x.</param>
        /// <param name="Y1">End y.</param>
        /// <param name="Color">Line color.</param>
        /// <param name="Thickness">Line thickness.</param>
        private static void DrawLine
            (
            Image<Rgba32> Image,
            Double X0,
            Double Y0,
            Double X1,
            Double Y1,
            Rgba32 Color,
            Int32 Thickness
            )
        {
            // Segment-distance rasterization keeps line width stable for any direction.
            Double MinX = (Math.Min(X0, X1) - Thickness - 1.0);
            Double MaxX = (Math.Max(X0, X1) + Thickness + 1.0);
            Double MinY = (Math.Min(Y0, Y1) - Thickness - 1.0);
            Double MaxY = (Math.Max(Y0, Y1) + Thickness + 1.0);
            Int32 PixelX0 = Math.Max(0, (Int32)Math.Floor(MinX));
            Int32 PixelX1 = Math.Min((Image.Width - 1), (Int32)Math.Ceiling(MaxX));
            Int32 PixelY0 = Math.Max(0, (Int32)Math.Floor(MinY));
            Int32 PixelY1 = Math.Min((Image.Height - 1), (Int32)Math.Ceiling(MaxY));

            Double Radius = (Math.Max(1, Thickness) / 2.0);
            Double RadiusSq = (Radius * Radius);
            Double Dx = (X1 - X0);
            Double Dy = (Y1 - Y0);
            Double LenSq = ((Dx * Dx) + (Dy * Dy));
            if (LenSq <= EPSILON)
            {
                DrawCircleFilled(Image, X0, Y0, Radius, Color);
                return;
            }

            for (Int32 Y = PixelY0; Y <= PixelY1; Y += 1)
            {
                for (Int32 X = PixelX0; X <= PixelX1; X += 1)
                {
                    Double T = ((((X - X0) * Dx) + ((Y - Y0) * Dy)) / LenSq);
                    T = Math.Max(0.0, Math.Min(1.0, T));
                    Double Px = (X0 + (T * Dx));
                    Double Py = (Y0 + (T * Dy));
                    Double DistSq = (((X - Px) * (X - Px)) + ((Y - Py) * (Y - Py)));
                    if (DistSq <= RadiusSq)
                    {
                        BlendPixel(Image, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Draws a filled circle.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="Cx">Center x.</param>
        /// <param name="Cy">Center y.</param>
        /// <param name="Radius">Radius.</param>
        /// <param name="Color">Fill color.</param>
        private static void DrawCircleFilled
            (
            Image<Rgba32> Image,
            Double Cx,
            Double Cy,
            Double Radius,
            Rgba32 Color
            )
        {
            // Filled circles are used for anchor and tower markers.
            Int32 X0 = Math.Max(0, (Int32)Math.Floor(Cx - Radius));
            Int32 X1 = Math.Min((Image.Width - 1), (Int32)Math.Ceiling(Cx + Radius));
            Int32 Y0 = Math.Max(0, (Int32)Math.Floor(Cy - Radius));
            Int32 Y1 = Math.Min((Image.Height - 1), (Int32)Math.Ceiling(Cy + Radius));
            Double R2 = (Radius * Radius);
            for (Int32 Y = Y0; Y <= Y1; Y += 1)
            {
                for (Int32 X = X0; X <= X1; X += 1)
                {
                    Double Dx = (X - Cx);
                    Double Dy = (Y - Cy);
                    if (((Dx * Dx) + (Dy * Dy)) <= R2)
                    {
                        BlendPixel(Image, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Draws a circle outline.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="Cx">Center x.</param>
        /// <param name="Cy">Center y.</param>
        /// <param name="Radius">Radius.</param>
        /// <param name="Color">Outline color.</param>
        /// <param name="Thickness">Outline thickness.</param>
        private static void DrawCircleOutline
            (
            Image<Rgba32> Image,
            Double Cx,
            Double Cy,
            Double Radius,
            Rgba32 Color,
            Int32 Thickness
            )
        {
            // Outline is rasterized as a radial band around the requested radius.
            Double Half = (Math.Max(1, Thickness) / 2.0);
            Double Inner = Math.Max(0.0, (Radius - Half));
            Double Outer = (Radius + Half);
            Double Inner2 = (Inner * Inner);
            Double Outer2 = (Outer * Outer);
            Int32 X0 = Math.Max(0, (Int32)Math.Floor(Cx - Outer));
            Int32 X1 = Math.Min((Image.Width - 1), (Int32)Math.Ceiling(Cx + Outer));
            Int32 Y0 = Math.Max(0, (Int32)Math.Floor(Cy - Outer));
            Int32 Y1 = Math.Min((Image.Height - 1), (Int32)Math.Ceiling(Cy + Outer));
            for (Int32 Y = Y0; Y <= Y1; Y += 1)
            {
                for (Int32 X = X0; X <= X1; X += 1)
                {
                    Double Dx = (X - Cx);
                    Double Dy = (Y - Cy);
                    Double Dist2 = ((Dx * Dx) + (Dy * Dy));
                    if ((Dist2 >= Inner2) && (Dist2 <= Outer2))
                    {
                        BlendPixel(Image, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Draws a filled rectangle.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="X">Left pixel.</param>
        /// <param name="Y">Top pixel.</param>
        /// <param name="Width">Width.</param>
        /// <param name="Height">Height.</param>
        /// <param name="Color">Fill color.</param>
        private static void DrawRectangleFilled
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Int32 Width,
            Int32 Height,
            Rgba32 Color
            )
        {
            // Rectangle fill supports translucent banner backgrounds.
            Int32 X0 = Math.Max(0, X);
            Int32 Y0 = Math.Max(0, Y);
            Int32 X1 = Math.Min((Image.Width - 1), (X + Width));
            Int32 Y1 = Math.Min((Image.Height - 1), (Y + Height));
            for (Int32 PixelY = Y0; PixelY <= Y1; PixelY += 1)
            {
                for (Int32 PixelX = X0; PixelX <= X1; PixelX += 1)
                {
                    BlendPixel(Image, PixelX, PixelY, Color);
                }
            }
        }

        /// <summary>
        /// Draws a rectangle outline.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="X">Left pixel.</param>
        /// <param name="Y">Top pixel.</param>
        /// <param name="Width">Width.</param>
        /// <param name="Height">Height.</param>
        /// <param name="Color">Outline color.</param>
        /// <param name="Thickness">Outline thickness.</param>
        private static void DrawRectangleOutline
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Int32 Width,
            Int32 Height,
            Rgba32 Color,
            Int32 Thickness
            )
        {
            // Four edge lines produce the outline ring.
            DrawLine(Image, X, Y, (X + Width), Y, Color, Thickness);
            DrawLine(Image, (X + Width), Y, (X + Width), (Y + Height), Color, Thickness);
            DrawLine(Image, (X + Width), (Y + Height), X, (Y + Height), Color, Thickness);
            DrawLine(Image, X, (Y + Height), X, Y, Color, Thickness);
        }

        /// <summary>
        /// Blends one source pixel onto destination.
        /// </summary>
        /// <param name="Image">Target image.</param>
        /// <param name="X">Pixel x.</param>
        /// <param name="Y">Pixel y.</param>
        /// <param name="Src">Source color.</param>
        private static void BlendPixel
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Rgba32 Src
            )
        {
            // Source-over alpha blend mirrors the compositing behavior used by PIL overlays.
            if ((X < 0) || (Y < 0) || (X >= Image.Width) || (Y >= Image.Height))
            {
                return;
            }

            Rgba32 Dst = Image[X, Y];
            Double Sa = (Src.A / Constants.MAX_COLOR_CHANNEL);
            Double Da = (Dst.A / Constants.MAX_COLOR_CHANNEL);
            Double OutA = (Sa + (Da * (1.0 - Sa)));
            if (OutA <= EPSILON)
            {
                Image[X, Y] = new Rgba32(0, 0, 0, 0);
                return;
            }

            Byte R = (Byte)Math.Clamp(Math.Round((((Src.R * Sa) + (Dst.R * Da * (1.0 - Sa))) / OutA)), 0.0, Constants.MAX_COLOR_CHANNEL);
            Byte G = (Byte)Math.Clamp(Math.Round((((Src.G * Sa) + (Dst.G * Da * (1.0 - Sa))) / OutA)), 0.0, Constants.MAX_COLOR_CHANNEL);
            Byte B = (Byte)Math.Clamp(Math.Round((((Src.B * Sa) + (Dst.B * Da * (1.0 - Sa))) / OutA)), 0.0, Constants.MAX_COLOR_CHANNEL);
            Byte A = (Byte)Math.Clamp(Math.Round((OutA * Constants.MAX_COLOR_CHANNEL)), 0.0, Constants.MAX_COLOR_CHANNEL);
            Image[X, Y] = new Rgba32(R, G, B, A);
        }
    }
}
