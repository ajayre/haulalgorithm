using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// One high-level cut-zone work step.
    /// </summary>
    public sealed class ZoneStep
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="ZoneStep"/> class.
        /// </summary>
        /// <param name="WorkOrder">Work order rank.</param>
        /// <param name="ZoneId">Cluster ID.</param>
        /// <param name="NLoads">Number of loads.</param>
        /// <param name="TotalBcy">Moved BCY.</param>
        /// <param name="Easting">Centroid easting.</param>
        /// <param name="Northing">Centroid northing.</param>
        /// <param name="Lat">Centroid latitude.</param>
        /// <param name="Lon">Centroid longitude.</param>
        /// <param name="FirstPass">First pass index.</param>
        /// <param name="LastPass">Last pass index.</param>
        /// <param name="MeanPass">Weighted mean pass index.</param>
        /// <param name="Compass">Compass label.</param>
        /// <param name="Outline">Cell-boundary outline segments.</param>
        public ZoneStep
            (
            Int32 WorkOrder,
            Int32 ZoneId,
            Int32 NLoads,
            Double TotalBcy,
            Double Easting,
            Double Northing,
            Double Lat,
            Double Lon,
            Int32 FirstPass,
            Int32 LastPass,
            Double MeanPass,
            String Compass,
            List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)> Outline
            )
        {
            // Store one zone summary row.
            this.WorkOrder = WorkOrder;
            this.ZoneId = ZoneId;
            this.NLoads = NLoads;
            this.TotalBcy = TotalBcy;
            this.Easting = Easting;
            this.Northing = Northing;
            this.Lat = Lat;
            this.Lon = Lon;
            this.FirstPass = FirstPass;
            this.LastPass = LastPass;
            this.MeanPass = MeanPass;
            this.Compass = Compass;
            this.Outline = Outline;
        }

        /// <summary>
        /// Gets work order.
        /// </summary>
        public Int32 WorkOrder { get; }

        /// <summary>
        /// Gets zone id.
        /// </summary>
        public Int32 ZoneId { get; }

        /// <summary>
        /// Gets load count.
        /// </summary>
        public Int32 NLoads { get; }

        /// <summary>
        /// Gets total BCY.
        /// </summary>
        public Double TotalBcy { get; }

        /// <summary>
        /// Gets centroid easting.
        /// </summary>
        public Double Easting { get; }

        /// <summary>
        /// Gets centroid northing.
        /// </summary>
        public Double Northing { get; }

        /// <summary>
        /// Gets centroid latitude.
        /// </summary>
        public Double Lat { get; }

        /// <summary>
        /// Gets centroid longitude.
        /// </summary>
        public Double Lon { get; }

        /// <summary>
        /// Gets first pass number.
        /// </summary>
        public Int32 FirstPass { get; }

        /// <summary>
        /// Gets last pass number.
        /// </summary>
        public Int32 LastPass { get; }

        /// <summary>
        /// Gets weighted mean pass.
        /// </summary>
        public Double MeanPass { get; }

        /// <summary>
        /// Gets compass descriptor.
        /// </summary>
        public String Compass { get; }

        /// <summary>
        /// Gets exact zone outline segments.
        /// </summary>
        public List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)> Outline { get; }
    }

    /// <summary>
    /// One pass-pair row that replaces CSV output.
    /// </summary>
    public sealed class PassPairData
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="PassPairData"/> class.
        /// </summary>
        /// <param name="Order">1-based route order.</param>
        /// <param name="LoadBcy">Load BCY.</param>
        /// <param name="HaulM">Haul distance metres.</param>
        /// <param name="CutStartLat">Cut start latitude.</param>
        /// <param name="CutStartLon">Cut start longitude.</param>
        /// <param name="CutEndLat">Cut end latitude.</param>
        /// <param name="CutEndLon">Cut end longitude.</param>
        /// <param name="FillStartLat">Fill start latitude.</param>
        /// <param name="FillStartLon">Fill start longitude.</param>
        /// <param name="FillEndLat">Fill end latitude.</param>
        /// <param name="FillEndLon">Fill end longitude.</param>
        public PassPairData
            (
            Int32 Order,
            Double LoadBcy,
            Double HaulM,
            Double CutStartLat,
            Double CutStartLon,
            Double CutEndLat,
            Double CutEndLon,
            Double FillStartLat,
            Double FillStartLon,
            Double FillEndLat,
            Double FillEndLon
            )
        {
            // Keep exact per-pass geometry converted to lat/lon.
            this.Order = Order;
            this.LoadBcy = LoadBcy;
            this.HaulM = HaulM;
            this.CutStartLat = CutStartLat;
            this.CutStartLon = CutStartLon;
            this.CutEndLat = CutEndLat;
            this.CutEndLon = CutEndLon;
            this.FillStartLat = FillStartLat;
            this.FillStartLon = FillStartLon;
            this.FillEndLat = FillEndLat;
            this.FillEndLon = FillEndLon;
        }

        /// <summary>
        /// Gets route order.
        /// </summary>
        public Int32 Order { get; }

        /// <summary>
        /// Gets load BCY.
        /// </summary>
        public Double LoadBcy { get; }

        /// <summary>
        /// Gets haul metres.
        /// </summary>
        public Double HaulM { get; }

        /// <summary>
        /// Gets cut-start latitude.
        /// </summary>
        public Double CutStartLat { get; }

        /// <summary>
        /// Gets cut-start longitude.
        /// </summary>
        public Double CutStartLon { get; }

        /// <summary>
        /// Gets cut-end latitude.
        /// </summary>
        public Double CutEndLat { get; }

        /// <summary>
        /// Gets cut-end longitude.
        /// </summary>
        public Double CutEndLon { get; }

        /// <summary>
        /// Gets fill-start latitude.
        /// </summary>
        public Double FillStartLat { get; }

        /// <summary>
        /// Gets fill-start longitude.
        /// </summary>
        public Double FillStartLon { get; }

        /// <summary>
        /// Gets fill-end latitude.
        /// </summary>
        public Double FillEndLat { get; }

        /// <summary>
        /// Gets fill-end longitude.
        /// </summary>
        public Double FillEndLon { get; }
    }

    /// <summary>
    /// Zone-sequencing helpers.
    /// </summary>
    public static class Sequence
    {
        /// <summary>
        /// Clusters routed passes into coarse cut zones.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Field">AGD field metadata.</param>
        /// <param name="Route">Routed passes.</param>
        /// <param name="NZones">Desired zone count.</param>
        /// <param name="Seed">Deterministic random seed.</param>
        /// <returns>Ordered zone steps.</returns>
        static public List<ZoneStep> SequenceCutZones
            (
            Grid Grid,
            AgdField Field,
            Route Route,
            Int32 NZones = 8,
            Int32 Seed = 0
            )
        {
            // Summarize fine-grained pass order into human-digestible work zones.
            List<Pass> Passes = Route.OrderedPasses;
            if (Passes.Count == 0)
            {
                return new List<ZoneStep>();
            }

            List<(Double X, Double Y)> Centroids = new List<(Double X, Double Y)>();
            List<Double> Loads = new List<Double>();
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> PassCutCells = new List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>();
            foreach (Pass P in Passes)
            {
                Double Cx = 0.0;
                Double Cy = 0.0;
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in P.CutCells)
                {
                    (Double X, Double Y) Point = Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By);
                    Cx += Point.X;
                    Cy += Point.Y;
                }

                Cx /= P.CutCells.Count;
                Cy /= P.CutCells.Count;
                Centroids.Add((Cx, Cy));
                Loads.Add(P.LoadBcy);
                PassCutCells.Add(new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(P.CutCells));
            }

            Int32 K = Math.Max(1, Math.Min(NZones, Centroids.Count));
            (Int32[] Labels, (Double X, Double Y)[] Centers) = KMeans(Centroids, K, Seed, 100);
            Double FieldCx = 0.0;
            Double FieldCy = 0.0;
            foreach ((Double X, Double Y) Point in Centroids)
            {
                FieldCx += Point.X;
                FieldCy += Point.Y;
            }

            FieldCx /= Centroids.Count;
            FieldCy /= Centroids.Count;

            Dictionary<(Int32 Bx, Int32 By), Double[]> CellZoneDepth = new Dictionary<(Int32 Bx, Int32 By), Double[]>();
            for (Int32 PassIndex = 0; PassIndex < PassCutCells.Count; PassIndex += 1)
            {
                Int32 Zone = Labels[PassIndex];
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in PassCutCells[PassIndex])
                {
                    if (!CellZoneDepth.TryGetValue(Cell.Cell, out Double[]? Acc))
                    {
                        Acc = new Double[K];
                        CellZoneDepth[Cell.Cell] = Acc;
                    }

                    Acc[Zone] += Cell.DepthM;
                }
            }

            List<HashSet<(Int32 Bx, Int32 By)>> ZoneCells = new List<HashSet<(Int32 Bx, Int32 By)>>();
            for (Int32 Zone = 0; Zone < K; Zone += 1)
            {
                ZoneCells.Add(new HashSet<(Int32 Bx, Int32 By)>());
            }

            foreach (KeyValuePair<(Int32 Bx, Int32 By), Double[]> Entry in CellZoneDepth)
            {
                Int32 Zone = ArgMax(Entry.Value);
                ZoneCells[Zone].Add(Entry.Key);
            }

            Double Half = Grid.GridSizeM / 2.0;
            List<_RawZone> Raw = new List<_RawZone>();
            for (Int32 Zone = 0; Zone < K; Zone += 1)
            {
                List<Int32> Indices = new List<Int32>();
                for (Int32 Index = 0; Index < Labels.Length; Index += 1)
                {
                    if (Labels[Index] == Zone)
                    {
                        Indices.Add(Index);
                    }
                }

                if (Indices.Count == 0)
                {
                    continue;
                }

                Double WSum = 0.0;
                Double Cx = 0.0;
                Double Cy = 0.0;
                Double MeanPass = 0.0;
                Int32 FirstPass = Int32.MaxValue;
                Int32 LastPass = Int32.MinValue;
                foreach (Int32 Index in Indices)
                {
                    Double W = Loads[Index];
                    WSum += W;
                    Cx += Centroids[Index].X * W;
                    Cy += Centroids[Index].Y * W;
                    MeanPass += Index * W;
                    FirstPass = Math.Min(FirstPass, Index + 1);
                    LastPass = Math.Max(LastPass, Index + 1);
                }

                if (WSum <= 0.0)
                {
                    WSum = 1.0;
                }

                Cx /= WSum;
                Cy /= WSum;
                MeanPass /= WSum;
                (Double Lat, Double Lon) = Agd.LatlonFromUtm(Cx, Cy, Field.Zone, Field.IsNorthern);
                List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)> Outline = ZoneBoundaryEdges(ZoneCells[Zone], Grid, Half);
                Double TotalBcy = 0.0;
                foreach (Int32 Index in Indices)
                {
                    TotalBcy += Loads[Index];
                }

                Raw.Add(new _RawZone(Zone, Indices.Count, TotalBcy, Cx, Cy, Lat, Lon, FirstPass, LastPass, MeanPass, Compass(Cx, Cy, FieldCx, FieldCy), Outline));
            }

            Raw.Sort((A, B) => A.MeanPass.CompareTo(B.MeanPass));
            List<ZoneStep> Steps = new List<ZoneStep>();
            for (Int32 Index = 0; Index < Raw.Count; Index += 1)
            {
                _RawZone Zone = Raw[Index];
                Steps.Add(new ZoneStep(Index + 1, Zone.ZoneId, Zone.NLoads, Zone.TotalBcy, Zone.Easting, Zone.Northing, Zone.Lat, Zone.Lon, Zone.FirstPass, Zone.LastPass, Zone.MeanPass, Zone.Compass, Zone.Outline));
            }

            return Steps;
        }

        /// <summary>
        /// Builds pass-pair data rows (replaces CSV writer).
        /// </summary>
        /// <param name="Field">Field metadata.</param>
        /// <param name="Route">Routed pass list.</param>
        /// <returns>Pass-pair data rows.</returns>
        static public List<PassPairData> BuildPassPairsData
            (
            AgdField Field,
            Route Route
            )
        {
            // Convert each pass endpoints to lat/lon for downstream export.
            List<PassPairData> Rows = new List<PassPairData>();
            for (Int32 Index = 0; Index < Route.OrderedPasses.Count; Index += 1)
            {
                Pass P = Route.OrderedPasses[Index];
                (Double Cslat, Double Cslon) = Agd.LatlonFromUtm(P.CutStartXy.X, P.CutStartXy.Y, Field.Zone, Field.IsNorthern);
                (Double Celat, Double Celon) = Agd.LatlonFromUtm(P.CutEndXy.X, P.CutEndXy.Y, Field.Zone, Field.IsNorthern);
                (Double Fslat, Double Fslon) = Agd.LatlonFromUtm(P.FillStartXy.X, P.FillStartXy.Y, Field.Zone, Field.IsNorthern);
                (Double Felat, Double Felon) = Agd.LatlonFromUtm(P.FillEndXy.X, P.FillEndXy.Y, Field.Zone, Field.IsNorthern);
                Rows.Add(new PassPairData(Index + 1, P.LoadBcy, P.HaulDistanceM, Cslat, Cslon, Celat, Celon, Fslat, Fslon, Felat, Felon));
            }

            return Rows;
        }

        static private List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)> ZoneBoundaryEdges
            (
            HashSet<(Int32 Bx, Int32 By)> Cells,
            Grid Grid,
            Double Half
            )
        {
            // Emit exact cell-edge perimeter where adjacent zone cell is missing.
            List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)> Segs = new List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)>();
            foreach ((Int32 Bx, Int32 By) Cell in Cells)
            {
                (Double Cx, Double Cy) = Grid.CellCenterXy(Cell.Bx, Cell.By);
                Double X0 = Cx - Half;
                Double X1 = Cx + Half;
                Double Y0 = Cy - Half;
                Double Y1 = Cy + Half;
                if (!Cells.Contains((Cell.Bx + 1, Cell.By))) Segs.Add(((X1, Y0), (X1, Y1)));
                if (!Cells.Contains((Cell.Bx - 1, Cell.By))) Segs.Add(((X0, Y0), (X0, Y1)));
                if (!Cells.Contains((Cell.Bx, Cell.By + 1))) Segs.Add(((X0, Y1), (X1, Y1)));
                if (!Cells.Contains((Cell.Bx, Cell.By - 1))) Segs.Add(((X0, Y0), (X1, Y0)));
            }

            return Segs;
        }

        static private (Int32[] Labels, (Double X, Double Y)[] Centers) KMeans
            (
            List<(Double X, Double Y)> Points,
            Int32 K,
            Int32 Seed,
            Int32 Iters
            )
        {
            // Deterministic k-means++ initialization plus Lloyd iterations.
            Random Rng = new Random(Seed);
            Int32 N = Points.Count;
            List<(Double X, Double Y)> Centers = new List<(Double X, Double Y)>();
            Int32 First = Rng.Next(N);
            Centers.Add(Points[First]);
            for (Int32 C = 1; C < K; C += 1)
            {
                Double[] D2 = new Double[N];
                Double Total = 0.0;
                for (Int32 Index = 0; Index < N; Index += 1)
                {
                    Double Best = Double.PositiveInfinity;
                    foreach ((Double X, Double Y) Center in Centers)
                    {
                        Double Dx = Points[Index].X - Center.X;
                        Double Dy = Points[Index].Y - Center.Y;
                        Best = Math.Min(Best, (Dx * Dx) + (Dy * Dy));
                    }

                    D2[Index] = Best;
                    Total += Best;
                }

                if (Total <= 0.0)
                {
                    Centers.Add(Points[Rng.Next(N)]);
                    continue;
                }

                Double Target = Rng.NextDouble() * Total;
                Double Prefix = 0.0;
                Int32 Pick = 0;
                for (Int32 Index = 0; Index < N; Index += 1)
                {
                    Prefix += D2[Index];
                    if (Prefix >= Target)
                    {
                        Pick = Index;
                        break;
                    }
                }

                Centers.Add(Points[Pick]);
            }

            Int32[] Labels = new Int32[N];
            for (Int32 Iter = 0; Iter < Iters; Iter += 1)
            {
                // Alternate assignment and recomputation until stable.
                Int32[] NewLabels = new Int32[N];
                for (Int32 Index = 0; Index < N; Index += 1)
                {
                    Double Best = Double.PositiveInfinity;
                    Int32 BestJ = 0;
                    for (Int32 J = 0; J < K; J += 1)
                    {
                        Double Dx = Points[Index].X - Centers[J].X;
                        Double Dy = Points[Index].Y - Centers[J].Y;
                        Double Dist2 = (Dx * Dx) + (Dy * Dy);
                        if (Dist2 < Best)
                        {
                            Best = Dist2;
                            BestJ = J;
                        }
                    }

                    NewLabels[Index] = BestJ;
                }

                (Double X, Double Y)[] NewCenters = new (Double X, Double Y)[K];
                Int32[] Counts = new Int32[K];
                for (Int32 Index = 0; Index < N; Index += 1)
                {
                    Int32 Label = NewLabels[Index];
                    NewCenters[Label].X += Points[Index].X;
                    NewCenters[Label].Y += Points[Index].Y;
                    Counts[Label] += 1;
                }

                for (Int32 J = 0; J < K; J += 1)
                {
                    if (Counts[J] > 0)
                    {
                        NewCenters[J].X /= Counts[J];
                        NewCenters[J].Y /= Counts[J];
                    }
                    else
                    {
                        NewCenters[J] = Centers[J];
                    }
                }

                Boolean LabelsEqual = true;
                for (Int32 Index = 0; Index < N; Index += 1)
                {
                    if (NewLabels[Index] != Labels[Index])
                    {
                        LabelsEqual = false;
                    }
                }

                Boolean CentersClose = true;
                for (Int32 J = 0; J < K; J += 1)
                {
                    Double Dx = NewCenters[J].X - Centers[J].X;
                    Double Dy = NewCenters[J].Y - Centers[J].Y;
                    if (((Dx * Dx) + (Dy * Dy)) > 1e-12)
                    {
                        CentersClose = false;
                    }
                }

                Labels = NewLabels;
                Centers = new List<(Double X, Double Y)>(NewCenters);
                if (LabelsEqual && CentersClose)
                {
                    break;
                }
            }

            return (Labels, Centers.ToArray());
        }

        static private String Compass
            (
            Double X,
            Double Y,
            Double Cx,
            Double Cy
            )
        {
            // Use axis-near single-letter or quadrant descriptor.
            String Ns = (Y >= Cy) ? "N" : "S";
            String Ew = (X >= Cx) ? "E" : "W";
            Double SpanX = Math.Abs(X - Cx);
            Double SpanY = Math.Abs(Y - Cy);
            if (SpanX < (0.3 * (SpanX + SpanY + 1e-9)))
            {
                return Ns;
            }

            if (SpanY < (0.3 * (SpanX + SpanY + 1e-9)))
            {
                return Ew;
            }

            return Ns + Ew;
        }

        static private Int32 ArgMax
            (
            Double[] Values
            )
        {
            // Return index of maximum value.
            Int32 Best = 0;
            Double BestValue = Values[0];
            for (Int32 Index = 1; Index < Values.Length; Index += 1)
            {
                if (Values[Index] > BestValue)
                {
                    BestValue = Values[Index];
                    Best = Index;
                }
            }

            return Best;
        }

        /// <summary>
        /// Internal raw zone accumulator.
        /// </summary>
        private sealed class _RawZone
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="_RawZone"/> class.
            /// </summary>
            /// <param name="ZoneId">Zone id.</param>
            /// <param name="NLoads">Load count.</param>
            /// <param name="TotalBcy">Total BCY.</param>
            /// <param name="Easting">Easting.</param>
            /// <param name="Northing">Northing.</param>
            /// <param name="Lat">Latitude.</param>
            /// <param name="Lon">Longitude.</param>
            /// <param name="FirstPass">First pass.</param>
            /// <param name="LastPass">Last pass.</param>
            /// <param name="MeanPass">Mean pass.</param>
            /// <param name="Compass">Compass label.</param>
            /// <param name="Outline">Outline segments.</param>
            public _RawZone
                (
                Int32 ZoneId,
                Int32 NLoads,
                Double TotalBcy,
                Double Easting,
                Double Northing,
                Double Lat,
                Double Lon,
                Int32 FirstPass,
                Int32 LastPass,
                Double MeanPass,
                String Compass,
                List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)> Outline
                )
            {
                // Hold aggregate zone statistics before final ordering.
                this.ZoneId = ZoneId;
                this.NLoads = NLoads;
                this.TotalBcy = TotalBcy;
                this.Easting = Easting;
                this.Northing = Northing;
                this.Lat = Lat;
                this.Lon = Lon;
                this.FirstPass = FirstPass;
                this.LastPass = LastPass;
                this.MeanPass = MeanPass;
                this.Compass = Compass;
                this.Outline = Outline;
            }

            /// <summary>
            /// Gets zone id.
            /// </summary>
            public Int32 ZoneId { get; }

            /// <summary>
            /// Gets load count.
            /// </summary>
            public Int32 NLoads { get; }

            /// <summary>
            /// Gets total BCY.
            /// </summary>
            public Double TotalBcy { get; }

            /// <summary>
            /// Gets easting.
            /// </summary>
            public Double Easting { get; }

            /// <summary>
            /// Gets northing.
            /// </summary>
            public Double Northing { get; }

            /// <summary>
            /// Gets latitude.
            /// </summary>
            public Double Lat { get; }

            /// <summary>
            /// Gets longitude.
            /// </summary>
            public Double Lon { get; }

            /// <summary>
            /// Gets first pass.
            /// </summary>
            public Int32 FirstPass { get; }

            /// <summary>
            /// Gets last pass.
            /// </summary>
            public Int32 LastPass { get; }

            /// <summary>
            /// Gets mean pass.
            /// </summary>
            public Double MeanPass { get; }

            /// <summary>
            /// Gets compass.
            /// </summary>
            public String Compass { get; }

            /// <summary>
            /// Gets outline.
            /// </summary>
            public List<((Double X1, Double Y1) A, (Double X2, Double Y2) B)> Outline { get; }
        }
    }
}
