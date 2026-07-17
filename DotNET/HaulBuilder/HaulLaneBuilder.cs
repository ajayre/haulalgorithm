using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder
{
    /// <summary>
    /// Builds deterministic blade-width work lanes before cut/fill ownership is paired.
    /// </summary>
    static public class HaulLaneBuilder
    {
        /// <summary>Supported first implementation lane axes.</summary>
        public enum LaneAxis
        {
            /// <summary>Rows run west to east.</summary>
            EastWest,

            /// <summary>Columns run south to north.</summary>
            NorthSouth,

            /// <summary>Each cell uses the cardinal axis closest to local flow.</summary>
            Adaptive,

            /// <summary>All lanes use one caller-supplied arbitrary axis.</summary>
            UserHeading,
        }

        /// <summary>Kind of physical work represented by a lane.</summary>
        public enum WorkKind
        {
            /// <summary>Cut material source.</summary>
            Cut,

            /// <summary>Fill material destination.</summary>
            Fill,
        }

        /// <summary>One depth-layer entry in a lane.</summary>
        public sealed class LaneEntry
        {
            /// <summary>Initializes one immutable lane entry.</summary>
            public LaneEntry
                (
                (Int32 Bx, Int32 By) Cell,
                Int32 LayerIndex,
                Double DepthM,
                Double VolumeBcy
                )
            {
                this.Cell = Cell;
                this.LayerIndex = LayerIndex;
                this.DepthM = DepthM;
                this.VolumeBcy = VolumeBcy;
            }

            /// <summary>Gets physical cell coordinate.</summary>
            public (Int32 Bx, Int32 By) Cell { get; }

            /// <summary>Gets zero-based physical depth layer.</summary>
            public Int32 LayerIndex { get; }

            /// <summary>Gets layer depth at the cell.</summary>
            public Double DepthM { get; }

            /// <summary>Gets bank volume represented by the entry.</summary>
            public Double VolumeBcy { get; }
        }

        /// <summary>One contiguous, same-layer, same-kind work lane.</summary>
        public sealed class Lane
        {
            /// <summary>Initializes one lane.</summary>
            public Lane
                (
                Int32 Id,
                WorkKind Kind,
                LaneAxis Axis,
                Int32 LayerIndex,
                IReadOnlyList<LaneEntry> Entries,
                Grid Grid,
                Double? AxisHeadingDeg = null
                )
            {
                this.Id = Id;
                this.Kind = Kind;
                this.Axis = Axis;
                this.LayerIndex = LayerIndex;
                this.Entries = Entries;
                this.AxisHeadingDeg = NormalizeAxisHeadingDeg(
                    AxisHeadingDeg
                    ?? (Axis == LaneAxis.EastWest ? 90.0 : 0.0));
                VolumeBcy = Entries.Sum(Entry => Entry.VolumeBcy);
                (Double X, Double Y) First = Grid.CellCenterXy(
                    Entries[0].Cell.Bx, Entries[0].Cell.By);
                (Double X, Double Y) Last = Grid.CellCenterXy(
                    Entries[Entries.Count - 1].Cell.Bx,
                    Entries[Entries.Count - 1].Cell.By);
                StartXy = First;
                EndXy = Last;
                CentroidXy = (
                    Entries.Average(Entry => Grid.CenterX[Entry.Cell.By, Entry.Cell.Bx]),
                    Entries.Average(Entry => Grid.CenterY[Entry.Cell.By, Entry.Cell.Bx]));
            }

            /// <summary>Gets deterministic lane id.</summary>
            public Int32 Id { get; }

            /// <summary>Gets work kind.</summary>
            public WorkKind Kind { get; }

            /// <summary>Gets lane axis.</summary>
            public LaneAxis Axis { get; }

            /// <summary>Gets depth-layer index.</summary>
            public Int32 LayerIndex { get; }

            /// <summary>Gets ordered work entries.</summary>
            public IReadOnlyList<LaneEntry> Entries { get; }

            /// <summary>
            /// Gets the undirected lane axis in compass degrees. The guidance
            /// planner may drive either this bearing or the opposite bearing.
            /// </summary>
            public Double AxisHeadingDeg { get; }

            /// <summary>Gets total lane volume.</summary>
            public Double VolumeBcy { get; }

            /// <summary>Gets first lane cell centre.</summary>
            public (Double X, Double Y) StartXy { get; }

            /// <summary>Gets last lane cell centre.</summary>
            public (Double X, Double Y) EndXy { get; }

            /// <summary>Gets volume-unweighted lane centroid.</summary>
            public (Double X, Double Y) CentroidXy { get; }
        }

        /// <summary>Complete selected cut and fill lane inventory.</summary>
        public sealed class Inventory
        {
            /// <summary>Initializes a lane inventory.</summary>
            public Inventory
                (
                LaneAxis Axis,
                IReadOnlyList<Lane> CutLanes,
                IReadOnlyList<Lane> FillLanes
                )
            {
                this.Axis = Axis;
                this.CutLanes = CutLanes;
                this.FillLanes = FillLanes;
            }

            /// <summary>Gets selected axis.</summary>
            public LaneAxis Axis { get; }

            /// <summary>Gets cut lanes.</summary>
            public IReadOnlyList<Lane> CutLanes { get; }

            /// <summary>Gets fill lanes.</summary>
            public IReadOnlyList<Lane> FillLanes { get; }
        }

        /// <summary>Builds complete same-layer cut and fill lanes on one axis.</summary>
        static public Inventory Build
            (
            HaulWorkLedger Ledger,
            LaneAxis Axis,
            Boolean ExcludeEdgeWorkCells = false,
            FlowField? Flow = null,
            Double? HeadingAxisDeg = null
            )
        {
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            FieldEdgePolicy EdgePolicy = new FieldEdgePolicy(
                Ledger.Grid, Ledger.Grid.GridSizeM);
            if (Axis == LaneAxis.UserHeading)
            {
                if (!HeadingAxisDeg.HasValue
                    || !Double.IsFinite(HeadingAxisDeg.Value))
                {
                    throw new ArgumentOutOfRangeException(
                        nameof(HeadingAxisDeg));
                }
                Double NormalizedHeadingDeg = NormalizeAxisHeadingDeg(
                    HeadingAxisDeg.Value);
                LaneAxis? CardinalAxis = Math.Abs(NormalizedHeadingDeg) <= Constants.NUMERIC_COMPARISON_TOLERANCE
                    ? LaneAxis.NorthSouth
                    : Math.Abs(NormalizedHeadingDeg - 90.0) <= Constants.NUMERIC_COMPARISON_TOLERANCE
                        ? LaneAxis.EastWest
                        : null;
                List<Lane> HeadingCut = CardinalAxis.HasValue
                    ? BuildKind(
                        Ledger,
                        WorkKind.Cut,
                        CardinalAxis.Value,
                        Ledger.RequiredCutDepthM,
                        EdgePolicy,
                        ExcludeEdgeWorkCells,
                        Ledger.CutBcyPerDepthM,
                        0,
                        null)
                    : BuildHeadingKind(
                        Ledger,
                        WorkKind.Cut,
                        Ledger.RequiredCutDepthM,
                        EdgePolicy,
                        ExcludeEdgeWorkCells,
                        Ledger.CutBcyPerDepthM,
                        0,
                        NormalizedHeadingDeg);
                List<Lane> HeadingFill = CardinalAxis.HasValue
                    ? BuildKind(
                        Ledger,
                        WorkKind.Fill,
                        CardinalAxis.Value,
                        Ledger.RequiredFillDepthM,
                        EdgePolicy,
                        ExcludeEdgeWorkCells,
                        Ledger.FillBcyPerDepthM,
                        HeadingCut.Count,
                        null)
                    : BuildHeadingKind(
                        Ledger,
                        WorkKind.Fill,
                        Ledger.RequiredFillDepthM,
                        EdgePolicy,
                        ExcludeEdgeWorkCells,
                        Ledger.FillBcyPerDepthM,
                        HeadingCut.Count,
                        NormalizedHeadingDeg);
                return new Inventory(Axis, HeadingCut, HeadingFill);
            }
            if (Axis == LaneAxis.Adaptive)
            {
                if (Flow == null)
                {
                    throw new ArgumentNullException(nameof(Flow));
                }
                List<Lane> AdaptiveCut = BuildAdaptiveKind(
                    Ledger,
                    WorkKind.Cut,
                    Ledger.RequiredCutDepthM,
                    EdgePolicy,
                    ExcludeEdgeWorkCells,
                    Ledger.CutBcyPerDepthM,
                    0,
                    Flow);
                List<Lane> AdaptiveFill = BuildAdaptiveKind(
                    Ledger,
                    WorkKind.Fill,
                    Ledger.RequiredFillDepthM,
                    EdgePolicy,
                    ExcludeEdgeWorkCells,
                    Ledger.FillBcyPerDepthM,
                    AdaptiveCut.Count,
                    Flow);
                return new Inventory(
                    Axis, AdaptiveCut, AdaptiveFill);
            }
            List<Lane> Cut = BuildKind(
                Ledger, WorkKind.Cut, Axis,
                Ledger.RequiredCutDepthM, EdgePolicy, ExcludeEdgeWorkCells,
                Ledger.CutBcyPerDepthM, 0, null);
            List<Lane> Fill = BuildKind(
                Ledger, WorkKind.Fill, Axis,
                Ledger.RequiredFillDepthM, EdgePolicy, ExcludeEdgeWorkCells,
                Ledger.FillBcyPerDepthM, Cut.Count, null);
            return new Inventory(Axis, Cut, Fill);
        }

        /// <summary>
        /// Builds contiguous work lanes for one cut or fill inventory.
        /// </summary>
        /// <param name="Ledger">Ledger containing the remaining field work.</param>
        /// <param name="Kind">Cut or fill work kind to include.</param>
        /// <param name="Axis">Axis along which lane cells are grouped.</param>
        /// <param name="RequiredDepthM">Required work depth for each grid cell.</param>
        /// <param name="EdgePolicy">Policy identifying restricted edge cells.</param>
        /// <param name="ExcludeEdgeWorkCells">Whether restricted edge work is omitted.</param>
        /// <param name="BcyPerDepthM">Volume conversion for one metre of cell depth.</param>
        /// <param name="StartingId">First identifier available to the generated lanes.</param>
        /// <param name="Eligible">Optional predicate restricting eligible cells.</param>
        /// <returns>The contiguous lanes covering all eligible work.</returns>
        private static List<Lane> BuildKind
            (
            HaulWorkLedger Ledger,
            WorkKind Kind,
            LaneAxis Axis,
            Double[,] RequiredDepthM,
            FieldEdgePolicy EdgePolicy,
            Boolean ExcludeEdgeWorkCells,
            Double BcyPerDepthM,
            Int32 StartingId,
            Func<Int32, Int32, Boolean>? Eligible
            )
        {
            Grid Grid = Ledger.Grid;
            Double MaximumDepthM = Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Int32 MaximumLayers = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    MaximumLayers = Math.Max(
                        MaximumLayers,
                        GridMath.LayersForDepth(
                            RequiredDepthM[By, Bx], Grid.MaxDepthMm));
                }
            }

            List<Lane> Output = new List<Lane>();
            Int32 NextId = StartingId;
            for (Int32 LayerIndex = 0; LayerIndex < MaximumLayers; LayerIndex += 1)
            {
                Int32 MajorCount = Axis == LaneAxis.EastWest
                    ? Grid.Height : Grid.Width;
                Int32 MinorCount = Axis == LaneAxis.EastWest
                    ? Grid.Width : Grid.Height;
                for (Int32 Major = 0; Major < MajorCount; Major += 1)
                {
                    List<LaneEntry> Current = new List<LaneEntry>();
                    for (Int32 Minor = 0; Minor < MinorCount; Minor += 1)
                    {
                        Int32 Bx = Axis == LaneAxis.EastWest ? Minor : Major;
                        Int32 By = Axis == LaneAxis.EastWest ? Major : Minor;
                        Double RemainingM = RequiredDepthM[By, Bx]
                            - (LayerIndex * MaximumDepthM);
                        Double DepthM = Math.Min(MaximumDepthM, RemainingM);
                        if (!Grid.Valid[By, Bx]
                            || (ExcludeEdgeWorkCells
                                && EdgePolicy.IsEdgeWorkCell((Bx, By)))
                            || ((Eligible != null) && !Eligible(Bx, By))
                            || (DepthM <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE))
                        {
                            FlushLane();
                            continue;
                        }

                        Current.Add(new LaneEntry(
                            (Bx, By), LayerIndex, DepthM,
                            DepthM * BcyPerDepthM));
                    }
                    FlushLane();

                    // Commits the current heading-aligned entries as one lane.
                    void FlushLane()
                    {
                        if (Current.Count == 0) return;
                        Output.Add(new Lane(
                            NextId, Kind, Axis, LayerIndex,
                            Current.ToList(), Grid));
                        NextId += 1;
                        Current.Clear();
                    }
                }
            }

            return Output;
        }

        /// <summary>
        /// Normalizes an undirected compass axis into [0, 180). Opposite
        /// bearings therefore select identical lane geometry.
        /// </summary>
        static public Double NormalizeAxisHeadingDeg(Double HeadingDeg)
        {
            if (!Double.IsFinite(HeadingDeg))
            {
                throw new ArgumentOutOfRangeException(nameof(HeadingDeg));
            }
            Double Normalized = HeadingDeg % Constants.DEGREES_PER_HALF_TURN;
            return Normalized < 0.0
                ? Normalized + Constants.DEGREES_PER_HALF_TURN
                : Normalized;
        }

        /// <summary>Builds an exact layer cover along one arbitrary axis.</summary>
        private static List<Lane> BuildHeadingKind
            (
            HaulWorkLedger Ledger,
            WorkKind Kind,
            Double[,] RequiredDepthM,
            FieldEdgePolicy EdgePolicy,
            Boolean ExcludeEdgeWorkCells,
            Double BcyPerDepthM,
            Int32 StartingId,
            Double HeadingAxisDeg
            )
        {
            Grid Grid = Ledger.Grid;
            Double MaximumDepthM = Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Int32 MaximumLayers = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    MaximumLayers = Math.Max(
                        MaximumLayers,
                        GridMath.LayersForDepth(
                            RequiredDepthM[By, Bx], Grid.MaxDepthMm));
                }
            }

            Double HeadingRad = HeadingAxisDeg * Math.PI
                / Constants.DEGREES_PER_HALF_TURN;
            Double AlongX = Math.Sin(HeadingRad);
            Double AlongY = Math.Cos(HeadingRad);
            Double CrossX = Math.Cos(HeadingRad);
            Double CrossY = -Math.Sin(HeadingRad);
            List<Lane> Output = new List<Lane>();
            Int32 NextId = StartingId;
            for (Int32 LayerIndex = 0;
                LayerIndex < MaximumLayers;
                LayerIndex += 1)
            {
                Dictionary<Int32, List<(LaneEntry Entry, Double Along)>> Bands =
                    new Dictionary<Int32,
                        List<(LaneEntry Entry, Double Along)>>();
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        Double RemainingM = RequiredDepthM[By, Bx]
                            - (LayerIndex * MaximumDepthM);
                        Double DepthM = Math.Min(MaximumDepthM, RemainingM);
                        if (!Grid.Valid[By, Bx]
                            || (ExcludeEdgeWorkCells
                                && EdgePolicy.IsEdgeWorkCell((Bx, By)))
                            || (DepthM <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE))
                        {
                            continue;
                        }
                        Double Along = (Bx * AlongX) + (By * AlongY);
                        Int32 Band = (Int32)Math.Round(
                            (Bx * CrossX) + (By * CrossY),
                            MidpointRounding.AwayFromZero);
                        if (!Bands.TryGetValue(
                            Band,
                            out List<(LaneEntry Entry, Double Along)>? Entries))
                        {
                            Entries = new List<(LaneEntry, Double)>();
                            Bands.Add(Band, Entries);
                        }
                        Entries.Add((
                            new LaneEntry(
                                (Bx, By),
                                LayerIndex,
                                DepthM,
                                DepthM * BcyPerDepthM),
                            Along));
                    }
                }

                foreach (KeyValuePair<Int32,
                    List<(LaneEntry Entry, Double Along)>> Band in Bands
                        .OrderBy(Pair => Pair.Key))
                {
                    List<LaneEntry> Current = new List<LaneEntry>();
                    (Int32 Bx, Int32 By)? Previous = null;
                    foreach ((LaneEntry Entry, Double Along) Item in Band.Value
                        .OrderBy(Item => Item.Along)
                        .ThenBy(Item => Item.Entry.Cell.Bx)
                        .ThenBy(Item => Item.Entry.Cell.By))
                    {
                        if (Previous.HasValue
                            && ((Math.Abs(Item.Entry.Cell.Bx
                                    - Previous.Value.Bx) > 1)
                                || (Math.Abs(Item.Entry.Cell.By
                                    - Previous.Value.By) > 1)))
                        {
                            FlushLane();
                        }
                        Current.Add(Item.Entry);
                        Previous = Item.Entry.Cell;
                    }
                    FlushLane();

                    // Commits the current contiguous entries as one lane.
                    void FlushLane()
                    {
                        if (Current.Count == 0) return;
                        Output.Add(new Lane(
                            NextId,
                            Kind,
                            LaneAxis.UserHeading,
                            LayerIndex,
                            Current.ToList(),
                            Grid,
                            HeadingAxisDeg));
                        NextId += 1;
                        Current.Clear();
                        Previous = null;
                    }
                }
            }
            return Output;
        }

        /// <summary>
        /// Builds work lanes whose local axes follow the dominant flow direction.
        /// </summary>
        /// <param name="Ledger">Ledger containing the remaining field work.</param>
        /// <param name="Kind">Cut or fill work kind to include.</param>
        /// <param name="RequiredDepthM">Required work depth for each grid cell.</param>
        /// <param name="EdgePolicy">Policy identifying restricted edge cells.</param>
        /// <param name="ExcludeEdgeWorkCells">Whether restricted edge work is omitted.</param>
        /// <param name="BcyPerDepthM">Volume conversion for one metre of cell depth.</param>
        /// <param name="StartingId">First identifier available to the generated lanes.</param>
        /// <param name="Flow">Flow field used to choose each cell's preferred axis.</param>
        /// <returns>The flow-aligned lanes covering all eligible work.</returns>
        private static List<Lane> BuildAdaptiveKind
            (
            HaulWorkLedger Ledger,
            WorkKind Kind,
            Double[,] RequiredDepthM,
            FieldEdgePolicy EdgePolicy,
            Boolean ExcludeEdgeWorkCells,
            Double BcyPerDepthM,
            Int32 StartingId,
            FlowField Flow
            )
        {
            // Selects the cardinal lane axis closest to the local flow vector
            // at the requested grid cell.
            LaneAxis PreferredAxis
                (
                Int32 Bx,
                Int32 By
                ) => Math.Abs(Flow.Fx[By, Bx])
                    >= Math.Abs(Flow.Fy[By, Bx])
                    ? LaneAxis.EastWest
                    : LaneAxis.NorthSouth;

            List<Lane> Output = BuildKind(
                Ledger,
                Kind,
                LaneAxis.EastWest,
                RequiredDepthM,
                EdgePolicy,
                ExcludeEdgeWorkCells,
                BcyPerDepthM,
                StartingId,
                (Bx, By) => PreferredAxis(Bx, By) == LaneAxis.EastWest);
            Output.AddRange(BuildKind(
                Ledger,
                Kind,
                LaneAxis.NorthSouth,
                RequiredDepthM,
                EdgePolicy,
                ExcludeEdgeWorkCells,
                BcyPerDepthM,
                StartingId + Output.Count,
                (Bx, By) => PreferredAxis(Bx, By) == LaneAxis.NorthSouth));
            return Output;
        }
    }
}
