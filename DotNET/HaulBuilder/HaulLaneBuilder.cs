using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder
{
    /// <summary>
    /// Builds deterministic blade-width work lanes before cut/fill ownership is paired.
    /// </summary>
    public static class HaulLaneBuilder
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
                Grid Grid
                )
            {
                this.Id = Id;
                this.Kind = Kind;
                this.Axis = Axis;
                this.LayerIndex = LayerIndex;
                this.Entries = Entries;
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
            FlowField? Flow = null
            )
        {
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            FieldEdgePolicy EdgePolicy = new FieldEdgePolicy(
                Ledger.Grid, Ledger.Grid.GridSizeM);
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
            Double MaximumDepthM = Grid.MaxDepthMm / 1000.0;
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
                            || (DepthM <= 1e-12))
                        {
                            FlushLane();
                            continue;
                        }

                        Current.Add(new LaneEntry(
                            (Bx, By), LayerIndex, DepthM,
                            DepthM * BcyPerDepthM));
                    }
                    FlushLane();

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
