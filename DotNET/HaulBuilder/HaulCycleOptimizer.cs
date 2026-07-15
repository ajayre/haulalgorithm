using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Pairs lane-first cut and fill work into capacity-balanced machine cycles.
    /// </summary>
    public static class HaulCycleOptimizer
    {
        private const Double VOLUME_TOLERANCE_BCY = 1e-8;
        private const Double DEFAULT_MINIMUM_TURN_RADIUS_M = 7.62;

        /// <summary>Builds one complete axis-lane candidate.</summary>
        static public HaulOptimizationResult Optimize
            (
            Grid Grid,
            FlowField Flow,
            HaulWorkLedger Ledger,
            HaulLaneBuilder.LaneAxis Axis,
            Double MaximumLoadBcy,
            Int32 MaximumRunsPerKind = 2,
            Boolean DeferEdgeWork = false,
            Double FlowPenaltyWeight = 0.20,
            ObstacleField? Obstacles = null
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Flow == null) throw new ArgumentNullException(nameof(Flow));
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            if (MaximumLoadBcy <= 0.0) throw new ArgumentOutOfRangeException(nameof(MaximumLoadBcy));
            if (MaximumRunsPerKind <= 0) throw new ArgumentOutOfRangeException(nameof(MaximumRunsPerKind));
            if (FlowPenaltyWeight < 0.0) throw new ArgumentOutOfRangeException(nameof(FlowPenaltyWeight));

            HaulLaneBuilder.Inventory Inventory = HaulLaneBuilder.Build(
                Ledger, Axis, DeferEdgeWork, Flow);
            List<LaneState> CutStates = Inventory.CutLanes
                .Select(Lane => new LaneState(Lane)).ToList();
            List<LaneState> FillStates = Inventory.FillLanes
                .Select(Lane => new LaneState(Lane)).ToList();
            List<Pass> PassList = new List<Pass>();
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            Double TotalAssignedBcy = 0.0;

            while (RemainingVolume(CutStates) > VOLUME_TOLERANCE_BCY
                && RemainingVolume(FillStates) > VOLUME_TOLERANCE_BCY)
            {
                LaneState? PrimaryCut = SelectPrimaryCut(CutStates, MaximumLoadBcy);
                if (PrimaryCut == null) break;
                List<LaneState> CutGroup = BuildGroup(
                    PrimaryCut, CutStates, MaximumLoadBcy,
                    MaximumRunsPerKind, Grid);
                Double CutAvailableBcy = Math.Min(
                    MaximumLoadBcy,
                    CutGroup.Sum(State => State.RemainingVolumeBcy));
                if (CutAvailableBcy <= VOLUME_TOLERANCE_BCY) break;

                List<LaneState>? FillGroup = SelectFillGroup(
                    PrimaryCut, FillStates, CutAvailableBcy,
                    MaximumLoadBcy, MaximumRunsPerKind,
                    Grid, Flow, FlowPenaltyWeight);
                if ((FillGroup == null) || (FillGroup.Count == 0)) break;
                Double FillAvailableBcy = Math.Min(
                    MaximumLoadBcy,
                    FillGroup.Sum(State => State.RemainingVolumeBcy));
                Double TargetBcy = Math.Min(CutAvailableBcy, FillAvailableBcy);
                if (TargetBcy <= VOLUME_TOLERANCE_BCY) break;

                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells =
                    Consume(CutGroup, TargetBcy, Ledger.CutBcyPerDepthM);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                    Consume(FillGroup, TargetBcy, Ledger.FillBcyPerDepthM);
                Double ActualCutBcy = CutCells.Sum(
                    Entry => Entry.DepthM * Ledger.CutBcyPerDepthM);
                Double ActualFillBcy = FillCells.Sum(
                    Entry => Entry.DepthM * Ledger.FillBcyPerDepthM);
                if ((CutCells.Count == 0) || (FillCells.Count == 0)
                    || (Math.Abs(ActualCutBcy - ActualFillBcy) > VOLUME_TOLERANCE_BCY))
                {
                    throw new InvalidOperationException(
                        "Lane cycle consumption did not produce a balanced load.");
                }

                Pass Pass = HaulBuilder.Passes.BuildPass(
                    Grid, CutCells, FillCells, ActualCutBcy,
                    ActiveObstacles, Ordered: true);
                Pass.CutHeadingDeg = PrimaryCut.Lane.Axis
                    == HaulLaneBuilder.LaneAxis.EastWest
                    ? 90.0 : 0.0;
                Pass.FillHeadingDeg = FillGroup[0].Lane.Axis
                    == HaulLaneBuilder.LaneAxis.EastWest
                    ? 90.0 : 0.0;
                PassList.Add(Pass);
                TotalAssignedBcy += ActualCutBcy;
            }

            IReadOnlyList<HaulDeferredWork> DeferredWork =
                Ledger.BuildDeferredWork(
                    PassList,
                    DeferEdgeWork
                        ? "FIELD_EDGE_HEADING"
                        : "MATERIAL_IMBALANCE");
            Double UnmatchedCutBcy = DeferredWork
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
                .Sum(Entry => Entry.VolumeBcy);
            Double UnmatchedFillBcy = DeferredWork
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
                .Sum(Entry => Entry.VolumeBcy);
            if (!DeferEdgeWork)
            {
                Double ExpectedCutBcy = Math.Max(
                    0.0, Ledger.TotalCutBcy - Ledger.TotalFillBcy);
                Double ExpectedFillBcy = Math.Max(
                    0.0, Ledger.TotalFillBcy - Ledger.TotalCutBcy);
                if ((Math.Abs(UnmatchedCutBcy - ExpectedCutBcy) > 1e-6)
                    || (Math.Abs(UnmatchedFillBcy - ExpectedFillBcy) > 1e-6))
                {
                    throw new InvalidOperationException(
                        "The complete optimizer left work deferred beyond the exact material imbalance.");
                }
            }
            PassResult PassResult = new PassResult(
                PassList, 1, UnmatchedCutBcy, UnmatchedFillBcy);
            HaulPlanValidation Validation = Ledger.ValidateComplete(
                PassList, MaximumLoadBcy, Grid.MaxDepthMm / 1000.0,
                AllowDeferredWork: true);
            return new HaulOptimizationResult(
                PassResult,
                Axis,
                Inventory.CutLanes.Count,
                Inventory.FillLanes.Count,
                TotalAssignedBcy,
                UnmatchedCutBcy,
                UnmatchedFillBcy,
                DeferredWork,
                Validation);
        }

        private static LaneState? SelectPrimaryCut
            (
            List<LaneState> States,
            Double MaximumLoadBcy
            )
        {
            return States
                .Where(State => State.RemainingVolumeBcy > VOLUME_TOLERANCE_BCY)
                .OrderByDescending(State => Math.Min(
                    State.RemainingVolumeBcy, MaximumLoadBcy))
                .ThenBy(State => State.Lane.Id)
                .FirstOrDefault();
        }

        private static List<LaneState> BuildGroup
            (
            LaneState Primary,
            List<LaneState> States,
            Double TargetBcy,
            Int32 MaximumRuns,
            Grid Grid
            )
        {
            List<LaneState> Output = new List<LaneState> { Primary };
            Double AvailableBcy = Primary.RemainingVolumeBcy;
            while ((Output.Count < MaximumRuns)
                && (AvailableBcy < TargetBcy - VOLUME_TOLERANCE_BCY))
            {
                HashSet<(Int32 Bx, Int32 By)> UsedCells = Output
                    .SelectMany(State => State.RemainingCells())
                    .ToHashSet();
                LaneState? Next = States
                    .Where(State => !Output.Contains(State)
                        && (State.RemainingVolumeBcy > VOLUME_TOLERANCE_BCY)
                        && (State.Lane.LayerIndex == Primary.Lane.LayerIndex)
                        && (State.Lane.Axis == Primary.Lane.Axis)
                        && !State.RemainingCells().Any(Cell => UsedCells.Contains(Cell)))
                    .OrderBy(State => LaneDistance(Primary, State, Grid))
                    .ThenBy(State => State.Lane.Id)
                    .FirstOrDefault();
                if (Next == null) break;
                Output.Add(Next);
                AvailableBcy += Next.RemainingVolumeBcy;
            }
            return Output;
        }

        private static List<LaneState>? SelectFillGroup
            (
            LaneState Cut,
            List<LaneState> FillStates,
            Double DesiredBcy,
            Double MaximumLoadBcy,
            Int32 MaximumRuns,
            Grid Grid,
            FlowField Flow,
            Double FlowPenaltyWeight
            )
        {
            List<LaneState> Candidates = FillStates
                .Where(State => State.RemainingVolumeBcy > VOLUME_TOLERANCE_BCY)
                .OrderBy(State => PairCost(
                    Cut,
                    State,
                    DesiredBcy,
                    Grid,
                    Flow,
                    FlowPenaltyWeight))
                .ThenByDescending(State => Math.Min(
                    State.RemainingVolumeBcy, MaximumLoadBcy))
                .ThenBy(State => State.Lane.Id)
                .Take(64)
                .ToList();
            List<LaneState>? Best = null;
            Double BestCost = Double.PositiveInfinity;
            foreach (LaneState Candidate in Candidates)
            {
                List<LaneState> Group = BuildGroup(
                    Candidate, FillStates, DesiredBcy,
                    MaximumRuns, Grid);
                Double AvailableBcy = Math.Min(
                    MaximumLoadBcy,
                    Group.Sum(State => State.RemainingVolumeBcy));
                Double ShortfallBcy = Math.Max(0.0, DesiredBcy - AvailableBcy);
                Double Cost = PairCost(
                    Cut,
                    Candidate,
                    DesiredBcy,
                    Grid,
                    Flow,
                    FlowPenaltyWeight)
                    + (ShortfallBcy * 10000.0);
                if (Cost < BestCost)
                {
                    BestCost = Cost;
                    Best = Group;
                }
            }
            return Best;
        }

        private static Double PairCost
            (
            LaneState Cut,
            LaneState Fill,
            Double DesiredBcy,
            Grid Grid,
            FlowField Flow,
            Double FlowPenaltyWeight
            )
        {
            (Double X, Double Y) CutPoint = Cut.CurrentCentroid(Grid);
            (Double X, Double Y) FillPoint = Fill.CurrentCentroid(Grid);
            Double Dx = FillPoint.X - CutPoint.X;
            Double Dy = FillPoint.Y - CutPoint.Y;
            Double DistanceM = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (DistanceM <= 1e-9) return 0.0;
            (Int32 Bx, Int32 By) CutCell = Cut.CurrentCell;
            Double Fx = Flow.Fx[CutCell.By, CutCell.Bx];
            Double Fy = Flow.Fy[CutCell.By, CutCell.Bx];
            Double Dot = ((Dx / DistanceM) * Fx) + ((Dy / DistanceM) * Fy);
            Double AgainstFlowPenalty = FlowPenaltyWeight
                * Math.Max(0.0, -Dot) * DistanceM * 4.0;
            Double CrossFlowPenalty = FlowPenaltyWeight
                * (1.0 - Math.Abs(Dot)) * DistanceM * 0.35;
            Double ConnectorLowerBoundM = ConnectorLowerBound(
                Cut,
                Fill,
                DesiredBcy,
                Grid);
            return ConnectorLowerBoundM
                + AgainstFlowPenalty
                + CrossFlowPenalty;
        }

        private static Double ConnectorLowerBound
            (
            LaneState Cut,
            LaneState Fill,
            Double DesiredBcy,
            Grid Grid
            )
        {
            (Double X, Double Y) CutFirst = Grid.CellCenterXy(
                Cut.CurrentCell.Bx, Cut.CurrentCell.By);
            (Int32 Bx, Int32 By) CutLastCell = Cut.SliceEndCell(
                DesiredBcy,
                Grid.AreaM2 * Constants.M3_TO_CY);
            (Double X, Double Y) CutLast = Grid.CellCenterXy(
                CutLastCell.Bx, CutLastCell.By);
            (Double X, Double Y) FillFirst = Grid.CellCenterXy(
                Fill.CurrentCell.Bx, Fill.CurrentCell.By);
            (Int32 Bx, Int32 By) FillLastCell = Fill.SliceEndCell(
                DesiredBcy,
                Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction);
            (Double X, Double Y) FillLast = Grid.CellCenterXy(
                FillLastCell.Bx, FillLastCell.By);
            Double CutForwardHeading = LaneHeading(
                CutFirst, CutLast, Cut.Lane.Axis);
            Double FillForwardHeading = LaneHeading(
                FillFirst, FillLast, Fill.Lane.Axis);
            ConnectorPlanner.Pose[] CutExits =
            {
                new ConnectorPlanner.Pose(
                    CutLast.X, CutLast.Y, CutForwardHeading),
                new ConnectorPlanner.Pose(
                    CutFirst.X,
                    CutFirst.Y,
                    (CutForwardHeading + 180.0) % 360.0),
            };
            ConnectorPlanner.Pose[] FillEntries =
            {
                new ConnectorPlanner.Pose(
                    FillFirst.X, FillFirst.Y, FillForwardHeading),
                new ConnectorPlanner.Pose(
                    FillLast.X,
                    FillLast.Y,
                    (FillForwardHeading + 180.0) % 360.0),
            };
            Double BestM = Double.PositiveInfinity;
            foreach (ConnectorPlanner.Pose CutExit in CutExits)
            {
                foreach (ConnectorPlanner.Pose FillEntry in FillEntries)
                {
                    BestM = Math.Min(
                        BestM,
                        ConnectorPlanner.ShortestPathLengthM(
                            CutExit,
                            FillEntry,
                            DEFAULT_MINIMUM_TURN_RADIUS_M));
                }
            }
            return BestM;
        }

        private static Double LaneHeading
            (
            (Double X, Double Y) First,
            (Double X, Double Y) Last,
            HaulLaneBuilder.LaneAxis Axis
            )
        {
            Double Dx = Last.X - First.X;
            Double Dy = Last.Y - First.Y;
            if ((Math.Abs(Dx) <= 1e-9) && (Math.Abs(Dy) <= 1e-9))
            {
                return Axis == HaulLaneBuilder.LaneAxis.EastWest
                    ? 90.0 : 0.0;
            }
            return (Math.Atan2(Dx, Dy) * 180.0 / Math.PI + 360.0) % 360.0;
        }

        private static Double LaneDistance
            (
            LaneState First,
            LaneState Second,
            Grid Grid
            )
        {
            (Double X, Double Y) A = First.CurrentCentroid(Grid);
            (Double X, Double Y) B = Second.CurrentCentroid(Grid);
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Consume
            (
            List<LaneState> States,
            Double TargetBcy,
            Double BcyPerDepthM
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Output =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double RemainingBcy = TargetBcy;
            foreach (LaneState State in States)
            {
                while ((RemainingBcy > VOLUME_TOLERANCE_BCY)
                    && (State.RemainingVolumeBcy > VOLUME_TOLERANCE_BCY))
                {
                    Double AvailableDepthM = State.CurrentDepthM;
                    Double AvailableBcy = AvailableDepthM * BcyPerDepthM;
                    Double TakeBcy = Math.Min(AvailableBcy, RemainingBcy);
                    Double TakeDepthM = TakeBcy / BcyPerDepthM;
                    Output.Add((State.CurrentCell, TakeDepthM));
                    State.Consume(TakeDepthM, TakeBcy);
                    RemainingBcy -= TakeBcy;
                }
                if (RemainingBcy <= VOLUME_TOLERANCE_BCY) break;
            }
            if (RemainingBcy > VOLUME_TOLERANCE_BCY)
            {
                throw new InvalidOperationException(
                    "Lane group did not contain the promised target volume.");
            }
            return Output;
        }

        private static Double RemainingVolume
            (
            IEnumerable<LaneState> States
            )
        {
            return States.Sum(State => State.RemainingVolumeBcy);
        }

        private sealed class LaneState
        {
            private Int32 _EntryIndex;
            private Double _CurrentDepthM;

            /// <summary>Initializes mutable consumption state for one immutable lane.</summary>
            public LaneState
                (
                HaulLaneBuilder.Lane Lane
                )
            {
                this.Lane = Lane;
                _EntryIndex = 0;
                _CurrentDepthM = Lane.Entries.Count > 0
                    ? Lane.Entries[0].DepthM : 0.0;
                RemainingVolumeBcy = Lane.VolumeBcy;
            }

            /// <summary>Gets immutable lane.</summary>
            public HaulLaneBuilder.Lane Lane { get; }

            /// <summary>Gets remaining bank volume.</summary>
            public Double RemainingVolumeBcy { get; private set; }

            /// <summary>Gets current physical cell.</summary>
            public (Int32 Bx, Int32 By) CurrentCell =>
                Lane.Entries[_EntryIndex].Cell;

            /// <summary>Gets current unconsumed entry depth.</summary>
            public Double CurrentDepthM => _CurrentDepthM;

            /// <summary>Consumes depth and volume from the current entry.</summary>
            public void Consume
                (
                Double DepthM,
                Double VolumeBcy
                )
            {
                _CurrentDepthM -= DepthM;
                RemainingVolumeBcy -= VolumeBcy;
                if (RemainingVolumeBcy < VOLUME_TOLERANCE_BCY)
                {
                    RemainingVolumeBcy = 0.0;
                }
                if (_CurrentDepthM <= 1e-12)
                {
                    _EntryIndex += 1;
                    if (_EntryIndex < Lane.Entries.Count)
                    {
                        _CurrentDepthM = Lane.Entries[_EntryIndex].DepthM;
                    }
                    else
                    {
                        _EntryIndex = Lane.Entries.Count - 1;
                        _CurrentDepthM = 0.0;
                    }
                }
            }

            /// <summary>Gets remaining physical cells.</summary>
            public IEnumerable<(Int32 Bx, Int32 By)> RemainingCells
                (
                )
            {
                if (RemainingVolumeBcy <= VOLUME_TOLERANCE_BCY)
                {
                    return Array.Empty<(Int32 Bx, Int32 By)>();
                }
                return Lane.Entries
                    .Skip(_EntryIndex)
                    .Select(Entry => Entry.Cell);
            }

            /// <summary>Finds the last cell in a prospective prefix slice.</summary>
            public (Int32 Bx, Int32 By) SliceEndCell
                (
                Double TargetBcy,
                Double BcyPerDepthM
                )
            {
                Double RemainingBcy = Math.Min(
                    TargetBcy, RemainingVolumeBcy);
                Int32 Index = _EntryIndex;
                Double DepthM = _CurrentDepthM;
                while ((RemainingBcy > VOLUME_TOLERANCE_BCY)
                    && (Index < Lane.Entries.Count))
                {
                    Double AvailableBcy = DepthM * BcyPerDepthM;
                    if (RemainingBcy <= AvailableBcy + VOLUME_TOLERANCE_BCY)
                    {
                        return Lane.Entries[Index].Cell;
                    }
                    RemainingBcy -= AvailableBcy;
                    Index += 1;
                    if (Index < Lane.Entries.Count)
                    {
                        DepthM = Lane.Entries[Index].DepthM;
                    }
                }
                return Lane.Entries[Math.Min(
                    Lane.Entries.Count - 1,
                    Math.Max(_EntryIndex, Index - 1))].Cell;
            }

            /// <summary>Gets centroid of the remaining lane suffix.</summary>
            public (Double X, Double Y) CurrentCentroid
                (
                Grid Grid
                )
            {
                if (RemainingVolumeBcy <= VOLUME_TOLERANCE_BCY)
                {
                    return Lane.CentroidXy;
                }
                IReadOnlyList<HaulLaneBuilder.LaneEntry> Entries = Lane.Entries;
                Double SumX = 0.0;
                Double SumY = 0.0;
                Int32 Count = 0;
                for (Int32 Index = _EntryIndex; Index < Entries.Count; Index += 1)
                {
                    SumX += Grid.CenterX[Entries[Index].Cell.By, Entries[Index].Cell.Bx];
                    SumY += Grid.CenterY[Entries[Index].Cell.By, Entries[Index].Cell.Bx];
                    Count += 1;
                }
                return Count > 0
                    ? (SumX / Count, SumY / Count)
                    : Lane.CentroidXy;
            }
        }
    }
}
