using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Drive-order phase checks, the plan stamp, and the first-100 trip lines.
    /// </summary>
    public sealed class PlanOrderTests
    {
        /// <summary>
        /// Flow may include a band, then correcting, sweep-up, and finishing.
        /// </summary>
        [Fact]
        public void DriveOrder_AllowsForwardPhases()
        {
            List<TripGuidance.TripCourse> Trips = new List<TripGuidance.TripCourse>
            {
                Trip(1, HaulStage.Flow, false, false, true, 10.0),
                Trip(2, HaulStage.Clusters, false, false, false, 4.0),
                Trip(3, HaulStage.Correcting, false, false, false, 2.0),
                Trip(4, HaulStage.Correcting, true, false, false, 1.0),
                Trip(5, HaulStage.Flow, false, true, false, 0.5),
            };
            List<Int32> Drive = new List<Int32> { 0, 1, 2, 3, 4 };
            PlanOrder.Require(Trips, Drive);
            Assert.Equal("flow", PlanOrder.PhaseName(Trips[0]));
            Assert.Equal("flow ribbon", PlanOrder.SourceName(Trips[0]));
            Assert.Equal("band", PlanOrder.SourceName(Trips[1]));
            Assert.Equal("correcting line", PlanOrder.SourceName(Trips[2]));
            Assert.Equal("sweep-up", PlanOrder.SourceName(Trips[3]));
            Assert.Equal("finishing", PlanOrder.PhaseName(Trips[4]));
        }

        /// <summary>
        /// The road stage sits after flow and before correcting.
        /// </summary>
        [Fact]
        public void DriveOrder_PutsTheRoadStageAfterFlow()
        {
            List<TripGuidance.TripCourse> Trips = new List<TripGuidance.TripCourse>
            {
                Trip(1, HaulStage.Flow, false, false, true, 10.0),
                Trip(2, HaulStage.Road, false, false, false, 3.0),
                Trip(3, HaulStage.Correcting, false, false, false, 2.0),
                Trip(4, HaulStage.Correcting, true, false, false, 1.0),
                Trip(5, HaulStage.Flow, false, true, false, 0.5),
            };
            PlanOrder.Require(Trips, new List<Int32> { 0, 1, 2, 3, 4 });
            Assert.Equal("road", PlanOrder.PhaseName(Trips[1]));
            Assert.Equal("road", PlanOrder.SourceName(Trips[1]));
            Assert.Equal("correcting", PlanOrder.PhaseName(Trips[2]));
            Assert.Equal("sweep-up", PlanOrder.PhaseName(Trips[3]));
            Assert.Equal("finishing", PlanOrder.PhaseName(Trips[4]));
        }

        /// <summary>
        /// A road trip after correcting fails the run.
        /// </summary>
        [Fact]
        public void DriveOrder_RejectsARoadTripAfterCorrecting()
        {
            List<TripGuidance.TripCourse> Trips = new List<TripGuidance.TripCourse>
            {
                Trip(1, HaulStage.Correcting, false, false, false, 2.0),
                Trip(2, HaulStage.Road, false, false, false, 3.0),
            };
            InvalidOperationException Caught = Assert.Throws<InvalidOperationException>(
                () => PlanOrder.Require(Trips, new List<Int32> { 0, 1 }));
            Assert.Contains("road after correcting", Caught.Message);
        }

        /// <summary>
        /// A flow trip after correcting fails the run.
        /// </summary>
        [Fact]
        public void DriveOrder_RejectsABackwardPhase()
        {
            List<TripGuidance.TripCourse> Trips = new List<TripGuidance.TripCourse>
            {
                Trip(1, HaulStage.Correcting, false, false, false, 2.0),
                Trip(2, HaulStage.Flow, false, false, true, 10.0),
            };
            InvalidOperationException Caught = Assert.Throws<InvalidOperationException>(
                () => PlanOrder.Require(Trips, new List<Int32> { 0, 1 }));
            Assert.Contains("flow after correcting", Caught.Message);
        }

        /// <summary>
        /// The header hours, trip count, and stamp are the trips that were saved.
        /// A different stamp warns.
        /// </summary>
        [Fact]
        public void Report_HeaderMatchesTheTrips_AndWarnsOnABadStamp()
        {
            List<TripGuidance.TripCourse> Trips = new List<TripGuidance.TripCourse>
            {
                Trip(1, HaulStage.Flow, false, false, true, 12.5, 3600.0),
                Trip(7, HaulStage.Correcting, false, false, false, 3.25, 0.0),
            };
            List<Int32> Drive = new List<Int32> { 0, 1 };
            TripGuidance.JobGuidance Job = new TripGuidance.JobGuidance(
                Trips,
                new Double[1, 1],
                null,
                1.0,
                new Dictionary<Int32, Double[,]>(),
                0.0,
                0.0,
                Drive,
                0,
                0,
                0);
            String Report = Job.FormatReport(0.0);
            Assert.Contains("Trips 2", Report);
            Assert.Contains("Hours 1.000", Report);
            Assert.Contains("this build 1.000 h", Report);
            Assert.Contains("drive 1 trip 1 stage flow source flow ribbon cut 12.5000 CY loaded-crosses-fill no", Report);
            Assert.Contains("drive 2 trip 7 stage correcting source correcting line cut 3.2500 CY loaded-crosses-fill no", Report);
            String Stamp = StampOf(Report);
            Assert.Equal(8, Stamp.Length);
            Assert.Equal("", PlanOrder.ViewerWarning(Stamp, Trips, Drive, 1.0));
            String Warning = PlanOrder.ViewerWarning("00000000", Trips, Drive, 1.0);
            Assert.Contains("does not match the trips", Warning);
        }

        /// <summary>
        /// A loaded connector that steps on a later fill cell is reported.
        /// The spread-start cell is the arrival, not a crossing.
        /// </summary>
        [Fact]
        public void LoadedPath_ReportsAFillCellItCrosses()
        {
            Grid Grid = TinyGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((0, 0), 0.1),
                    ((1, 0), 0.1),
                };
            TripGuidance.TripCourse Clear = CarryAt(1.0, 1.0);
            TripGuidance.TripCourse Hit = CarryAt(3.0, 1.0);
            Assert.False(PlanOrder.LoadedCrossesFill(Clear, Grid, Fills));
            Assert.True(PlanOrder.LoadedCrossesFill(Hit, Grid, Fills));
        }

        /// <summary>
        /// Reads the stamp line from a report.
        /// </summary>
        /// <param name="Report">Report text.</param>
        /// <returns>The eight digits.</returns>
        private static String StampOf
            (
            String Report
            )
        {
            foreach (String Line in Report.Split('\n'))
            {
                String Trimmed = Line.Trim();
                if (Trimmed.StartsWith("Stamp ", StringComparison.Ordinal))
                {
                    return Trimmed.Substring("Stamp ".Length);
                }
            }

            throw new InvalidOperationException("The report has no stamp.");
        }

        /// <summary>
        /// One trip with the fields the order check and the report read.
        /// </summary>
        /// <param name="Number">1-based trip number.</param>
        /// <param name="Stage">Builder that made the trip.</param>
        /// <param name="SweepUp">True for a sweep-up trip.</param>
        /// <param name="Finishing">True for a finishing trip.</param>
        /// <param name="Ribbon">True when the cells are a flow ribbon.</param>
        /// <param name="CutBcy">Cut load in bank cubic yards.</param>
        /// <param name="CutSeconds">Productive cut seconds.</param>
        /// <returns>The trip.</returns>
        private static TripGuidance.TripCourse Trip
            (
            Int32 Number,
            HaulStage Stage,
            Boolean SweepUp,
            Boolean Finishing,
            Boolean Ribbon,
            Double CutBcy,
            Double CutSeconds = 0.0
            )
        {
            TripGuidance.TripCourse Built = new TripGuidance.TripCourse(
                Number,
                Stage,
                new List<TripGuidance.CourseSegment>(),
                new List<TripGuidance.GuidancePoint>(),
                new List<TripGuidance.ConnectorNote>(),
                CutSeconds,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                new TripGuidance.GradeLedger(0.0, 0.0, 0.0));
            Built.SweepUp = SweepUp;
            Built.Finishing = Finishing;
            Built.RibbonOrder = Ribbon;
            Built.CutBcy = CutBcy;
            return Built;
        }

        /// <summary>
        /// A carry whose only sample is the given point.
        /// </summary>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <returns>The trip.</returns>
        private static TripGuidance.TripCourse CarryAt
            (
            Double X,
            Double Y
            )
        {
            DubinsConnector.Pose Sample = new DubinsConnector.Pose(X, Y, 0.0);
            TripGuidance.DrawnPiece Piece = new TripGuidance.DrawnPiece(
                DubinsConnector.PieceKind.Straight,
                1.0,
                0.0,
                new List<DubinsConnector.Pose> { Sample },
                null);
            TripGuidance.CourseSegment Segment = new TripGuidance.CourseSegment(
                TripGuidance.GuidanceAction.Carry,
                false,
                false,
                "S",
                0.0,
                0.0,
                1.0,
                1.0,
                new List<TripGuidance.DrawnPiece> { Piece });
            return new TripGuidance.TripCourse(
                1,
                HaulStage.Flow,
                new List<TripGuidance.CourseSegment> { Segment },
                new List<TripGuidance.GuidancePoint>(),
                new List<TripGuidance.ConnectorNote>(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                new TripGuidance.GradeLedger(0.0, 0.0, 0.0));
        }

        /// <summary>
        /// Two cells along east, each 2 m, origin at zero.
        /// </summary>
        /// <returns>The grid.</returns>
        private static Grid TinyGrid()
        {
            return new Grid(
                2.0,
                2,
                1,
                0.0,
                0.0,
                1,
                true,
                0.0,
                0.0,
                new Double[1, 2],
                new Double[1, 2],
                new Boolean[1, 2],
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
