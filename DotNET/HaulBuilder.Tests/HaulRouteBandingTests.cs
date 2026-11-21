using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    /// <summary>Tests early full-strip route banding.</summary>
    public sealed class HaulRouteBandingTests
    {
        /// <summary>
        /// Skipping a far hop picks the same next load as pricing every hop.
        /// </summary>
        [Fact]
        public void FarHopSkipMatchesTheFullNeighborScan
            (
            )
        {
            String? Mismatch = Routing.NeighborOrderMismatch();
            Assert.True(Mismatch == null, Mismatch);
        }

        /// <summary>
        /// A short edge load at the front of the list is driven after the
        /// one-strip load and the multi-strip load.
        /// </summary>
        [Fact]
        public void FullStripsPrecedeOtherInteriorAndEdgeLoads
            (
            )
        {
            Pass Edge = Strip(
                new[] { (50, 50) },
                new[] { (50, 60) },
                90.0);
            Pass Multi = Strip(
                new[] { (0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1) },
                new[] { (0, 20), (1, 20), (2, 20) },
                90.0);
            Pass Full = Strip(
                new[] { (10, 0), (11, 0), (12, 0) },
                new[] { (10, 20), (11, 20), (12, 20) },
                90.0);
            List<Pass> Passes = new List<Pass> { Edge, Multi, Full };
            Route Unbanded = Routing.SequencePasses(Passes, StartCandidateCount: 1);

            HaulRouteBanding.Result Banded = HaulRouteBanding.Apply(
                Passes,
                EdgeCells(Edge.CutCells),
                EdgeCells(Edge.FillCells),
                StartCandidateCount: 1,
                ReorderWindow: 200);

            Assert.Equal(0, Unbanded.Order[0]);
            Assert.Equal(1, Banded.FullStripCount);
            Assert.Equal(1, Banded.OtherInteriorCount);
            Assert.Equal(0, Banded.EdgeCount);
            Assert.Equal(1, Banded.ScrapCount);
            Assert.Same(Full, Banded.Route.OrderedPasses[0]);
            Assert.Same(Multi, Banded.Route.OrderedPasses[1]);
            Assert.Same(Edge, Banded.Route.OrderedPasses[2]);
            Assert.Equal(new[] { 2, 1, 0 }, Banded.Route.Order);
        }

        /// <summary>
        /// A one-cell scrap stays behind a full edge load, even when the
        /// scrap is the closer drive.
        /// </summary>
        [Fact]
        public void ScrapStaysBehindAFullEdgeLoad
            (
            )
        {
            Pass Full = At(
                Strip(new[] { (0, 0), (1, 0), (2, 0) }, new[] { (0, 4), (1, 4), (2, 4) }, 90.0),
                (0.0, 0.0));
            Pass Edge = At(
                Strip(new[] { (10, 0), (11, 0), (12, 0) }, new[] { (10, 4), (11, 4), (12, 4) }, 90.0),
                (0.0, 30.0));
            Pass Scrap = At(
                Strip(new[] { (0, 2) }, new[] { (1, 2) }, 90.0),
                (0.0, 3.0));

            HaulRouteBanding.Result Banded = HaulRouteBanding.Apply(
                new List<Pass> { Scrap, Edge, Full },
                EdgeCells(Edge.CutCells),
                EdgeCells(Edge.FillCells),
                StartCandidateCount: 1);

            Assert.Equal(1, Banded.FullStripCount);
            Assert.Equal(1, Banded.EdgeCount);
            Assert.Equal(1, Banded.ScrapCount);
            Assert.Same(Full, Banded.Route.OrderedPasses[0]);
            Assert.Same(Edge, Banded.Route.OrderedPasses[1]);
            Assert.Same(Scrap, Banded.Route.OrderedPasses[2]);
        }

        /// <summary>A one-strip edge load stays in the edge band.</summary>
        [Fact]
        public void OneStripEdgeLoadStaysLast
            (
            )
        {
            Pass Edge = Strip(
                new[] { (4, 0), (5, 0), (6, 0) },
                new[] { (4, 8), (5, 8), (6, 8) },
                90.0);
            Pass Full = Strip(
                new[] { (0, 0), (1, 0), (2, 0) },
                new[] { (0, 3), (1, 3), (2, 3) },
                90.0);

            HaulRouteBanding.Result Banded = HaulRouteBanding.Apply(
                new List<Pass> { Edge, Full },
                EdgeCells(Edge.CutCells),
                EdgeCells(Edge.FillCells));

            Assert.Equal(1, Banded.FullStripCount);
            Assert.Equal(1, Banded.EdgeCount);
            Assert.Same(Full, Banded.Route.OrderedPasses[0]);
            Assert.Same(Edge, Banded.Route.OrderedPasses[1]);
        }

        /// <summary>The reorder window cannot pull an edge load in among the full strips.</summary>
        [Fact]
        public void ReorderWindowStaysInsideTheBand
            (
            )
        {
            Pass First = At(
                Strip(new[] { (0, 0), (1, 0), (2, 0) }, new[] { (0, 4), (1, 4), (2, 4) }, 90.0),
                (0.0, 0.0));
            Pass Second = At(
                Strip(new[] { (0, 10), (1, 10), (2, 10) }, new[] { (0, 14), (1, 14), (2, 14) }, 90.0),
                (0.0, 100.0));
            Pass Edge = At(
                Strip(new[] { (0, 5) }, new[] { (1, 5) }, 90.0),
                (0.0, 10.0));

            HaulRouteBanding.Result Banded = HaulRouteBanding.Apply(
                new List<Pass> { Edge, Second, First },
                EdgeCells(Edge.CutCells),
                EdgeCells(Edge.FillCells),
                ReorderWindow: 50);

            Assert.Equal(2, Banded.FullStripCount);
            Assert.Same(Edge, Banded.Route.OrderedPasses[Banded.Route.OrderedPasses.Count - 1]);
            Assert.DoesNotContain(
                Banded.Route.OrderedPasses.Take(Banded.FullStripCount),
                Pass => Object.ReferenceEquals(Pass, Edge));
        }

        /// <summary>A gap, a second lane, and a diagonal lane classify as expected.</summary>
        [Fact]
        public void StripShapeFollowsOneLane
            (
            )
        {
            Pass Gapped = Strip(
                new[] { (0, 0), (1, 0), (3, 0) },
                new[] { (0, 5), (1, 5) },
                90.0);
            Pass AdjacentLanes = Strip(
                new[] { (0, 0), (1, 0), (0, 1), (1, 1) },
                new[] { (0, 5), (1, 5) },
                0.0);
            Pass Diagonal = Strip(
                new[] { (0, 0), (1, 1), (2, 2) },
                new[] { (0, 4), (1, 5), (2, 6) },
                45.0);
            Pass Repeated = Strip(
                new[] { (0, 0), (0, 0), (1, 0) },
                new[] { (0, 2), (1, 2) },
                90.0);

            HaulRouteBanding.Result Banded = HaulRouteBanding.Apply(
                new List<Pass> { Gapped, AdjacentLanes, Diagonal, Repeated },
                new HashSet<(Int32 Bx, Int32 By)>(),
                new HashSet<(Int32 Bx, Int32 By)>());

            Assert.Equal(2, Banded.FullStripCount);
            Assert.Equal(2, Banded.OtherInteriorCount);
            Assert.Equal(0, Banded.EdgeCount);
            Assert.Equal(0, Banded.ScrapCount);
            Assert.Contains(Diagonal, Banded.Route.OrderedPasses.Take(2));
            Assert.Contains(Repeated, Banded.Route.OrderedPasses.Take(2));
            Assert.Contains(Gapped, Banded.Route.OrderedPasses.Skip(2));
            Assert.Contains(AdjacentLanes, Banded.Route.OrderedPasses.Skip(2));
        }

        /// <summary>Builds one east-west or arbitrary-heading load.</summary>
        /// <param name="Cut">Cut cell columns and rows.</param>
        /// <param name="Fill">Fill cell columns and rows.</param>
        /// <param name="HeadingDeg">Lane axis in degrees.</param>
        /// <returns>The load.</returns>
        private static Pass Strip
            (
            (Int32 Bx, Int32 By)[] Cut,
            (Int32 Bx, Int32 By)[] Fill,
            Double HeadingDeg
            )
        {
            Pass Pass = new Pass(
                Cells(Cut),
                Cells(Fill),
                1.0,
                1.0,
                (0.0, 0.0),
                (1.0, 0.0),
                (0.0, 10.0),
                (1.0, 10.0));
            Pass.CutHeadingDeg = HeadingDeg;
            Pass.FillHeadingDeg = HeadingDeg;
            return Pass;
        }

        /// <summary>Moves a load so travel order would want a different sequence.</summary>
        /// <param name="Pass">Load to place.</param>
        /// <param name="CutStart">Cut start coordinate.</param>
        /// <returns>The same load.</returns>
        private static Pass At
            (
            Pass Pass,
            (Double X, Double Y) CutStart
            )
        {
            return new Pass(
                Pass.CutCells,
                Pass.FillCells,
                Pass.LoadBcy,
                Pass.HaulDistanceM,
                CutStart,
                (CutStart.X + 1.0, CutStart.Y),
                (CutStart.X, CutStart.Y + 4.0),
                (CutStart.X + 1.0, CutStart.Y + 4.0))
            {
                CutHeadingDeg = Pass.CutHeadingDeg,
                FillHeadingDeg = Pass.FillHeadingDeg,
            };
        }

        /// <summary>Builds cell records.</summary>
        /// <param name="Coordinates">Cell columns and rows.</param>
        /// <returns>Cell and depth records.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            (
            (Int32 Bx, Int32 By)[] Coordinates
            )
        {
            return Coordinates
                .Select(Cell => (Cell, 0.1))
                .ToList();
        }

        /// <summary>Collects cell coordinates.</summary>
        /// <param name="Cells">Cell records.</param>
        /// <returns>The coordinates.</returns>
        private static HashSet<(Int32 Bx, Int32 By)> EdgeCells
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            return Cells.Select(Entry => Entry.Cell).ToHashSet();
        }
    }
}
