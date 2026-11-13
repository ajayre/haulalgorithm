using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

using HaulBuilder;

namespace FieldSolver.Tests
{
    /// <summary>
    /// Shared test data: the eight measured TheShop2 runs as a table, and small
    /// builders for settings, rows, requests, and a tiny field file.
    /// </summary>
    static internal class SolverFixtures
    {
        /// <summary>Required fill of TheShop2 in BCY, from the reference build.</summary>
        public const Double SHOP_REQUIRED_FILL_BCY = 46643.752;

        /// <summary>Required cut of TheShop2 in BCY, from the reference build.</summary>
        public const Double SHOP_REQUIRED_CUT_BCY = 46763.981;

        /// <summary>Edge cap the engine ships with.</summary>
        public const Int32 EDGE_DEFAULT = 12;

        /// <summary>Edge cap that the standard search adds.</summary>
        public const Int32 EDGE_LOOSE = 40;

        /// <summary>Minimum work length value for the engine default.</summary>
        public const Double WORK_DEFAULT = FieldSolverSettings.MINIMUM_WORK_LENGTH_DEFAULT_FT;

        /// <summary>Minimum work length value for "rule off".</summary>
        public const Double WORK_OFF = FieldSolverSettings.MINIMUM_WORK_LENGTH_OFF_FT;

        /// <summary>Lane axis for north-south lanes.</summary>
        public const Double AXIS_NORTH_SOUTH = 0.0;

        /// <summary>Lane axis for east-west lanes.</summary>
        public const Double AXIS_EAST_WEST = 90.0;

        /// <summary>Number of runs in the standard search on engine defaults.</summary>
        public const Int32 STANDARD_RUNS = 8;

        /// <summary>Rows and columns of the tiny field lattice.</summary>
        private const Int32 TINY_SIZE = 24;

        /// <summary>First column of the tiny field that needs fill; the columns before it need cut.</summary>
        private const Int32 TINY_FILL_COLUMN = 12;

        /// <summary>Latitude of the tiny field's first row, in degrees.</summary>
        private const Double TINY_LATITUDE_DEG = 40.0;

        /// <summary>Longitude of the tiny field's first column, in degrees.</summary>
        private const Double TINY_LONGITUDE_DEG = -93.0;

        /// <summary>Spacing of the tiny field's survey points, in degrees.</summary>
        private const Double TINY_STEP_DEG = 0.00003;

        /// <summary>Design elevation of the tiny field, in metres.</summary>
        private const Double TINY_DESIGN_M = 100.0;

        /// <summary>Height of the existing ground above or below the design, in metres.</summary>
        private const Double TINY_OFFSET_M = 0.1;

        /// <summary>Output image count the default request carries, to prove it comes back unchanged.</summary>
        public const Int32 CALLER_IMAGE_COUNT = 7;

        /// <summary>Builds engine-default settings with the three combined values replaced.</summary>
        /// <param name="Edge">Edge cap.</param>
        /// <param name="WorkFt">Minimum work length in feet.</param>
        /// <param name="AxisDeg">Lane axis in degrees.</param>
        /// <returns>Engine defaults with the three values applied.</returns>
        static public FieldSolverSettings Primary
            (
            Int32 Edge,
            Double WorkFt,
            Double AxisDeg
            )
        {
            return FieldSolverSettings.FromOptions(new HaulBuilderOptions()) with
            {
                EdgeMaxLaneSlicesPerCycle = Edge,
                MinimumWorkLengthFt = WorkFt,
                LaneAxisHeadingDeg = AxisDeg,
            };
        }

        /// <summary>Builds one row with the required volumes of TheShop2.</summary>
        /// <param name="DistanceM">Whole-job driven metres.</param>
        /// <param name="Hours">Deterministic hours.</param>
        /// <param name="DeferredCut">Deferred cut in BCY.</param>
        /// <param name="DeferredFill">Deferred fill in BCY.</param>
        /// <param name="Assists">Manual assists.</param>
        /// <returns>The row.</returns>
        static public CandidateMeasurement ShopRow
            (
            Double DistanceM,
            Double Hours,
            Double DeferredCut,
            Double DeferredFill,
            Int32 Assists
            )
        {
            return new CandidateMeasurement
            {
                DistanceM = DistanceM,
                DeterministicHours = Hours,
                DeferredCutBcy = DeferredCut,
                DeferredFillBcy = DeferredFill,
                ManualAssistCount = Assists,
                RequiredFillBcy = SHOP_REQUIRED_FILL_BCY,
                RequiredCutBcy = SHOP_REQUIRED_CUT_BCY,
            };
        }

        /// <summary>Builds the table of the eight measured TheShop2 runs.</summary>
        /// <returns>A runner that answers from the measured table.</returns>
        static public TableRunner ShopTable
            (
            )
        {
            TableRunner Table = new TableRunner();
            // Re-measured 12 November 2026 after Task L step 1. Run order is the search order.
            Table.Add(Primary(EDGE_DEFAULT, WORK_DEFAULT, AXIS_NORTH_SOUTH), ShopRow(2069949.3, 221.682, 462.145, 286.488, 100));
            Table.Add(Primary(EDGE_LOOSE, WORK_DEFAULT, AXIS_NORTH_SOUTH), ShopRow(2029277.8, 218.017, 469.054, 266.827, 65));
            Table.Add(Primary(EDGE_DEFAULT, WORK_OFF, AXIS_NORTH_SOUTH), ShopRow(2069401.8, 221.804, 450.148, 259.131, 155));
            Table.Add(Primary(EDGE_LOOSE, WORK_OFF, AXIS_NORTH_SOUTH), ShopRow(2047112.9, 220.003, 442.474, 252.318, 126));
            Table.Add(Primary(EDGE_DEFAULT, WORK_DEFAULT, AXIS_EAST_WEST), ShopRow(2043719.6, 219.371, 455.468, 294.117, 113));
            Table.Add(Primary(EDGE_LOOSE, WORK_DEFAULT, AXIS_EAST_WEST), ShopRow(1994349.1, 215.322, 543.328, 303.351, 73));
            Table.Add(Primary(EDGE_DEFAULT, WORK_OFF, AXIS_EAST_WEST), ShopRow(2052111.3, 220.859, 445.836, 279.986, 224));
            Table.Add(Primary(EDGE_LOOSE, WORK_OFF, AXIS_EAST_WEST), ShopRow(2001217.3, 216.598, 540.069, 319.132, 141));
            return Table;
        }

        /// <summary>
        /// Builds a request on engine defaults with two of the caller's own output
        /// switches set. The field path is never opened by a table runner.
        /// </summary>
        /// <returns>A request with the standard search.</returns>
        static public FieldSolverRequest DefaultRequest
            (
            )
        {
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = "table-only.agd",
                // The caller's own outputs must come back in BestOptions untouched.
                IncludeGuidancePng = true,
                RouteGuidanceImageCount = CALLER_IMAGE_COUNT,
            };
            return new FieldSolverRequest(Options);
        }

        /// <summary>
        /// Writes a tiny field file: a 24 by 24 lattice whose west half needs cut and
        /// whose east half needs fill. It is the same field as the minimal fixture of
        /// HaulBuilder.Tests, and one engine run on it takes seconds.
        /// </summary>
        /// <returns>Path of a temporary AGD file. The caller deletes it.</returns>
        static public String CreateTinyAgd
            (
            )
        {
            String PathValue = Path.Combine(
                Path.GetTempPath(),
                "fieldsolver_tiny_" + Guid.NewGuid().ToString("N") + ".agd");
            List<String> Lines = new List<String>
            {
                "Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments",
                "40.0, -93.0, 100.0,,,0MB,",
            };
            for (Int32 Row = 0; Row < TINY_SIZE; Row += 1)
            {
                for (Int32 Column = 0; Column < TINY_SIZE; Column += 1)
                {
                    Double Latitude = TINY_LATITUDE_DEG + (Row * TINY_STEP_DEG);
                    Double Longitude = TINY_LONGITUDE_DEG + (Column * TINY_STEP_DEG);
                    Double Existing = TINY_DESIGN_M + ((Column < TINY_FILL_COLUMN) ? TINY_OFFSET_M : -TINY_OFFSET_M);
                    Lines.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "{0:F8}, {1:F8}, {2:F3}, {3:F3}, {4:F3}, 3GRD, ",
                        Latitude,
                        Longitude,
                        Existing,
                        TINY_DESIGN_M,
                        TINY_DESIGN_M - Existing));
                }
            }
            File.WriteAllText(PathValue, String.Join("\n", Lines) + "\n");
            return PathValue;
        }
    }
}
