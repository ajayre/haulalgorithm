using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies that depth which guidance neither drives nor defers is reported
    /// as deferred work, so the deferred totals of a run cover every cell.
    /// </summary>
    public sealed class GuidanceCoverageOmittedTests
    {
        /// <summary>Rows and columns of the minimal field lattice.</summary>
        private const Int32 LATTICE_SIZE = 24;

        /// <summary>First column that needs fill; the columns before it need cut.</summary>
        private const Int32 FILL_COLUMN = 12;

        /// <summary>Latitude of the first row, in degrees.</summary>
        private const Double LATITUDE_DEG = 40.0;

        /// <summary>Longitude of the first column, in degrees.</summary>
        private const Double LONGITUDE_DEG = -93.0;

        /// <summary>Spacing of the survey points, in degrees.</summary>
        private const Double STEP_DEG = 0.00003;

        /// <summary>Design elevation, in metres.</summary>
        private const Double DESIGN_M = 100.0;

        /// <summary>Height of the existing ground above or below the design, in metres.</summary>
        private const Double OFFSET_M = 0.1;

        /// <summary>Lane axis for east-west lanes, which keeps a run on this field to seconds.</summary>
        private const Double LANE_AXIS_EAST_WEST_DEG = 90.0;

        /// <summary>Start of the message that totals the unrecorded depth of one guidance build.</summary>
        private const String UNRECORDED_MESSAGE_PREFIX = "whole_plan unrecorded ";

        /// <summary>Message field that holds the unrecorded cut volume.</summary>
        private const String CUT_FIELD_PREFIX = "cut_bcy=";

        /// <summary>Message field that holds the unrecorded fill volume.</summary>
        private const String FILL_FIELD_PREFIX = "fill_bcy=";

        /// <summary>
        /// Rounding of one message total, which is written with three decimals, in BCY.
        /// </summary>
        private const Double MESSAGE_ROUNDING_BCY = 0.001;

        /// <summary>Largest difference between two volume totals that count as equal, in BCY.</summary>
        private const Double TOTAL_TOLERANCE_BCY = 1e-6;

        /// <summary>
        /// Verifies a run lists the depth that guidance left out under its own
        /// reason, that the listed volume equals the volume the guidance builds
        /// report, and that the deferred totals include it.
        /// </summary>
        [Fact]
        public void Run_ReportsDepthThatGuidanceLeavesOut
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Options = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    LaneAxisHeadingDeg = LANE_AXIS_EAST_WEST_DEG,
                };
                HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);
                Assert.NotNull(Result.HaulOptimization);
                Assert.NotNull(Result.DeferredWorkSummary);
                IReadOnlyList<HaulDeferredWork> Deferred = Result.HaulOptimization.DeferredWork;

                Double OmittedCutBcy = SumVolume(
                    Deferred,
                    HaulLaneBuilder.WorkKind.Cut,
                    HaulDeferredWorkReasons.GUIDANCE_COVERAGE_OMITTED);
                Double OmittedFillBcy = SumVolume(
                    Deferred,
                    HaulLaneBuilder.WorkKind.Fill,
                    HaulDeferredWorkReasons.GUIDANCE_COVERAGE_OMITTED);
                (Double CutBcy, Double FillBcy, Int32 MessageCount) Reported =
                    SumUnrecordedMessages(Result.Messages);

                // This field leaves cells out on both sides, so both totals are positive.
                Assert.True(OmittedCutBcy > 0.0, "No cut depth was reported as left out.");
                Assert.True(OmittedFillBcy > 0.0, "No fill depth was reported as left out.");
                Assert.True(Reported.MessageCount > 0, "No guidance build reported its unrecorded depth.");
                // Each message rounds its total to three decimals.
                Double MessageToleranceBcy = MESSAGE_ROUNDING_BCY * Reported.MessageCount;
                Assert.InRange(Math.Abs(OmittedCutBcy - Reported.CutBcy), 0.0, MessageToleranceBcy);
                Assert.InRange(Math.Abs(OmittedFillBcy - Reported.FillBcy), 0.0, MessageToleranceBcy);

                // The run totals are the sum of every listed entry, the new ones included.
                Assert.InRange(
                    Math.Abs(Result.DeferredWorkSummary.DeferredCutBcy
                        - SumVolume(Deferred, HaulLaneBuilder.WorkKind.Cut, null)),
                    0.0,
                    TOTAL_TOLERANCE_BCY);
                Assert.InRange(
                    Math.Abs(Result.DeferredWorkSummary.DeferredFillBcy
                        - SumVolume(Deferred, HaulLaneBuilder.WorkKind.Fill, null)),
                    0.0,
                    TOTAL_TOLERANCE_BCY);
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>Sums the bank volume of the deferred entries of one kind.</summary>
        /// <param name="Deferred">Deferred entries of a run.</param>
        /// <param name="Kind">Cut or fill.</param>
        /// <param name="Reason">Reason to keep, or null to keep every reason.</param>
        /// <returns>Total bank volume in BCY.</returns>
        static private Double SumVolume
            (
            IReadOnlyList<HaulDeferredWork> Deferred,
            HaulLaneBuilder.WorkKind Kind,
            String? Reason
            )
        {
            Double Total = 0.0;
            foreach (HaulDeferredWork Entry in Deferred)
            {
                if ((Entry.Kind == Kind) && ((Reason == null) || (Entry.Reason == Reason)))
                {
                    Total += Entry.VolumeBcy;
                }
            }
            return Total;
        }

        /// <summary>
        /// Adds up the unrecorded cut and fill volumes that the guidance builds
        /// of one run wrote into the messages.
        /// </summary>
        /// <param name="Messages">Messages of a run.</param>
        /// <returns>Total cut BCY, total fill BCY, and the number of messages read.</returns>
        static private (Double CutBcy, Double FillBcy, Int32 MessageCount) SumUnrecordedMessages
            (
            IReadOnlyList<String> Messages
            )
        {
            Double CutBcy = 0.0;
            Double FillBcy = 0.0;
            Int32 MessageCount = 0;
            foreach (String Message in Messages)
            {
                if (!Message.StartsWith(UNRECORDED_MESSAGE_PREFIX, StringComparison.Ordinal))
                {
                    continue;
                }
                MessageCount += 1;
                // Fields are separated by spaces; the twice-handled fields have
                // another prefix, so they are not read here.
                foreach (String Field in Message.Split(' '))
                {
                    if (Field.StartsWith(CUT_FIELD_PREFIX, StringComparison.Ordinal))
                    {
                        CutBcy += Double.Parse(
                            Field.Substring(CUT_FIELD_PREFIX.Length),
                            CultureInfo.InvariantCulture);
                    }
                    else if (Field.StartsWith(FILL_FIELD_PREFIX, StringComparison.Ordinal))
                    {
                        FillBcy += Double.Parse(
                            Field.Substring(FILL_FIELD_PREFIX.Length),
                            CultureInfo.InvariantCulture);
                    }
                }
            }
            return (CutBcy, FillBcy, MessageCount);
        }

        /// <summary>
        /// Writes a tiny AGD whose west half needs cut and whose east half needs fill.
        /// </summary>
        /// <returns>Path to a temporary AGD file.</returns>
        static private String CreateMinimalAgd
            (
            )
        {
            String PathValue = Path.Combine(
                Path.GetTempPath(),
                "haulbuilder_coverageomitted_"
                    + Guid.NewGuid().ToString("N")
                    + ".agd");
            List<String> Lines = new List<String>
            {
                "Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments",
                "40.0, -93.0, 100.0,,,0MB,",
            };
            for (Int32 Row = 0; Row < LATTICE_SIZE; Row += 1)
            {
                for (Int32 Column = 0; Column < LATTICE_SIZE; Column += 1)
                {
                    Double Existing = DESIGN_M + ((Column < FILL_COLUMN) ? OFFSET_M : -OFFSET_M);
                    Lines.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "{0:F8}, {1:F8}, {2:F3}, {3:F3}, {4:F3}, 3GRD, ",
                        LATITUDE_DEG + (Row * STEP_DEG),
                        LONGITUDE_DEG + (Column * STEP_DEG),
                        Existing,
                        DESIGN_M,
                        DESIGN_M - Existing));
                }
            }

            File.WriteAllText(PathValue, String.Join("\n", Lines) + "\n");
            return PathValue;
        }
    }
}
