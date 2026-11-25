using System.Globalization;
using System.Text;

namespace HaulBuilder
{
    /// <summary>
    /// Roads and well pads built to a fixed height above the fitted field design.
    /// The relative height does not depend on the field design, so the fill volume
    /// is known before the field is shifted.
    /// </summary>
    public sealed class RoadPadSurface
    {
        /// <summary>
        /// Initializes an empty surface. The builder fills the grids.
        /// </summary>
        /// <param name="Width">Grid width in cells.</param>
        /// <param name="Height">Grid height in cells.</param>
        internal RoadPadSurface
            (
            Int32 Width,
            Int32 Height
            )
        {
            RelM = new Double[Height, Width];
            PanM = new Double[Height, Width];
            PanCoverage = new Double[Height, Width];
            StageCutM = new Double[Height, Width];
            Weight = new Double[Height, Width];
            FieldDesignM = new Double[Height, Width];
            FinalDesignM = new Double[Height, Width];
            ReservedDepthM = new Double[Height, Width];
            Excluded = new Boolean[Height, Width];
            OwnerIsPad = new Boolean[Height, Width];
            OwnerIndex = new Int32[Height, Width];
            Warnings = new List<String>();
            Roads = new List<RoadReport>();
            Pads = new List<PadReport>();
            KeptRoads = new List<RoadCentreline>();
            KeptPads = new List<WellPad>();
            RoadCourses = new List<TripGuidance.TripCourse>();
            RoadTopOk = true;
            RoadTopFailure = String.Empty;
            for (Int32 Row = 0; Row < Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Width; Col += 1)
                {
                    // No owner until a road or pad sets the cell.
                    OwnerIndex[Row, Col] = -1;
                }
            }
        }

        /// <summary>Gets the finished height above the field design, in metres. Zero off the works.</summary>
        public Double[,] RelM { get; }

        /// <summary>
        /// Gets the flat pan height above the field design, in metres.
        /// A full cell is the build height. A partial cell is that height times its coverage.
        /// </summary>
        public Double[,] PanM { get; }

        /// <summary>
        /// Gets the fraction of each cell that lies inside the pan, from 0 to 1.
        /// Snapped axis-aligned cells are 0 or 1. Diagonal cells may be in between.
        /// </summary>
        public Double[,] PanCoverage { get; }

        /// <summary>
        /// Gets trim the road stage cuts from a pan cell, in metres.
        /// The rest of any trim stays with the field stages.
        /// </summary>
        internal Double[,] StageCutM { get; }

        /// <summary>Gets the bank fill the road stage must still borrow from field cells.</summary>
        public Double PanImportBcy { get; internal set; }

        /// <summary>Gets road-stage cut taken from pan cells, in BCY.</summary>
        public Double RoadCellCutBcy { get; internal set; }

        /// <summary>Gets road-stage cut taken from reserved field cells, in BCY.</summary>
        public Double FieldCutBcy { get; internal set; }

        /// <summary>Gets the fraction of each cell that lies inside the field.</summary>
        public Double[,] Weight { get; }

        /// <summary>Gets the fitted field design, in metres, before roads and pads are added.</summary>
        public Double[,] FieldDesignM { get; }

        /// <summary>Gets the final design, field design plus <see cref="RelM"/>.</summary>
        public Double[,] FinalDesignM { get; }

        /// <summary>Gets cut depth held back from the field stages, in metres.</summary>
        public Double[,] ReservedDepthM { get; }

        /// <summary>Gets cells inside a well-casing exclusion. The target is still set.</summary>
        public Boolean[,] Excluded { get; }

        /// <summary>Gets whether the owning feature is a pad. Meaningful when <see cref="OwnerIndex"/> is non-negative.</summary>
        internal Boolean[,] OwnerIsPad { get; }

        /// <summary>Gets the owning road or pad index, or -1 when the cell is not raised.</summary>
        internal Int32[,] OwnerIndex { get; }

        /// <summary>Gets warnings. They do not fail the run.</summary>
        public List<String> Warnings { get; }

        /// <summary>Gets a value indicating whether every road-top sample matches the crown formula.</summary>
        public Boolean RoadTopOk { get; internal set; }

        /// <summary>Gets the failure text when <see cref="RoadTopOk"/> is false.</summary>
        public String RoadTopFailure { get; internal set; }

        /// <summary>Gets the bank fill of every road and pad, in BCY.</summary>
        public Double RoadFillBcy { get; internal set; }

        /// <summary>Gets the uniform shift added to the incoming design, in metres. Negative lowers it.</summary>
        public Double FieldShiftM { get; internal set; }

        /// <summary>Gets the uniform shift in feet.</summary>
        public Double FieldShiftFt
        {
            get
            {
                return FieldShiftM / Constants.FEET_TO_METERS;
            }
        }

        /// <summary>Gets the summed centreline length in feet.</summary>
        public Double CentrelineLengthFt { get; internal set; }

        /// <summary>Gets blade-shaping hours for slopes, crown, and exclusion zones.</summary>
        public Double BladeShapingHours { get; internal set; }

        /// <summary>Gets estimated road-stage hours, field efficiency included.</summary>
        public Double RoadStageHours { get; internal set; }

        /// <summary>
        /// Gets the guided course of each road-stage trip, in planner order.
        /// Empty until the road stage has been built.
        /// </summary>
        public IReadOnlyList<TripGuidance.TripCourse> RoadCourses { get; internal set; }

        /// <summary>Gets the number of road-stage trips.</summary>
        public Int32 RoadTripCount { get; internal set; }

        /// <summary>Gets the mean trip load as a fraction of the bowl.</summary>
        public Double MeanLoadFraction { get; internal set; }

        /// <summary>Gets the mean loaded haul, in feet.</summary>
        public Double MeanLoadedCarryFt { get; internal set; }

        /// <summary>Gets road-stage hours per bank cubic yard.</summary>
        public Double RoadHoursPerBcy { get; internal set; }

        /// <summary>Gets bank cubic yards the pans haul on the road stage.</summary>
        public Double PanHaulBcy { get; internal set; }

        /// <summary>Gets bank cubic yards handed to the blade because the pan load would have been too light.</summary>
        public Double BladeMoveBcy { get; internal set; }

        /// <summary>Gets field-pass hours per bank cubic yard. Null until the field plan is measured.</summary>
        public Double? FieldHoursPerBcy { get; internal set; }

        /// <summary>Gets one row per kept road.</summary>
        public List<RoadReport> Roads { get; }

        /// <summary>Gets one row per pad.</summary>
        public List<PadReport> Pads { get; }

        /// <summary>Gets roads that were not dropped as duplicates.</summary>
        internal List<RoadCentreline> KeptRoads { get; }

        /// <summary>Gets the pads supplied for this run.</summary>
        internal List<WellPad> KeptPads { get; }

        /// <summary>
        /// Writes the road and pad report. Feature fill sums to <see cref="RoadFillBcy"/>.
        /// </summary>
        /// <returns>The report text.</returns>
        public String FormatReport
            (
            )
        {
            StringBuilder Text = new StringBuilder();
            Text.AppendLine("Roads and well pads");
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Road fill {0:F3} BCY",
                RoadFillBcy));
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Field design shift {0:F3} ft",
                FieldShiftFt));
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Road stage {0:F3} h",
                RoadStageHours));
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Blade shaping {0:F1} ft, {1:F3} h",
                CentrelineLengthFt,
                BladeShapingHours));
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Cut from road cells {0:F3} BCY",
                RoadCellCutBcy));
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Cut from the field {0:F3} BCY",
                FieldCutBcy));
            String Stage = FormatStage();
            if (Stage.Length > 0)
            {
                Text.AppendLine(Stage);
            }

            Text.AppendLine();
            Double FeatureSum = 0.0;
            for (Int32 Index = 0; Index < Roads.Count; Index += 1)
            {
                RoadReport Road = Roads[Index];
                FeatureSum += Road.BankBcy;
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Road {0}",
                    Index + 1));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Length {0:F1} ft",
                    Road.LengthFt));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Top width {0:F1} ft",
                    Road.TopWidthFt));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Lanes {0}",
                    Road.LaneCount));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Pan width {0:F1} ft",
                    Road.PanWidthFt));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Pan height {0:F3} ft",
                    Road.BuildHeightFt));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Pan shift {0:F2} ft",
                    Math.Abs(Road.PanShiftFt)));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Partial pan {0:F1}% of pan volume",
                    Road.PartialPanFraction * 100.0));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  NRCS minimum {0:F1} ft",
                    Road.NrcsMinRoadbedFt));
                if (Road.HasRoadbedOverride)
                {
                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "  Roadbed override {0:F1} ft",
                        Road.RoadbedOverrideFt));
                }

                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Shoulder {0:F2} ft",
                    Road.HeightFt));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Centreline {0:F3} ft",
                    Road.CentrelineFt));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Fill {0:F3} BCY",
                    Road.BankBcy));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Shared length {0:F1} ft",
                    Road.SharedLengthFt));
                foreach (String OpenEnd in Road.OpenEndLines)
                {
                    Text.AppendLine("  " + OpenEnd);
                }
            }

            for (Int32 Index = 0; Index < Pads.Count; Index += 1)
            {
                PadReport Pad = Pads[Index];
                FeatureSum += Pad.BankBcy;
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Pad {0}",
                    Index + 1));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Top radius {0:F1} ft",
                    Pad.TopRadiusFt));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Fill {0:F3} BCY",
                    Pad.BankBcy));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  Excluded {0:F3} BCY",
                    Pad.ExcludedBcy));
            }

            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Feature fill {0:F3} BCY",
                FeatureSum));
            return Text.ToString();
        }

        /// <summary>
        /// One line of road-stage diagnostics: hauls, bowl load, pan and blade volume, and hours.
        /// </summary>
        /// <returns>The line, or empty when the road stage moved no soil.</returns>
        public String FormatStage
            (
            )
        {
            if ((RoadTripCount <= 0) && (BladeMoveBcy <= 1e-6))
            {
                return String.Empty;
            }

            String Hours = (RoadFillBcy > 1e-6)
                ? String.Format(CultureInfo.InvariantCulture, "{0:F4} h/BCY", RoadHoursPerBcy)
                : "no fill";
            String Versus = FieldHoursPerBcy.HasValue
                ? String.Format(CultureInfo.InvariantCulture, " vs field {0:F4} h/BCY", FieldHoursPerBcy.Value)
                : String.Empty;
            return String.Format(
                CultureInfo.InvariantCulture,
                "Road stage: {0} hauls, mean load {1:F1}% of bowl, pan {2:F3} BCY, blade {3:F3} BCY, {4:F3} h, mean loaded carry {5:F0} ft, {6}{7}.",
                RoadTripCount,
                MeanLoadFraction * 100.0,
                PanHaulBcy,
                BladeMoveBcy,
                RoadStageHours,
                MeanLoadedCarryFt,
                Hours,
                Versus);
        }

        /// <summary>One road row in the report.</summary>
        public sealed class RoadReport
        {
            /// <summary>Gets the centreline length in feet.</summary>
            public Double LengthFt { get; internal set; }

            /// <summary>Gets the finished top width in feet.</summary>
            public Double TopWidthFt { get; internal set; }

            /// <summary>Gets the number of pan lanes.</summary>
            public Int32 LaneCount { get; internal set; }

            /// <summary>Gets the pan width in feet. One lane is one blade.</summary>
            public Double PanWidthFt { get; internal set; }

            /// <summary>Gets the flat pan height in feet. It carries the same volume per foot as the finished section.</summary>
            public Double BuildHeightFt { get; internal set; }

            /// <summary>Gets the lateral pan snap in feet. Positive is east or north. At most half a cell.</summary>
            public Double PanShiftFt { get; internal set; }

            /// <summary>Gets the fraction of this road's pan volume that sits in cells which are not fully covered.</summary>
            public Double PartialPanFraction { get; internal set; }

            /// <summary>Gets the NRCS minimum roadbed, in feet. A local override is reported separately.</summary>
            public Double NrcsMinRoadbedFt { get; internal set; }

            /// <summary>Gets a value indicating whether this road overrides the roadbed.</summary>
            public Boolean HasRoadbedOverride { get; internal set; }

            /// <summary>Gets the local roadbed override in feet. Zero when there is no override.</summary>
            public Double RoadbedOverrideFt { get; internal set; }

            /// <summary>Gets the shoulder height above the field design, in feet.</summary>
            public Double HeightFt { get; internal set; }

            /// <summary>Gets the centreline height above the field design, in feet.</summary>
            public Double CentrelineFt { get; internal set; }

            /// <summary>Gets fill attributed to this road, in BCY.</summary>
            public Double BankBcy { get; internal set; }

            /// <summary>Gets centreline length that lies on another road's top, in feet.</summary>
            public Double SharedLengthFt { get; internal set; }

            /// <summary>Gets open-end notes for this road. Each names the end and the BCY not built.</summary>
            public List<String> OpenEndLines { get; } = new List<String>();
        }

        /// <summary>One pad row in the report.</summary>
        public sealed class PadReport
        {
            /// <summary>Gets the flat-top radius in feet.</summary>
            public Double TopRadiusFt { get; internal set; }

            /// <summary>Gets fill attributed to this pad, including the exclusion, in BCY.</summary>
            public Double BankBcy { get; internal set; }

            /// <summary>Gets the excluded part of <see cref="BankBcy"/>, in BCY.</summary>
            public Double ExcludedBcy { get; internal set; }
        }
    }
}
