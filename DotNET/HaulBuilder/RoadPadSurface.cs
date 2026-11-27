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
            UnusedDepthM = new Double[Height, Width];
            PanGapM = new Double[Height, Width];
            ReserveLanes = new List<ReserveLane>();
            SurplusM = new Double[Height, Width];
            DeficitM = new Double[Height, Width];
            OpenEnd = new Boolean[Height, Width];
            Excluded = new Boolean[Height, Width];
            BladeCuts = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            BladeFills = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
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
        /// Gets leftover pan surplus the road stage may still cut, in metres.
        /// The main stages take the surplus first, so this stays zero until those passes are accounted.
        /// </summary>
        internal Double[,] StageCutM { get; }

        /// <summary>
        /// Gets the bank fill the road stage must borrow.
        /// It is the pan deficit times fill-to-bank, not the finished crown volume.
        /// </summary>
        public Double PanImportBcy { get; internal set; }

        /// <summary>
        /// Gets how far each pan cell sits above its pan target, in metres, before haul planning.
        /// Zero off the pan and where the ground is not high.
        /// </summary>
        internal Double[,] SurplusM { get; }

        /// <summary>
        /// Gets how far each pan cell sits below its pan target, in metres, before haul planning.
        /// Zero off the pan and where the ground is not low.
        /// </summary>
        internal Double[,] DeficitM { get; }

        /// <summary>
        /// Gets cells at an open end. The finish check does not require them to meet the pan.
        /// </summary>
        internal Boolean[,] OpenEnd { get; }

        /// <summary>Gets pan surplus before haul planning, in bank cubic yards.</summary>
        public Double RoadSurplusBcy { get; internal set; }

        /// <summary>Gets pan deficit before haul planning, in bank cubic yards.</summary>
        public Double RoadDeficitBcy { get; internal set; }

        /// <summary>Gets pan surplus the main stages left, in bank cubic yards.</summary>
        public Double SurplusRemainingBcy { get; internal set; }

        /// <summary>Gets pan deficit still to fill when the road stage starts, in bank cubic yards.</summary>
        public Double DeficitRemainingBcy { get; internal set; }

        /// <summary>Gets reserved field cut still in the ground when the road stage starts, in bank cubic yards.</summary>
        public Double ReserveAvailableBcy { get; internal set; }

        /// <summary>Gets reserved field cut the road stage took, in bank cubic yards.</summary>
        public Double ReserveUsedBcy { get; internal set; }

        /// <summary>
        /// Gets reserved field cut still in the ground after correcting, in bank cubic yards.
        /// Correcting cuts whatever the road stage left. This should be zero.
        /// </summary>
        public Double UnusedReserveBcy { get; internal set; }

        /// <summary>Gets how many field cells finished more than the tolerance from the design.</summary>
        public Int32 FieldCellsOffDesign { get; internal set; }

        /// <summary>
        /// Gets the road-stage passes built after flow and before correcting.
        /// Null until that stage runs.
        /// </summary>
        internal List<Pass>? StagePasses { get; set; }

        /// <summary>Gets cut the blade takes for the road stage. Filled when the passes are built.</summary>
        internal List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BladeCuts { get; }

        /// <summary>Gets fill the blade places for the road stage. Filled when the passes are built.</summary>
        internal List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BladeFills { get; }

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

        /// <summary>
        /// Gets reserved depth the road stage did not cut, in metres.
        /// Correcting cuts this and fills the pan gap.
        /// </summary>
        internal Double[,] UnusedDepthM { get; }

        /// <summary>
        /// Gets the pan gap the road stage left, in metres.
        /// Correcting fills it from the unused reserve.
        /// </summary>
        internal Double[,] PanGapM { get; }

        /// <summary>
        /// Gets the blade-width strips the reserve was taken from.
        /// Empty when the pan needs no borrow.
        /// </summary>
        internal List<ReserveLane> ReserveLanes { get; }

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

        /// <summary>Gets bank cubic yards the blade places: light pan drafts, and the well casing the pan does not enter.</summary>
        public Double BladeMoveBcy { get; internal set; }

        /// <summary>Gets how many road-stage trips spread on a pad.</summary>
        public Int32 PadTripCount { get; internal set; }

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
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Road surplus {0:F3} BCY",
                RoadSurplusBcy));
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "Road deficit {0:F3} BCY",
                RoadDeficitBcy));
            Text.AppendLine(RoadPadBuild.FormatBalanceLine(this));
            Text.AppendLine(RoadPadBuild.FormatReserveLine(this));
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

        /// <summary>
        /// One blade-width strip of field cut held for the road.
        /// The strip is at least four turning radii long and runs with the local cut.
        /// </summary>
        internal sealed class ReserveLane
        {
            /// <summary>
            /// Initializes one strip.
            /// </summary>
            /// <param name="DirX">Cut direction east.</param>
            /// <param name="DirY">Cut direction north.</param>
            /// <param name="LengthM">Strip length in metres, including one cell.</param>
            /// <param name="DistanceM">Distance from the strip to the nearest pan deficit, in metres.</param>
            /// <param name="Cells">Cells, nearest the deficit first.</param>
            public ReserveLane
                (
                Double DirX,
                Double DirY,
                Double LengthM,
                Double DistanceM,
                List<ReserveCell> Cells
                )
            {
                this.DirX = DirX;
                this.DirY = DirY;
                this.LengthM = LengthM;
                this.DistanceM = DistanceM;
                this.Cells = Cells;
            }

            /// <summary>Gets the cut direction east.</summary>
            public Double DirX { get; }

            /// <summary>Gets the cut direction north.</summary>
            public Double DirY { get; }

            /// <summary>Gets the strip length in metres, including one cell.</summary>
            public Double LengthM { get; }

            /// <summary>Gets the distance from the strip to the nearest pan deficit, in metres.</summary>
            public Double DistanceM { get; }

            /// <summary>Gets the cells, nearest the deficit first.</summary>
            public List<ReserveCell> Cells { get; }
        }

        /// <summary>One cell of a reserve strip. The depth still to cut changes as the road stage takes it.</summary>
        internal sealed class ReserveCell
        {
            /// <summary>
            /// Initializes one cell.
            /// </summary>
            /// <param name="Bx">Column.</param>
            /// <param name="By">Row.</param>
            /// <param name="DistanceM">Distance to the nearest pan deficit, in metres.</param>
            public ReserveCell
                (
                Int32 Bx,
                Int32 By,
                Double DistanceM
                )
            {
                this.Bx = Bx;
                this.By = By;
                this.DistanceM = DistanceM;
            }

            /// <summary>Gets the column.</summary>
            public Int32 Bx { get; }

            /// <summary>Gets the row.</summary>
            public Int32 By { get; }

            /// <summary>Gets the distance to the nearest pan deficit, in metres.</summary>
            public Double DistanceM { get; }

            /// <summary>Gets or sets the reserved depth still to cut, in metres.</summary>
            public Double LeftM { get; set; }
        }
    }
}
