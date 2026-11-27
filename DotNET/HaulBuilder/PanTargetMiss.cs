using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// Road and pad cells that finished outside the finish tolerance.
    /// The haul plan is still complete. This is a warning, not a failure.
    /// </summary>
    public sealed class PanTargetMiss
    {
        /// <summary>
        /// One road or pad cell that finished outside the finish tolerance.
        /// </summary>
        public readonly struct Cell
        {
            /// <summary>
            /// Initializes a failing pan cell.
            /// </summary>
            /// <param name="Column">Grid column.</param>
            /// <param name="Row">Grid row.</param>
            public Cell
                (
                Int32 Column,
                Int32 Row
                )
            {
                this.Column = Column;
                this.Row = Row;
            }

            /// <summary>Gets the grid column.</summary>
            public Int32 Column { get; }

            /// <summary>Gets the grid row.</summary>
            public Int32 Row { get; }
        }

        /// <summary>
        /// Initializes a pan-target warning.
        /// </summary>
        /// <param name="WorstColumn">Column of the cell with the largest absolute miss.</param>
        /// <param name="WorstRow">Row of the cell with the largest absolute miss.</param>
        /// <param name="WorstMm">Signed miss of that cell, in millimetres. Positive is above the pan target.</param>
        /// <param name="AboveBcy">Bank volume still above the pan target, in BCY.</param>
        /// <param name="BelowBcy">Bank volume still below the pan target, in BCY. Compaction is included.</param>
        /// <param name="Cells">Failing cells. Null is stored as an empty list.</param>
        public PanTargetMiss
            (
            Int32 WorstColumn,
            Int32 WorstRow,
            Double WorstMm,
            Double AboveBcy,
            Double BelowBcy,
            IReadOnlyList<Cell>? Cells
            )
        {
            this.WorstColumn = WorstColumn;
            this.WorstRow = WorstRow;
            this.WorstMm = WorstMm;
            this.AboveBcy = AboveBcy;
            this.BelowBcy = BelowBcy;
            this.Cells = Cells ?? new List<Cell>();
        }

        /// <summary>Gets how many pan cells finished outside the finish tolerance.</summary>
        public Int32 CellCount
        {
            get
            {
                return Cells.Count;
            }
        }

        /// <summary>Gets the column of the cell with the largest absolute miss.</summary>
        public Int32 WorstColumn { get; }

        /// <summary>Gets the row of the cell with the largest absolute miss.</summary>
        public Int32 WorstRow { get; }

        /// <summary>
        /// Gets the signed miss of the worst cell, in millimetres.
        /// Positive means the finished ground is above the pan target.
        /// </summary>
        public Double WorstMm { get; }

        /// <summary>
        /// Gets bank volume of failing cells that finished above the pan target, in BCY.
        /// This is the cut still sitting on the pan.
        /// </summary>
        public Double AboveBcy { get; }

        /// <summary>
        /// Gets bank volume of failing cells that finished below the pan target, in BCY.
        /// Compaction is included, matching the road-stage fill balance.
        /// </summary>
        public Double BelowBcy { get; }

        /// <summary>Gets the failing cells.</summary>
        public IReadOnlyList<Cell> Cells { get; }

        /// <summary>
        /// Formats the warning the run records. The plan is not failed.
        /// </summary>
        /// <returns>One line with the count, the worst cell, and the above and below totals.</returns>
        public String FormatMessage
            (
            )
        {
            // Positive is ground left high. The word tells the operator which way to correct the worst cell.
            String Side = WorstMm >= 0.0 ? "above" : "below";
            return String.Format(
                CultureInfo.InvariantCulture,
                "A road or pad cell is off its pan target by more than the finish tolerance. {0} cells, worst {1:F1} mm {2} at ({3}, {4}). Above {5:F1} BCY, below {6:F1} BCY.",
                CellCount,
                Math.Abs(WorstMm),
                Side,
                WorstColumn,
                WorstRow,
                AboveBcy,
                BelowBcy);
        }
    }
}
