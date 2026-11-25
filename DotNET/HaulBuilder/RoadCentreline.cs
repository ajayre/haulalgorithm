namespace HaulBuilder
{
    /// <summary>
    /// One access-road centreline in the project coordinate system (metres).
    /// Null overrides inherit the road defaults on <see cref="HaulBuilderOptions"/>.
    /// </summary>
    public sealed class RoadCentreline
    {
        /// <summary>
        /// Initializes a centreline. Vertices are copied.
        /// </summary>
        /// <param name="Vertices">Ordered centreline vertices, at least two.</param>
        public RoadCentreline
            (
            IReadOnlyList<(Double X, Double Y)> Vertices
            )
        {
            if (Vertices == null)
            {
                throw new ArgumentNullException(nameof(Vertices));
            }

            this.Vertices = new List<(Double X, Double Y)>(Vertices);
        }

        /// <summary>Gets the ordered centreline vertices in project metres.</summary>
        public List<(Double X, Double Y)> Vertices { get; }

        /// <summary>Gets or sets an override for the shoulder height above the field design, in feet. The crown rises above this.</summary>
        public Double? HeightFt { get; set; }

        /// <summary>Gets or sets an override for the side slope, horizontal over vertical.</summary>
        public Double? SideSlope { get; set; }

        /// <summary>Gets or sets an override for the crown cross slope, percent each side of the centreline.</summary>
        public Double? CrossSlopePct { get; set; }

        /// <summary>Gets or sets a local roadbed override in feet. Null keeps <see cref="HaulBuilderOptions.RoadNrcsMinRoadbedFt"/>.</summary>
        public Double? MinRoadbedFt { get; set; }

        /// <summary>Gets or sets an override for the minimum lane count.</summary>
        public Int32? MinLanes { get; set; }

        /// <summary>Gets or sets an override for the grade warning limit, in percent.</summary>
        public Double? MaxGradePct { get; set; }

        /// <summary>
        /// Copies the centreline and its overrides.
        /// </summary>
        /// <returns>A new centreline with the same vertices and overrides.</returns>
        public RoadCentreline Clone
            (
            )
        {
            RoadCentreline Copy = new RoadCentreline(Vertices);
            Copy.HeightFt = HeightFt;
            Copy.SideSlope = SideSlope;
            Copy.CrossSlopePct = CrossSlopePct;
            Copy.MinRoadbedFt = MinRoadbedFt;
            Copy.MinLanes = MinLanes;
            Copy.MaxGradePct = MaxGradePct;
            return Copy;
        }
    }
}
