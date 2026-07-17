using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace HaulBuilder
{
    /// <summary>
    /// UTM and AGD parsing utilities ported from landlevel.agd.
    /// </summary>
    static public class Agd
    {
        private const Double UTM_A = 6378137.0;
        private const Double UTM_F = 1.0 / 298.257223563;
        private const Double UTM_K0 = 0.9996;
        private static readonly Double UTM_B = UTM_A * (1.0 - UTM_F);
        private static readonly Double UTM_E2 = 1.0 - ((UTM_B * UTM_B) / (UTM_A * UTM_A));
        private static readonly Double UTM_EPRIME2 = UTM_E2 / (1.0 - UTM_E2);

        /// <summary>
        /// Converts latitude and longitude to UTM coordinates.
        /// </summary>
        /// <param name="LatitudeDeg">Latitude in degrees.</param>
        /// <param name="LongitudeDeg">Longitude in degrees.</param>
        /// <returns>Projected UTM coordinate.</returns>
        static public UtmCoord UtmFromLatlon
            (
            Double LatitudeDeg,
            Double LongitudeDeg
            )
        {
            // Enforce UTM latitude bounds to match the Python implementation.
            if ((LatitudeDeg < -80.0) || (LatitudeDeg > 84.0))
            {
                throw new ArgumentOutOfRangeException(nameof(LatitudeDeg), "Latitude out of UTM bounds (-80 to 84 degrees).");
            }

            // Normalize longitude to [-180, 180).
            Double LonNorm = ((((LongitudeDeg + Constants.DEGREES_PER_HALF_TURN) % Constants.DEGREES_PER_TURN) + Constants.DEGREES_PER_TURN) % Constants.DEGREES_PER_TURN) - Constants.DEGREES_PER_HALF_TURN;
            Int32 Zone = (Int32)Math.Floor(((LonNorm + Constants.DEGREES_PER_HALF_TURN) / 6.0) + 1.0);
            Boolean IsNorthern = (LatitudeDeg >= 0.0);

            Double LatRad = DegToRad(LatitudeDeg);
            Double LonRad = DegToRad(LonNorm);
            Double LonOrigin = ((Zone - 1) * 6) - 180 + 3;
            Double LonOriginRad = DegToRad(LonOrigin);

            Double SinLat = Math.Sin(LatRad);
            Double CosLat = Math.Cos(LatRad);
            Double N = UTM_A / Math.Sqrt(1.0 - (UTM_E2 * SinLat * SinLat));
            Double T = Math.Pow(Math.Tan(LatRad), 2.0);
            Double C = UTM_EPRIME2 * CosLat * CosLat;
            Double A = CosLat * (LonRad - LonOriginRad);

            Double M = UTM_A *
                (
                    ((1.0 - (UTM_E2 / 4.0) - ((3.0 * UTM_E2 * UTM_E2) / 64.0) - ((5.0 * Math.Pow(UTM_E2, 3.0)) / 256.0)) * LatRad) -
                    (((3.0 * UTM_E2) / 8.0) + ((3.0 * UTM_E2 * UTM_E2) / 32.0) + ((45.0 * Math.Pow(UTM_E2, 3.0)) / 1024.0)) * Math.Sin(2.0 * LatRad) +
                    (((15.0 * UTM_E2 * UTM_E2) / 256.0) + ((45.0 * Math.Pow(UTM_E2, 3.0)) / 1024.0)) * Math.Sin(4.0 * LatRad) -
                    ((35.0 * Math.Pow(UTM_E2, 3.0)) / 3072.0) * Math.Sin(6.0 * LatRad)
                );

            Double X = UTM_K0 * N *
                (
                    A +
                    ((1.0 - T + C) * Math.Pow(A, 3.0) / 6.0) +
                    ((5.0 - (18.0 * T) + (T * T) + (72.0 * C) - (58.0 * UTM_EPRIME2)) * Math.Pow(A, 5.0) / 120.0)
                );

            Double Y = UTM_K0 *
                (
                    M +
                    (N * Math.Tan(LatRad) *
                        (
                            ((A * A) / 2.0) +
                            ((5.0 - T + (9.0 * C) + (4.0 * C * C)) * Math.Pow(A, 4.0) / 24.0) +
                            ((61.0 - (58.0 * T) + (T * T) + (600.0 * C) - (330.0 * UTM_EPRIME2)) * Math.Pow(A, 6.0) / 720.0)
                        ))
                );

            Double Easting = X + 500000.0;
            Double Northing = Y + (IsNorthern ? 0.0 : 10000000.0);
            return new UtmCoord(Zone, IsNorthern, Easting, Northing);
        }

        /// <summary>
        /// Converts UTM coordinates to latitude and longitude.
        /// </summary>
        /// <param name="Easting">UTM easting in metres.</param>
        /// <param name="Northing">UTM northing in metres.</param>
        /// <param name="Zone">UTM zone number.</param>
        /// <param name="IsNorthern">Whether the coordinate is in the northern hemisphere.</param>
        /// <returns>Latitude and longitude in degrees.</returns>
        static public (Double Lat, Double Lon) LatlonFromUtm
            (
            Double Easting,
            Double Northing,
            Int32 Zone,
            Boolean IsNorthern
            )
        {
            // Undo false origins exactly as in the Python inverse projection.
            Double X = Easting - 500000.0;
            Double Y = Northing - (IsNorthern ? 0.0 : 10000000.0);

            Double M = Y / UTM_K0;
            Double Mu = M / (UTM_A * (1.0 - (UTM_E2 / 4.0) - ((3.0 * Math.Pow(UTM_E2, 2.0)) / 64.0) - ((5.0 * Math.Pow(UTM_E2, 3.0)) / 256.0)));
            Double E1 = (1.0 - Math.Sqrt(1.0 - UTM_E2)) / (1.0 + Math.Sqrt(1.0 - UTM_E2));

            Double Phi1 =
                Mu +
                (((3.0 * E1) / 2.0) - ((27.0 * Math.Pow(E1, 3.0)) / 32.0)) * Math.Sin(2.0 * Mu) +
                (((21.0 * Math.Pow(E1, 2.0)) / 16.0) - ((55.0 * Math.Pow(E1, 4.0)) / 32.0)) * Math.Sin(4.0 * Mu) +
                ((151.0 * Math.Pow(E1, 3.0)) / 96.0) * Math.Sin(6.0 * Mu) +
                ((1097.0 * Math.Pow(E1, 4.0)) / 512.0) * Math.Sin(8.0 * Mu);

            Double SinP = Math.Sin(Phi1);
            Double CosP = Math.Cos(Phi1);
            Double TanP = Math.Tan(Phi1);
            Double N1 = UTM_A / Math.Sqrt(1.0 - (UTM_E2 * SinP * SinP));
            Double T1 = TanP * TanP;
            Double C1 = UTM_EPRIME2 * CosP * CosP;
            Double R1 = UTM_A * (1.0 - UTM_E2) / Math.Pow(1.0 - (UTM_E2 * SinP * SinP), 1.5);
            Double D = X / (N1 * UTM_K0);

            Double LatRad =
                Phi1 -
                ((N1 * TanP) / R1) *
                (
                    ((D * D) / 2.0) -
                    ((5.0 + (3.0 * T1) + (10.0 * C1) - (4.0 * C1 * C1) - (9.0 * UTM_EPRIME2)) * Math.Pow(D, 4.0) / 24.0) +
                    ((61.0 + (90.0 * T1) + (298.0 * C1) + (45.0 * T1 * T1) - (252.0 * UTM_EPRIME2) - (3.0 * C1 * C1)) * Math.Pow(D, 6.0) / 720.0)
                );

            Double LonRadOffset =
                (
                    D -
                    ((1.0 + (2.0 * T1) + C1) * Math.Pow(D, 3.0) / 6.0) +
                    ((5.0 - (2.0 * C1) + (28.0 * T1) - (3.0 * C1 * C1) + (8.0 * UTM_EPRIME2) + (24.0 * T1 * T1)) * Math.Pow(D, 5.0) / 120.0)
                ) / CosP;

            Double LonOrigin = ((Zone - 1) * 6) - 180 + 3;
            Double LatDeg = LatRad * Constants.DEGREES_PER_HALF_TURN / Math.PI;
            Double LonDeg = LonOrigin + (LonRadOffset * Constants.DEGREES_PER_HALF_TURN / Math.PI);
            return (LatDeg, LonDeg);
        }

        /// <summary>
        /// Backward-compatible casing alias for UTM inverse.
        /// </summary>
        /// <param name="Easting">UTM easting in metres.</param>
        /// <param name="Northing">UTM northing in metres.</param>
        /// <param name="Zone">UTM zone number.</param>
        /// <param name="IsNorthern">Northern hemisphere flag.</param>
        /// <returns>Latitude and longitude in degrees.</returns>
        static public (Double Lat, Double Lon) LatLonFromUtm
            (
            Double Easting,
            Double Northing,
            Int32 Zone,
            Boolean IsNorthern
            )
        {
            // Keep old method spelling used by existing callers.
            return LatlonFromUtm(Easting, Northing, Zone, IsNorthern);
        }

        /// <summary>
        /// Projects a lat/lon ring to UTM easting/northing.
        /// </summary>
        /// <param name="LatLon">Ring of (lat, lon) points in degrees.</param>
        /// <returns>Projected (easting, northing) points in metres.</returns>
        static public List<(Double X, Double Y)> ProjectLatlonRing
            (
            List<(Double Lat, Double Lon)> LatLon
            )
        {
            // Reuse the same projection as AGD parsing to keep coordinate frames consistent.
            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>();
            foreach ((Double Lat, Double Lon) Point in LatLon)
            {
                UtmCoord Coord = UtmFromLatlon(Point.Lat, Point.Lon);
                Out.Add((Coord.Easting, Coord.Northing));
            }

            return Out;
        }

        /// <summary>
        /// Parses an AGD file and projects all retained points to UTM.
        /// </summary>
        /// <param name="Path">AGD file path.</param>
        /// <returns>Parsed AGD field data.</returns>
        static public AgdField Parse
            (
            String Path
            )
        {
            // Read text using UTF-8 first, with fallback to ignore malformed bytes.
            String[] Lines;
            try
            {
                Lines = File.ReadAllLines(Path);
            }
            catch (DecoderFallbackException)
            {
                String Text = File.ReadAllText(Path, Encoding.Default);
                Lines = Text.Split(new[] { "\r\n", "\n" }, StringSplitOptions.None);
            }

            List<AgdPoint> Interior = new List<AgdPoint>();
            List<AgdPoint> Boundary = new List<AgdPoint>();
            Int32 Skipped = 0;

            // Skip the first header line.
            for (Int32 Row = 1; Row < Lines.Length; Row += 1)
            {
                String Raw = Lines[Row].Trim();
                if (Raw.Length == 0)
                {
                    continue;
                }

                String[] Parts = Raw.Split(',');
                for (Int32 Index = 0; Index < Parts.Length; Index += 1)
                {
                    Parts[Index] = Parts[Index].Trim();
                }

                if (Parts.Length < 4)
                {
                    Skipped += 1;
                    continue;
                }

                String LatS = Parts[0];
                String LonS = Parts[1];
                String ExistingS = Parts[2];
                String ProposedS = Parts[3];
                String CodeS = (Parts.Length > 5) ? Parts[5].Trim() : String.Empty;
                String CodeUpper = CodeS.ToUpperInvariant();

                // Skip benchmark rows.
                if (CodeUpper.Contains("MB", StringComparison.Ordinal) || CodeUpper.Contains("BM", StringComparison.Ordinal))
                {
                    continue;
                }

                if ((ExistingS.Length == 0) || (ProposedS.Length == 0))
                {
                    Skipped += 1;
                    continue;
                }

                if (!Double.TryParse(LatS, NumberStyles.Float, CultureInfo.InvariantCulture, out Double Lat) ||
                    !Double.TryParse(LonS, NumberStyles.Float, CultureInfo.InvariantCulture, out Double Lon) ||
                    !Double.TryParse(ExistingS, NumberStyles.Float, CultureInfo.InvariantCulture, out Double Existing) ||
                    !Double.TryParse(ProposedS, NumberStyles.Float, CultureInfo.InvariantCulture, out Double Proposed))
                {
                    Skipped += 1;
                    continue;
                }

                if ((Existing == 0.0) || (Proposed == 0.0))
                {
                    Skipped += 1;
                    continue;
                }

                Boolean IsBoundary = (CodeUpper == "2PER");
                AgdPoint Point = new AgdPoint(Lat, Lon, Existing, Proposed, IsBoundary, 0.0, 0.0);
                if (IsBoundary)
                {
                    Boundary.Add(Point);
                }

                Interior.Add(Point);
            }

            if (Interior.Count == 0)
            {
                throw new InvalidDataException($"No valid elevation points found in AGD file: {Path}");
            }

            HashSet<Int32> Zones = new HashSet<Int32>();
            HashSet<Boolean> Norths = new HashSet<Boolean>();
            Double SumLat = 0.0;
            Double SumLon = 0.0;
            for (Int32 Index = 0; Index < Interior.Count; Index += 1)
            {
                AgdPoint Point = Interior[Index];
                UtmCoord Coord = UtmFromLatlon(Point.Lat, Point.Lon);
                Point.Easting = Coord.Easting;
                Point.Northing = Coord.Northing;
                Zones.Add(Coord.Zone);
                Norths.Add(Coord.IsNorthern);
                SumLat += Point.Lat;
                SumLon += Point.Lon;
            }

            if ((Zones.Count > 1) || (Norths.Count > 1))
            {
                throw new InvalidDataException("Field crosses a UTM zone boundary; not supported.");
            }

            Int32 Zone = FirstSetValue(Zones);
            Boolean IsNorthern = FirstSetValue(Norths);
            Double MeanLat = SumLat / Interior.Count;
            Double MeanLon = SumLon / Interior.Count;

            if (Skipped > 0)
            {
                // Emit the same warning behavior as Python without failing the parse.
                Console.Error.WriteLine($"WARNING: Skipped {Skipped} unusable AGD rows (missing/zero/short).");
            }

            return new AgdField(Interior, Boundary, Zone, IsNorthern, MeanLat, MeanLon, Skipped);
        }

        /// <summary>
        /// Converts degrees to radians.
        /// </summary>
        /// <param name="Deg">Angle in degrees.</param>
        /// <returns>Angle in radians.</returns>
        static private Double DegToRad
            (
            Double Deg
            )
        {
            // Keep conversion centralized so projection formulas stay readable.
            return Deg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
        }

        /// <summary>
        /// Returns the first item in a set.
        /// </summary>
        /// <typeparam name="T">Set element type.</typeparam>
        /// <param name="Set">Set to read from.</param>
        /// <returns>One value from the set.</returns>
        static private T FirstSetValue<T>
            (
            HashSet<T> Set
            )
        {
            // This mirrors Python's pop() after we already validated the set is non-empty.
            foreach (T Item in Set)
            {
                return Item;
            }

            throw new InvalidOperationException("Set is empty.");
        }
    }

    /// <summary>
    /// One projected UTM coordinate.
    /// </summary>
    public readonly struct UtmCoord
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="UtmCoord"/> struct.
        /// </summary>
        /// <param name="Zone">UTM zone number.</param>
        /// <param name="IsNorthern">Northern hemisphere flag.</param>
        /// <param name="Easting">UTM easting.</param>
        /// <param name="Northing">UTM northing.</param>
        public UtmCoord
            (
            Int32 Zone,
            Boolean IsNorthern,
            Double Easting,
            Double Northing
            )
        {
            // Persist immutable coordinate components.
            this.Zone = Zone;
            this.IsNorthern = IsNorthern;
            this.Easting = Easting;
            this.Northing = Northing;
        }

        /// <summary>
        /// Gets UTM zone number.
        /// </summary>
        public Int32 Zone { get; }

        /// <summary>
        /// Gets a value indicating whether the point is in the northern hemisphere.
        /// </summary>
        public Boolean IsNorthern { get; }

        /// <summary>
        /// Gets easting in metres.
        /// </summary>
        public Double Easting { get; }

        /// <summary>
        /// Gets northing in metres.
        /// </summary>
        public Double Northing { get; }
    }

    /// <summary>
    /// One AGD point with existing/proposed elevations and projected coordinates.
    /// </summary>
    public sealed class AgdPoint
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="AgdPoint"/> class.
        /// </summary>
        /// <param name="Lat">Latitude in degrees.</param>
        /// <param name="Lon">Longitude in degrees.</param>
        /// <param name="Existing">Existing elevation.</param>
        /// <param name="Proposed">Proposed elevation.</param>
        /// <param name="IsBoundary">Whether this row is a 2PER boundary row.</param>
        /// <param name="Easting">Projected easting in metres.</param>
        /// <param name="Northing">Projected northing in metres.</param>
        public AgdPoint
            (
            Double Lat,
            Double Lon,
            Double Existing,
            Double Proposed,
            Boolean IsBoundary,
            Double Easting,
            Double Northing
            )
        {
            // Store point attributes exactly as parsed.
            this.Lat = Lat;
            this.Lon = Lon;
            this.Existing = Existing;
            this.Proposed = Proposed;
            this.IsBoundary = IsBoundary;
            this.Easting = Easting;
            this.Northing = Northing;
        }

        /// <summary>
        /// Gets latitude in degrees.
        /// </summary>
        public Double Lat { get; }

        /// <summary>
        /// Gets longitude in degrees.
        /// </summary>
        public Double Lon { get; }

        /// <summary>
        /// Gets existing elevation.
        /// </summary>
        public Double Existing { get; }

        /// <summary>
        /// Gets proposed elevation.
        /// </summary>
        public Double Proposed { get; }

        /// <summary>
        /// Gets a value indicating whether this point is a boundary row.
        /// </summary>
        public Boolean IsBoundary { get; }

        /// <summary>
        /// Gets or sets projected easting in metres.
        /// </summary>
        public Double Easting { get; set; }

        /// <summary>
        /// Gets or sets projected northing in metres.
        /// </summary>
        public Double Northing { get; set; }

        /// <summary>
        /// Gets existing minus proposed elevation.
        /// </summary>
        public Double Delta
        {
            get
            {
                // Keep positive as cut, matching the Python sign convention.
                return this.Existing - Proposed;
            }
        }
    }

    /// <summary>
    /// Parsed AGD field containing projected interior and boundary points.
    /// </summary>
    public sealed class AgdField
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="AgdField"/> class.
        /// </summary>
        /// <param name="Points">All retained AGD points.</param>
        /// <param name="Boundary">Boundary-tagged points.</param>
        /// <param name="Zone">Shared UTM zone.</param>
        /// <param name="IsNorthern">Shared hemisphere flag.</param>
        /// <param name="MeanLat">Mean latitude of retained points.</param>
        /// <param name="MeanLon">Mean longitude of retained points.</param>
        /// <param name="Skipped">Count of skipped rows.</param>
        public AgdField
            (
            List<AgdPoint> Points,
            List<AgdPoint> Boundary,
            Int32 Zone,
            Boolean IsNorthern,
            Double MeanLat,
            Double MeanLon,
            Int32 Skipped
            )
        {
            // Preserve parsed collections and shared projection metadata.
            this.Points = Points;
            this.Boundary = Boundary;
            this.Zone = Zone;
            this.IsNorthern = IsNorthern;
            this.MeanLat = MeanLat;
            this.MeanLon = MeanLon;
            this.Skipped = Skipped;
        }

        /// <summary>
        /// Gets all retained AGD points used for grid construction.
        /// </summary>
        public List<AgdPoint> Points { get; }

        /// <summary>
        /// Gets boundary points.
        /// </summary>
        public List<AgdPoint> Boundary { get; }

        /// <summary>
        /// Gets shared UTM zone.
        /// </summary>
        public Int32 Zone { get; }

        /// <summary>
        /// Gets a value indicating whether the field is in the northern hemisphere.
        /// </summary>
        public Boolean IsNorthern { get; }

        /// <summary>
        /// Gets mean latitude.
        /// </summary>
        public Double MeanLat { get; }

        /// <summary>
        /// Gets mean longitude.
        /// </summary>
        public Double MeanLon { get; }

        /// <summary>
        /// Gets the number of skipped AGD rows.
        /// </summary>
        public Int32 Skipped { get; }

        /// <summary>
        /// Gets all points used for processing.
        /// </summary>
        public List<AgdPoint> AllUsed
        {
            get
            {
                // Preserve Python's alias property name.
                return this.Points;
            }
        }

        /// <summary>
        /// Converts projected XY to latitude/longitude in this field's UTM frame.
        /// </summary>
        /// <param name="Easting">Easting in metres.</param>
        /// <param name="Northing">Northing in metres.</param>
        /// <returns>Latitude and longitude in degrees.</returns>
        public (Double Lat, Double Lon) LatLonFromUtm
            (
            Double Easting,
            Double Northing
            )
        {
            // Instance helper mirrors Python field-scoped convenience.
            return Agd.LatLonFromUtm(Easting, Northing, this.Zone, this.IsNorthern);
        }
    }
}
