using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// The diesel estimate prices the worked example and the optional haul pull.
    /// </summary>
    public sealed class DieselCostTests
    {
        /// <summary>Full-load rate from the worked example, in gallons per hour.</summary>
        private const Double MAX_FUEL = 20.0;

        /// <summary>Drawbar horsepower that keeps the worked example inside the fuel-rate band.</summary>
        private const Double DRAWBAR_HP = 300.0;

        /// <summary>Price from the worked example, in dollars per gallon.</summary>
        private const Double PRICE = 3.75;

        /// <summary>
        /// The published example: 100 cycles plus 5 grading hours, default load factors.
        /// </summary>
        [Fact]
        public void WorkedExampleMatchesTheSpec()
        {
            DieselCost.Result Estimate = DieselCost.Estimate(WorkedRequest());
            DieselCost.PhaseEstimate Load = Phase(Estimate, DieselCost.Phase.Load);
            DieselCost.PhaseEstimate Haul = Phase(Estimate, DieselCost.Phase.Haul);
            DieselCost.PhaseEstimate Dump = Phase(Estimate, DieselCost.Phase.Dump);
            DieselCost.PhaseEstimate Grade = Phase(Estimate, DieselCost.Phase.Grade);
            DieselCost.PhaseEstimate Return = Phase(Estimate, DieselCost.Phase.Return);

            Assert.False(Estimate.Blocked);
            Assert.Empty(Estimate.Warnings);
            Assert.Equal(DieselCost.LoadFactorSource.Default, Load.Source);
            Assert.Equal(1.0, Load.LoadFactor, 6);
            Assert.Equal(1.0, Load.FuelFraction, 6);
            Assert.Equal(0.60, Haul.LoadFactor, 6);
            Assert.Equal(0.72, Haul.FuelFraction, 6);
            Assert.Equal(0.55, Dump.LoadFactor, 6);
            Assert.Equal(0.685, Dump.FuelFraction, 6);
            Assert.Equal(0.72, Grade.FuelFraction, 6);
            Assert.Equal(0.35, Return.LoadFactor, 6);
            Assert.Equal(0.545, Return.FuelFraction, 6);
            Assert.Equal(33.33, Load.Gallons, 2);
            Assert.Equal(48.00, Haul.Gallons, 2);
            Assert.Equal(11.42, Dump.Gallons, 2);
            Assert.Equal(72.00, Grade.Gallons, 2);
            Assert.Equal(27.25, Return.Gallons, 2);
            Assert.Equal(192.0, Math.Round(Estimate.TotalGallons, 2));
            Assert.Equal(13.33, Math.Round(Estimate.TotalHours, 2));
            Assert.Equal(14.4, Math.Round(Estimate.AverageFuelGalPerHour, 1));
            Assert.Equal(72.0, Math.Round(Estimate.AverageLoadPercent, 0));
            Assert.Equal(720.0, Math.Round(Estimate.TotalCost, 2));
            Assert.Contains("BASE_FRACTION=0.30", Estimate.Assumptions);
            Assert.Contains("X_HAUL=0.60", Estimate.Assumptions);
            Assert.Contains("ROLLING_RESISTANCE=0.06 (default)", Estimate.Assumptions);
            Assert.Contains("SOIL_DENSITY=2700", Estimate.Assumptions);
        }

        /// <summary>
        /// A plan with no driving is not priced. Idle alone does not count as work.
        /// </summary>
        [Fact]
        public void NoDrivingBlocksTheEstimate()
        {
            DieselCost.Request Request = WorkedRequest();
            Request.LoadingHours = 0.0;
            Request.HaulingHours = 0.0;
            Request.DumpingHours = 0.0;
            Request.GradingHours = 0.0;
            Request.ReturningHours = 0.0;
            Request.IdleHours = 4.0;

            DieselCost.Result Estimate = DieselCost.Estimate(Request);

            Assert.True(Estimate.Blocked);
            Assert.Equal(0.0, Estimate.TotalGallons);
            Assert.Contains(
                "The plan produced no work, so diesel cost was not calculated.",
                Estimate.Warnings);
        }

        /// <summary>
        /// A full-load rate far from the drawbar rating is reported and still priced.
        /// </summary>
        [Fact]
        public void InconsistentTractorInputsWarn()
        {
            DieselCost.Request Request = WorkedRequest();
            Request.MaxDrawbarHp = 100.0;

            DieselCost.Result Estimate = DieselCost.Estimate(Request);

            Assert.False(Estimate.Blocked);
            Assert.Contains(
                "Fuel rate and drawbar HP look inconsistent. Check the tractor inputs.",
                Estimate.Warnings);
        }

        /// <summary>
        /// An all-return plan sits below the expected fuel-rate band.
        /// </summary>
        [Fact]
        public void UnusualAverageLoadWarns()
        {
            DieselCost.Request Request = new DieselCost.Request
            {
                MaxFuelGalPerHour = MAX_FUEL,
                MaxDrawbarHp = DRAWBAR_HP,
                PricePerGallon = PRICE,
                ReturningHours = 10.0,
            };

            DieselCost.Result Estimate = DieselCost.Estimate(Request);

            Assert.Contains(
                "Unusual average load. Check the plan's phase times.",
                Estimate.Warnings);
        }

        /// <summary>
        /// Engine-on idle is a tenth of the full-load rate and is added to the total.
        /// </summary>
        [Fact]
        public void IdleAddsATenthOfTheFullLoadRate()
        {
            DieselCost.Request Request = WorkedRequest();
            Request.IdleHours = 2.0;

            DieselCost.Result Estimate = DieselCost.Estimate(Request);

            Assert.Equal(4.0, Estimate.IdleGallons, 6);
            Assert.Equal(15.0, Estimate.IdleCost, 6);
            Assert.Equal(196.0, Math.Round(Estimate.TotalGallons, 2));
        }

        /// <summary>
        /// Part throttle reduces a light duty and leaves a full-load cut alone.
        /// </summary>
        [Fact]
        public void PartThrottleSavesFuelOnlyBelowTheLoadLimit()
        {
            DieselCost.Request Request = WorkedRequest();
            Request.ReducedThrottleLoad = true;
            Request.ReducedThrottleHaul = true;

            DieselCost.Result Estimate = DieselCost.Estimate(Request);
            DieselCost.PhaseEstimate Load = Phase(Estimate, DieselCost.Phase.Load);
            DieselCost.PhaseEstimate Haul = Phase(Estimate, DieselCost.Phase.Haul);

            Assert.False(Load.ReducedThrottle);
            Assert.Equal(1.0, Load.FuelFraction, 6);
            Assert.True(Haul.ReducedThrottle);
            Assert.Equal(0.72 * 0.85, Haul.FuelFraction, 6);
        }

        /// <summary>
        /// A caller override replaces the full-load cutting assumption.
        /// </summary>
        [Fact]
        public void OverrideReplacesTheCuttingLoad()
        {
            DieselCost.Request Request = new DieselCost.Request
            {
                MaxFuelGalPerHour = MAX_FUEL,
                MaxDrawbarHp = DRAWBAR_HP,
                PricePerGallon = PRICE,
                LoadingHours = 1.0,
                LoadFactorLoad = 0.5,
            };

            DieselCost.Result Estimate = DieselCost.Estimate(Request);
            DieselCost.PhaseEstimate Load = Phase(Estimate, DieselCost.Phase.Load);

            Assert.Equal(DieselCost.LoadFactorSource.Override, Load.Source);
            Assert.Equal(0.5, Load.LoadFactor, 6);
            Assert.Equal(0.65, Load.FuelFraction, 6);
            Assert.Equal(13.0, Load.Gallons, 6);
        }

        /// <summary>
        /// Weight, speed, and a level grade produce a haul factor other than the default.
        /// </summary>
        [Fact]
        public void WeightAndSpeedCalculateHaulAndReturn()
        {
            DieselCost.Result Estimate = DieselCost.Estimate(LevelCycleRequest(Grade: null));
            DieselCost.PhaseEstimate Haul = Phase(Estimate, DieselCost.Phase.Haul);
            DieselCost.PhaseEstimate Return = Phase(Estimate, DieselCost.Phase.Return);

            // 67,000 lb at 6% and 10 mph is 107.2 drawbar hp, over 170 hp available.
            Assert.Equal(DieselCost.LoadFactorSource.Calculated, Haul.Source);
            Assert.Equal(DieselCost.LoadFactorSource.Calculated, Return.Source);
            Assert.Equal(107.2 / 170.0, Haul.LoadFactor, 4);
            Assert.Equal(64.0 / 170.0, Return.LoadFactor, 4);
            Assert.DoesNotContain(Estimate.Warnings, Warning => Warning.Contains("cannot sustain"));
        }

        /// <summary>
        /// An uphill loaded trip raises the haul factor and lowers the return.
        /// </summary>
        [Fact]
        public void UphillLoadedRaisesHaulLoad()
        {
            DieselCost.Result Level = DieselCost.Estimate(LevelCycleRequest(Grade: null));
            DieselCost.Result Uphill = DieselCost.Estimate(LevelCycleRequest(Grade: 0.02));

            Assert.True(
                Phase(Uphill, DieselCost.Phase.Haul).LoadFactor
                > Phase(Level, DieselCost.Phase.Haul).LoadFactor);
            Assert.True(
                Phase(Uphill, DieselCost.Phase.Return).LoadFactor
                < Phase(Level, DieselCost.Phase.Return).LoadFactor);
        }

        /// <summary>
        /// A coasting downhill uses the floor. The opposite return that exceeds
        /// rated power is clamped and reported.
        /// </summary>
        [Fact]
        public void DownhillPullUsesTheFloorAndWarnsWhenTheReturnExceedsPower()
        {
            DieselCost.Result Estimate = DieselCost.Estimate(LevelCycleRequest(Grade: -0.20));

            Assert.Equal(DieselCost.X_MIN, Phase(Estimate, DieselCost.Phase.Haul).LoadFactor, 6);
            Assert.Equal(DieselCost.X_MAX, Phase(Estimate, DieselCost.Phase.Return).LoadFactor, 6);
            Assert.Contains(
                "Tractor cannot sustain the planned speed/load on 1 cycle. The plan's haul times may be too optimistic.",
                Estimate.Warnings);
        }

        /// <summary>
        /// An override wins even when a rig weight and a cycle are present.
        /// </summary>
        [Fact]
        public void OverrideWinsOverTheCalculatedHaul()
        {
            DieselCost.Request Request = LevelCycleRequest(Grade: null);
            Request.LoadFactorHaul = 0.5;

            DieselCost.Result Estimate = DieselCost.Estimate(Request);

            Assert.Equal(DieselCost.LoadFactorSource.Override, Phase(Estimate, DieselCost.Phase.Haul).Source);
            Assert.Equal(0.5, Phase(Estimate, DieselCost.Phase.Haul).LoadFactor, 6);
            Assert.Equal(
                DieselCost.LoadFactorSource.Calculated,
                Phase(Estimate, DieselCost.Phase.Return).Source);
        }

        /// <summary>
        /// Cycles without a rig weight stay on the default factors.
        /// </summary>
        [Fact]
        public void MissingWeightKeepsTheDefaultHaulFactor()
        {
            DieselCost.Request Request = LevelCycleRequest(Grade: null);
            Request.EmptyRigWeightLb = null;

            DieselCost.Result Estimate = DieselCost.Estimate(Request);

            Assert.Equal(DieselCost.LoadFactorSource.Default, Phase(Estimate, DieselCost.Phase.Haul).Source);
            Assert.Equal(DieselCost.X_HAUL, Phase(Estimate, DieselCost.Phase.Haul).LoadFactor, 6);
        }

        /// <summary>
        /// Guidance seconds land in the five duties. The empty approach is a return.
        /// </summary>
        [Fact]
        public void GuidancePhasesSumIntoTheFiveDuties()
        {
            Dictionary<LifecyclePlanner.Phase, Double> Seconds =
                new Dictionary<LifecyclePlanner.Phase, Double>
                {
                    [LifecyclePlanner.Phase.Cut] = 3600.0,
                    [LifecyclePlanner.Phase.Transit] = 7200.0,
                    [LifecyclePlanner.Phase.Fill] = 1800.0,
                    [LifecyclePlanner.Phase.Grade] = 3600.0,
                    [LifecyclePlanner.Phase.TurnToGrade] = 1800.0,
                    [LifecyclePlanner.Phase.TransitToNextCut] = 1800.0,
                    [LifecyclePlanner.Phase.LineupCut] = 1800.0,
                };

            DieselCost.PhaseHours Hours = DieselCost.HoursFromGuidance(Seconds);

            Assert.Equal(1.0, Hours.LoadingHours, 6);
            Assert.Equal(2.0, Hours.HaulingHours, 6);
            Assert.Equal(0.5, Hours.DumpingHours, 6);
            Assert.Equal(1.5, Hours.GradingHours, 6);
            Assert.Equal(1.0, Hours.ReturningHours, 6);
        }

        /// <summary>
        /// The speed model has no grade pass, and its other-time bucket is not fuel.
        /// </summary>
        [Fact]
        public void TimeModelHoursLeaveOutTheEfficiencyGap()
        {
            TimeResult Time = new TimeResult(
                99999.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                3600.0,
                7200.0,
                1800.0,
                1800.0,
                50000.0);

            DieselCost.PhaseHours Hours = DieselCost.HoursFromTimeModel(Time);

            Assert.Equal(1.0, Hours.LoadingHours, 6);
            Assert.Equal(2.0, Hours.HaulingHours, 6);
            Assert.Equal(0.5, Hours.DumpingHours, 6);
            Assert.Equal(0.0, Hours.GradingHours, 6);
            Assert.Equal(0.5, Hours.ReturningHours, 6);
        }

        /// <summary>
        /// Return time is stretched so one shared distance still yields the return speed.
        /// </summary>
        [Fact]
        public void AverageCycleReproducesReturnSpeed()
        {
            DieselCost.Cycle? Cycle = DieselCost.AverageCycle(
                5280.0,
                0.1,
                2640.0,
                0.1,
                10.0,
                null);

            Assert.NotNull(Cycle);
            Assert.Equal(5280.0, Cycle.DistanceFt, 6);
            Assert.Equal(0.1, Cycle.HaulTimeHours, 6);
            Assert.Equal(0.2, Cycle.ReturnTimeHours, 6);
        }

        /// <summary>
        /// Firm ground lowers the calculated haul factor against the tilled default.
        /// </summary>
        [Fact]
        public void FirmSoilLowersTheHaulFactor()
        {
            DieselCost.Request Tilled = LevelCycleRequest(Grade: null);
            DieselCost.Request Firm = LevelCycleRequest(Grade: null);
            Firm.SoilCondition = DieselCost.SoilCondition.Firm;

            Double TilledFactor = Phase(
                DieselCost.Estimate(Tilled), DieselCost.Phase.Haul).LoadFactor;
            Double FirmFactor = Phase(
                DieselCost.Estimate(Firm), DieselCost.Phase.Haul).LoadFactor;

            Assert.True(FirmFactor < TilledFactor);
            Assert.Contains(
                "ROLLING_RESISTANCE=0.04 (firm soil)",
                DieselCost.Estimate(Firm).Assumptions);
        }

        /// <summary>
        /// Unset diesel options are not an estimate, and a price alone is rejected.
        /// </summary>
        [Fact]
        public void ValidateRequiresTheThreeTractorInputsTogether()
        {
            DieselOptions Unset = new DieselOptions();
            DieselCost.Validate(Unset);
            Assert.False(DieselCost.RequestsEstimate(Unset));

            DieselOptions PriceOnly = new DieselOptions
            {
                PricePerGallon = PRICE,
            };
            Assert.Throws<ArgumentException>(() => DieselCost.Validate(PriceOnly));

            DieselOptions Complete = new DieselOptions
            {
                MaxFuelGalPerHour = MAX_FUEL,
                MaxDrawbarHp = DRAWBAR_HP,
                PricePerGallon = PRICE,
            };
            DieselCost.Validate(Complete);
            Assert.True(DieselCost.RequestsEstimate(Complete));
            Assert.Throws<ArgumentOutOfRangeException>(() => DieselCost.Validate(
                new DieselOptions
                {
                    MaxFuelGalPerHour = MAX_FUEL,
                    MaxDrawbarHp = DRAWBAR_HP,
                    PricePerGallon = PRICE,
                    LoadFactorHaul = 1.5,
                }));
        }

        /// <summary>The worked example, with a drawbar rating inside the sanity band.</summary>
        /// <returns>The request.</returns>
        private static DieselCost.Request WorkedRequest()
        {
            return new DieselCost.Request
            {
                MaxFuelGalPerHour = MAX_FUEL,
                MaxDrawbarHp = DRAWBAR_HP,
                PricePerGallon = PRICE,
                LoadingHours = 100.0 / 60.0,
                HaulingHours = 200.0 / 60.0,
                DumpingHours = 50.0 / 60.0,
                GradingHours = 5.0,
                ReturningHours = 150.0 / 60.0,
            };
        }

        /// <summary>
        /// One level-or-graded cycle at 10 mph with a 40,000 lb rig and a 10-yard bowl.
        /// </summary>
        /// <param name="Grade">Rise over run. Null is level ground.</param>
        /// <returns>A request whose haul and return can be calculated.</returns>
        private static DieselCost.Request LevelCycleRequest
            (
            Double? Grade
            )
        {
            Double DistanceFt = 5280.0;
            Double? CutFt = null;
            Double? FillFt = null;
            if (Grade.HasValue)
            {
                CutFt = 0.0;
                FillFt = Grade.Value * DistanceFt;
            }

            return new DieselCost.Request
            {
                MaxFuelGalPerHour = MAX_FUEL,
                MaxDrawbarHp = 200.0,
                PricePerGallon = PRICE,
                HaulingHours = 1.0,
                ReturningHours = 1.0,
                EmptyRigWeightLb = 40000.0,
                Cycles = new DieselCost.Cycle[]
                {
                    new DieselCost.Cycle(DistanceFt, 0.1, 0.1, 10.0, CutFt, FillFt),
                },
            };
        }

        /// <summary>Reads one phase line, failing when the estimate omitted it.</summary>
        /// <param name="Estimate">A completed estimate.</param>
        /// <param name="Phase">The duty.</param>
        /// <returns>The phase line.</returns>
        private static DieselCost.PhaseEstimate Phase
            (
            DieselCost.Result Estimate,
            DieselCost.Phase Phase
            )
        {
            DieselCost.PhaseEstimate? Line = Estimate.Find(Phase);
            Assert.NotNull(Line);
            return Line;
        }
    }
}
