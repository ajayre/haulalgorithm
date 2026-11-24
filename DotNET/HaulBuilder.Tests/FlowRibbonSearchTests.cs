using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Xunit;

namespace HaulBuilder.Tests;

/// <summary>
/// Drives the flow ribbon on a 10 acre mound-and-basin field and checks
/// that each load is one downstream ribbon.
/// </summary>
public sealed class FlowRibbonSearchTests
{
	/// <summary>Ten acre square, in square metres.</summary>
	private const double TEN_ACRES_M2 = 40468.564224;

	/// <summary>
	/// The footprint shortcut keeps the same cells as the full sample grid.
	/// </summary>
	[Fact]
	public void CoverageShortcutMatchesTheSampleGrid()
	{
		string? text = FlowRibbonSearch.CoverageShortcutMismatch();
		Assert.True(text == null, text);
	}

	/// <summary>
	/// A northwest mound drains into a southeast basin, and the opening loads are full bowls.
	/// </summary>
	[Fact]
	public void TenAcreMoundDrainsDownstreamInRibbons()
	{
		string path = WriteTenAcreAgd();
		AgdField agd = Agd.Parse(path);
		Grid grid = GridMath.BuildGrid(agd, 8.5, 1.3, 1.0);
		FlowField flowField = FlowFieldSolver.Compute(grid);
		HaulWorkLedger ledger = HaulWorkLedger.Build(grid, 1.0);
		double num = 15.384615384615383;
		HaulOptimizationResult haulOptimizationResult = FlowRibbonSearch.Optimize(grid, flowField, ledger, HaulLaneBuilder.LaneAxis.UserHeading, num, DeferEdgeWork: false, null, AllowAnyDeferredRemainder: false, 2.5908);
		Assert.True(haulOptimizationResult.Validation.Success, string.Join("; ", haulOptimizationResult.Validation.Issues.Take(8)));
		Assert.NotEmpty(haulOptimizationResult.PassResult.Passes);
		Assert.All(haulOptimizationResult.PassResult.Passes, (Pass Pass) =>
		{
			Assert.True(Pass.RibbonOrder);
		});
		int num2 = 0;
		int num3 = 0;
		int num4 = 0;
		int num5 = 0;
		int value = 0;
		int num6 = 0;
		int num7 = -1;
		int num8 = 0;
		double num9 = 0.0;
		double num10 = 0.0;
		double num11 = 0.0;
		double num12 = 0.0;
		foreach (Pass pass4 in haulOptimizationResult.PassResult.Passes)
		{
			num12 += pass4.LoadBcy;
			if (pass4.LoadBcy >= num - 0.05)
			{
				num5++;
				num10 += pass4.HaulDistanceM;
				value = num6;
			}
			else
			{
				if (num7 < 0)
				{
					num7 = num8;
				}
				num6++;
			}
			num8++;
			CoveragePlanner.PlanResult planResult = CoveragePlanner.PlanRibbon(pass4.CutCells, grid, 2.6);
			CoveragePlanner.PlanResult planResult2 = CoveragePlanner.PlanRibbon(pass4.FillCells, grid, 2.6);
			num11 += planResult.WorkDistanceM + planResult2.WorkDistanceM;
			num9 += pass4.HaulDistanceM;
			for (int num13 = 1; num13 < pass4.CutCells.Count; num13++)
			{
				num3++;
				int val = Math.Abs(pass4.CutCells[num13].Cell.Bx - pass4.CutCells[num13 - 1].Cell.Bx);
				int val2 = Math.Abs(pass4.CutCells[num13].Cell.By - pass4.CutCells[num13 - 1].Cell.By);
				if (Math.Max(val, val2) > 2)
				{
					num4++;
				}
			}
			(int, int) item = pass4.CutCells[pass4.CutCells.Count - 1].Cell;
			double num14 = flowField.Fx[item.Item2, item.Item1];
			double num15 = flowField.Fy[item.Item2, item.Item1];
			double num16 = pass4.FillStartXy.X - pass4.CutEndXy.X;
			double num17 = pass4.FillStartXy.Y - pass4.CutEndXy.Y;
			double num18 = Math.Sqrt(num16 * num16 + num17 * num17);
			if (num18 > 1.0)
			{
				double num19 = num16 / num18 * num14 + num17 / num18 * num15;
				if (num19 < 0.0)
				{
					num2++;
				}
			}
		}
		double value2 = (double)(grid.Width * grid.Height) * grid.GridSizeM * grid.GridSizeM / 4046.8564224;
		int count = haulOptimizationResult.PassResult.Passes.Count;
		Pass pass = haulOptimizationResult.PassResult.Passes[0];
		Pass pass2 = haulOptimizationResult.PassResult.Passes[Math.Min(20, count - 1)];
		Pass pass3 = haulOptimizationResult.PassResult.Passes[Math.Min(40, count - 1)];
		(int, int) item2 = pass.CutCells[0].Cell;
		(int, int) item3 = pass.CutCells[pass.CutCells.Count - 1].Cell;
		(int, int) item4 = pass.FillCells[0].Cell;
		string value3 = $"mid {pass2.CutCells[0].Cell.Bx},{pass2.CutCells[0].Cell.By}->{pass2.CutCells[pass2.CutCells.Count - 1].Cell.Bx},{pass2.CutCells[pass2.CutCells.Count - 1].Cell.By} fill {pass2.FillCells[0].Cell.Bx},{pass2.FillCells[0].Cell.By} bcy {pass2.LoadBcy:F1} haul {pass2.HaulDistanceM:F0} late {pass3.CutCells[0].Cell.Bx},{pass3.CutCells[0].Cell.By}->{pass3.CutCells[pass3.CutCells.Count - 1].Cell.Bx},{pass3.CutCells[pass3.CutCells.Count - 1].Cell.By} fill {pass3.FillCells[0].Cell.Bx},{pass3.FillCells[0].Cell.By} bcy {pass3.LoadBcy:F1} haul {pass3.HaulDistanceM:F0}";
		double value4 = num12 / (double)count;
		string userMessage = $"loads={count} meanBcy={value4:F2} cap={num:F2} against={num2} cutGaps={num4}/{num3} haulM={num9:F0} meanHaul={num9 / (double)count:F1} workM={num11:F0} acres={value2:F2} deferredCut={haulOptimizationResult.UnmatchedCutBcy:F2} deferredFill={haulOptimizationResult.UnmatchedFillBcy:F2} full={num5} fullHaul={((num5 == 0) ? 0.0 : (num10 / (double)num5)):F1} partialsBeforeLastFull={value} firstPartial={num7} firstBcy={pass.LoadBcy:F2} firstCut={item2.Item1},{item2.Item2}->{item3.Item1},{item3.Item2} n={pass.CutCells.Count} fill0={item4.Item1},{item4.Item2} grid={grid.Width}x{grid.Height} {value3}";
		Assert.True(num7 < 0 || num7 >= 20, userMessage);
		Assert.True((double)num5 * (num - 0.05) >= num12 * 0.75, userMessage);
		Assert.True(num2 * 5 < count, userMessage);
		Assert.True(num9 / (double)count < 140.0, userMessage);
	}

	/// <summary>
	/// Excess cut with only a dusting of fill is the material imbalance, not a failed search.
	/// </summary>
	[Fact]
	public void ExcessCutWithDustFillIsTheImbalance()
	{
		double[,] array = new double[4, 4];
		bool[,] array2 = new bool[4, 4];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				array2[i, j] = true;
			}
		}
		array[1, 1] = 0.05;
		array[2, 2] = -9E-07;
		Grid grid = new Grid(1.0, 4, 4, 0.0, 0.0, 12, IsNorthern: true, 0.0, 0.0, array, new double[4, 4], array2, 1.3, 1.0, 61.0, new List<List<(double, double)>>());
		HaulWorkLedger ledger = HaulWorkLedger.Build(grid, 0.0001);
		FlowField flow = new FlowField(new double[4, 4], new double[4, 4], new double[4, 4], new double[4, 4], new bool[4, 4], "test");
		HaulOptimizationResult haulOptimizationResult = FlowRibbonSearch.Optimize(grid, flow, ledger, HaulLaneBuilder.LaneAxis.UserHeading, 15.0);
		Assert.True(haulOptimizationResult.Validation.Success, string.Join("; ", haulOptimizationResult.Validation.Issues.Take(4)));
		Assert.True(haulOptimizationResult.UnmatchedCutBcy > 0.0);
		Assert.True(haulOptimizationResult.UnmatchedFillBcy < 0.001);
	}

	/// <summary>
	/// Ten-acre guidance keeps the blade down through the cut and the fill.
	/// </summary>
	[Fact]
	public void TenAcreGuidanceKeepsBladeDownCutAndFill()
	{
		string agdPath = WriteTenAcreAgd();
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			GridSizeFt = 8.5,
			SkipGuidance = false,
			EnableMinimumWorkLength = false
		};
		haulBuilderOptions.Optimization.EnableRouteConsolidation = false;
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.LifecycleGuidance);
		Assert.True(haulBuilderResult.LifecycleGuidance.Success, haulBuilderResult.LifecycleGuidance.FailureReason);
		Assert.NotNull(haulBuilderResult.Grid);
		WriteRibbonProof(haulBuilderResult);
		int num = haulBuilderResult.GuidancePoints.Select((GuidancePoint Point) => Point.PassNo).Distinct().Count();
		int num2 = (from Point in haulBuilderResult.GuidancePoints
			where Point.Action == "CUT" && Point.Blade == "DOWN"
			select Point.PassNo).Distinct().Count();
		int num3 = (from Point in haulBuilderResult.GuidancePoints
			where Point.Action == "FILL" && Point.Blade == "DOWN"
			select Point.PassNo).Distinct().Count();
		Assert.True(num > 0, "guidance published no routes");
		Assert.True(num2 * 5 >= num * 4, string.Format(CultureInfo.InvariantCulture, "blade-down cut on {0} of {1} routes", num2, num));
		Assert.True(num3 * 5 >= num * 4, string.Format(CultureInfo.InvariantCulture, "blade-down fill on {0} of {1} routes", num3, num));
	}

	/// <summary>
	/// The first Shop 2 load has blade-down cut and fill work.
	/// </summary>
	[Fact]
	public void Shop2FirstRouteHasBladeDownCutAndFill()
	{
		string text = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = text,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.Route);
		Assert.NotNull(haulBuilderResult.Grid);
		Pass pass = haulBuilderResult.Route.OrderedPasses[0];
		AgdField field = Agd.Parse(text);
		Route route = new Route(new List<int> { 0 }, new List<Pass> { pass }, 0.0);
		GuidancePlanner.BuildResult Built = GuidancePlanner.Build(route, haulBuilderResult.Grid, field, 0.0, haulBuilderOptions);
		int num = Enumerable.Count(Built.Guidance.Points, (GuidancePoint Point) => Point.Action == "CUT" && Point.Blade == "DOWN");
		int num2 = Enumerable.Count(Built.Guidance.Points, (GuidancePoint Point) => Point.Action == "FILL" && Point.Blade == "DOWN");
		double num3 = WorstBladeBendDeg(Built.Guidance.Points, "CUT");
		double num4 = WorstBladeBendDeg(Built.Guidance.Points, "FILL");
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof"));
		Directory.CreateDirectory(fullPath);
		double bladeWidthM = 2.5908;
		CoveragePlanner.PlanResult planResult = CoveragePlanner.PlanRibbon(pass.CutCells, haulBuilderResult.Grid, bladeWidthM);
		CoveragePlanner.PlanResult planResult2 = CoveragePlanner.PlanRibbon(pass.FillCells, haulBuilderResult.Grid, bladeWidthM);
		File.WriteAllBytes(Path.Combine(fullPath, "shop2-route-001.png"), Render.RenderSinglePairPng(haulBuilderResult.Grid, Built.Guidance.Points, bladeWidthM, 4, 2, ShowRoute: false, pass.CutCells, pass.FillCells));
		string text2 = string.Join(" ", from Point in Built.Guidance.Points
			group Point by Point.Action + ":" + Point.Blade into Group
			orderby Group.Key
			select Group.Key + "=" + Group.Count().ToString(CultureInfo.InvariantCulture) + "/" + StrokePathM(Built.Guidance.Points, Group.Key).ToString("F0", CultureInfo.InvariantCulture) + "m");
		File.WriteAllText(Path.Combine(fullPath, "shop2-route-001.txt"), string.Format(CultureInfo.InvariantCulture, "ribbon={0} cutCells={1} fillCells={2} load={3:F2} haul={4:F1} cutPts={5} fillPts={6} guidancePts={7} cutBend={8:F0} fillBend={9:F0} cutRuns={10} fillRuns={11} {12}", pass.RibbonOrder, pass.CutCells.Count, pass.FillCells.Count, pass.LoadBcy, pass.HaulDistanceM, num, num2, Built.Guidance.Points.Count, num3, num4, planResult.Runs.Count, planResult2.Runs.Count, text2));
		Assert.True(pass.CutCells.Count > 1, "route 1 was given no cut");
		Assert.True(pass.FillCells.Count > 1, "route 1 was given no fill");
		Assert.True(num >= 2, "route 1 guidance has no blade-down cut");
		Assert.True(num2 >= 2, "route 1 guidance has no blade-down fill");
	}

	/// <summary>
	/// The first hundred found Shop 2 loads are drawn, and each fill shares edges.
	/// </summary>
	[Fact]
	public void Shop2Route1FillSharesEdges()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double num = 2.5908;
		(double[,], double[,]) tuple = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		int num2 = Math.Min(100, passes.Count);
		double[,] array = haulBuilderResult.Grid.Delta();
		StringBuilder stringBuilder = new StringBuilder();
		for (int i = 0; i < num2; i++)
		{
			Pass pass = passes[i];
			File.WriteAllBytes(Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", i + 1)), Render.RenderSinglePairPng(haulBuilderResult.Grid, CenterlineTrack(pass, haulBuilderResult.Grid, num), num, 16, 2, ShowRoute: false, pass.CutCells, pass.FillCells, tuple.Item1, tuple.Item2, 1, array));
			foreach (var cutCell in pass.CutCells)
			{
				array[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				array[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
			(double Length, double Width) tuple2 = StripExtent(pass.CutCells);
			double item = tuple2.Length;
			double item2 = tuple2.Width;
			(double Length, double Width) tuple3 = StripExtent(pass.FillCells);
			double item3 = tuple3.Length;
			double item4 = tuple3.Width;
			bool flag = item2 > 5.0 || item2 > item * 0.55;
			bool flag2 = item4 > 5.0 || item4 > item3 * 0.55;
			if ((!CellsTouch(pass.FillCells, EdgesOnly: true) || !CellsTouch(pass.CutCells, EdgesOnly: false)) | flag | flag2)
			{
				stringBuilder.AppendLine(string.Format(CultureInfo.InvariantCulture, "route {0} load={1:F2} cut={2} {3:F1}x{4:F1} fill={5} {6:F1}x{7:F1}", i + 1, pass.LoadBcy, pass.CutCells.Count, item, item2, pass.FillCells.Count, item3, item4));
			}
		}
		Assert.True(stringBuilder.Length == 0, stringBuilder.ToString());
	}

	/// <summary>
	/// Routes 500 through 550, in the order the search found them, are drawn on the remaining ground.
	/// </summary>
	[Fact]
	public void Shop2FoundRoutes500To550AreDrawn()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double num = 2.5908;
		(double[,], double[,]) tuple = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		Assert.True(passes.Count >= 550, "the search found fewer than 550 loads");
		double[,] array = haulBuilderResult.Grid.Delta();
		int num2 = 0;
		for (int i = 0; i < 550; i++)
		{
			Pass pass = passes[i];
			int num3 = i + 1;
			if (num3 >= 500)
			{
				File.WriteAllBytes(Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", num3)), Render.RenderSinglePairPng(haulBuilderResult.Grid, CenterlineTrack(pass, haulBuilderResult.Grid, num), num, 16, 2, ShowRoute: false, pass.CutCells, pass.FillCells, tuple.Item1, tuple.Item2, 1, array));
				num2++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				array[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				array[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(51, num2);
	}

	/// <summary>
	/// Routes 1000 through 1049, in the order the search found them, are drawn on the remaining ground.
	/// </summary>
	[Fact]
	public void Shop2FoundRoutes1000To1049AreDrawn()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double num = 2.5908;
		(double[,], double[,]) tuple = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		Assert.True(passes.Count >= 1049, "the search found fewer than 1049 loads");
		double[,] array = haulBuilderResult.Grid.Delta();
		int num2 = 0;
		for (int i = 0; i < 1049; i++)
		{
			Pass pass = passes[i];
			int num3 = i + 1;
			if (num3 >= 1000)
			{
				File.WriteAllBytes(Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", num3)), Render.RenderSinglePairPng(haulBuilderResult.Grid, CenterlineTrack(pass, haulBuilderResult.Grid, num), num, 16, 2, ShowRoute: false, pass.CutCells, pass.FillCells, tuple.Item1, tuple.Item2, 1, array));
				num2++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				array[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				array[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(50, num2);
	}

	/// <summary>
	/// Routes 2000 through 2049, in the order the search found them, drawn
	/// on the dirt still left when that load is taken.
	/// </summary>
	[Fact]
	public void Shop2FoundRoutes2000To2049AreDrawn()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double bladeM = 2.5908;
		(double[,], double[,]) arrows = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		Assert.True(passes.Count >= 2049, "the search found fewer than 2049 loads");
		double[,] surface = haulBuilderResult.Grid.Delta();
		int written = 0;
		for (int i = 0; i < 2049; i++)
		{
			Pass pass = passes[i];
			int number = i + 1;
			if (number >= 2000)
			{
				File.WriteAllBytes(
					Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", number)),
					Render.RenderSinglePairPng(
						haulBuilderResult.Grid,
						CenterlineTrack(pass, haulBuilderResult.Grid, bladeM),
						bladeM,
						16,
						2,
						ShowRoute: false,
						pass.CutCells,
						pass.FillCells,
						arrows.Item1,
						arrows.Item2,
						1,
						surface));
				written++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				surface[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				surface[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(50, written);
	}


	/// <summary>
	/// Routes 2429 and 2430, in the order the search found them, drawn on the
	/// whole field with a flow arrow on every cell. The ground is the dirt
	/// still left when that load is taken.
	/// </summary>
	[Fact]
	public void Shop2FoundRoutes2429And2430ShowTheField()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double bladeM = 2.5908;
		(double[,], double[,]) arrows = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		Assert.True(passes.Count >= 2430, "the search found fewer than 2430 loads");
		double[,] surface = haulBuilderResult.Grid.Delta();
		int written = 0;
		for (int i = 0; i < 2430; i++)
		{
			Pass pass = passes[i];
			int number = i + 1;
			if ((number == 2429) || (number == 2430))
			{
				File.WriteAllBytes(
					Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", number)),
					Render.RenderSinglePairPng(
						haulBuilderResult.Grid,
						CenterlineTrack(pass, haulBuilderResult.Grid, bladeM),
						bladeM,
						16,
						2,
						ShowRoute: false,
						pass.CutCells,
						pass.FillCells,
						arrows.Item1,
						arrows.Item2,
						1,
						surface,
						EntireField: true));
				written++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				surface[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				surface[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(2, written);
	}

	/// <summary>
	/// Route 2794, in the order the search found it, drawn on the whole field
	/// with a flow arrow on every cell. The ground is the dirt still left
	/// when that load is taken.
	/// </summary>
	[Fact]
	public void Shop2FoundRoute2794ShowsTheField()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double bladeM = 2.5908;
		(double[,], double[,]) arrows = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		Assert.True(passes.Count >= 2794, "the search found fewer than 2794 loads");
		double[,] surface = haulBuilderResult.Grid.Delta();
		int written = 0;
		for (int i = 0; i < 2794; i++)
		{
			Pass pass = passes[i];
			int number = i + 1;
			if (number == 2794)
			{
				File.WriteAllBytes(
					Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", number)),
					Render.RenderSinglePairPng(
						haulBuilderResult.Grid,
						CenterlineTrack(pass, haulBuilderResult.Grid, bladeM),
						bladeM,
						16,
						2,
						ShowRoute: false,
						pass.CutCells,
						pass.FillCells,
						arrows.Item1,
						arrows.Item2,
						1,
						surface,
						EntireField: true));
				written++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				surface[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				surface[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(1, written);
	}

	/// <summary>
	/// Route 2871, in the order the search found it, drawn on the whole field
	/// with a flow arrow on every cell. The ground is the dirt still left
	/// when that load is taken.
	/// </summary>
	[Fact]
	public void Shop2FoundRoute2871ShowsTheField()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double bladeM = 2.5908;
		(double[,], double[,]) arrows = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		Assert.True(passes.Count >= 2871, "the search found fewer than 2871 loads");
		double[,] surface = haulBuilderResult.Grid.Delta();
		int written = 0;
		for (int i = 0; i < 2871; i++)
		{
			Pass pass = passes[i];
			int number = i + 1;
			if (number == 2871)
			{
				File.WriteAllBytes(
					Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", number)),
					Render.RenderSinglePairPng(
						haulBuilderResult.Grid,
						CenterlineTrack(pass, haulBuilderResult.Grid, bladeM),
						bladeM,
						16,
						2,
						ShowRoute: false,
						pass.CutCells,
						pass.FillCells,
						arrows.Item1,
						arrows.Item2,
						1,
						surface,
						EntireField: true));
				written++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				surface[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				surface[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(1, written);
	}

	/// <summary>
	/// The last full bowl, in the order the search found it, drawn on the
	/// whole field with a flow arrow on every cell. The ground is the dirt
	/// still left when that load is taken.
	/// </summary>
	[Fact]
	public void Shop2FoundLastFullLoadShowsTheField()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double bladeM = 2.5908;
		double fullBcy = (20.0 / 1.3) - 0.05;
		(double[,], double[,]) arrows = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		int lastFull = -1;
		for (int i = 0; i < passes.Count; i++)
		{
			if (passes[i].LoadBcy >= fullBcy)
			{
				lastFull = i;
			}
		}
		Assert.True(lastFull >= 0, "the search found no full load");
		double[,] surface = haulBuilderResult.Grid.Delta();
		int written = 0;
		for (int i = 0; i <= lastFull; i++)
		{
			Pass pass = passes[i];
			int number = i + 1;
			if (i == lastFull)
			{
				File.WriteAllBytes(
					Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", number)),
					Render.RenderSinglePairPng(
						haulBuilderResult.Grid,
						CenterlineTrack(pass, haulBuilderResult.Grid, bladeM),
						bladeM,
						16,
						2,
						ShowRoute: false,
						pass.CutCells,
						pass.FillCells,
						arrows.Item1,
						arrows.Item2,
						1,
						surface,
						EntireField: true));
				written++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				surface[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				surface[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(1, written);
	}

	/// <summary>
	/// Whole-field sheet for route 2888 with the highest and lowest cells
	/// ringed. The cells are the extremes measured on that sheet: column 59
	/// row 2 is 895 mm above target, and column 2 row 14 is 548 mm below.
	/// </summary>
	[Fact]
	public void Shop2Route2888SpotsAreMarked()
	{
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		string sourcePath = Path.Combine(fullPath, "route-2888.png");
		string destPath = Path.Combine(fullPath, "route-2888-spots.png");
		Assert.True(File.Exists(sourcePath), "route 2888 sheet is missing");
		Render.MarkSpotsOnSheet(
			sourcePath,
			destPath,
			16,
			59,
			2,
			"HIGH +895 mm",
			2,
			14,
			"LOW -548 mm");
		Assert.True(new FileInfo(destPath).Length > 100000);
	}

	/// <summary>
	/// The last fifty Shop 2 loads, in the order the search found them, drawn
	/// on the dirt still left when that load is taken.
	/// </summary>
	[Fact]
	public void Shop2FoundLastFiftyRoutesAreDrawn()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.PassResult);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-2026-11-20-found"));
		Directory.CreateDirectory(fullPath);
		double bladeM = 2.5908;
		(double[,], double[,]) arrows = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		List<Pass> passes = haulBuilderResult.PassResult.Passes;
		Assert.True(passes.Count >= 50, "the search found fewer than 50 loads");
		double[,] surface = haulBuilderResult.Grid.Delta();
		int written = 0;
		int start = passes.Count - 50;
		for (int i = 0; i < passes.Count; i++)
		{
			Pass pass = passes[i];
			int number = i + 1;
			if (i >= start)
			{
				File.WriteAllBytes(
					Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", number)),
					Render.RenderSinglePairPng(
						haulBuilderResult.Grid,
						CenterlineTrack(pass, haulBuilderResult.Grid, bladeM),
						bladeM,
						16,
						2,
						ShowRoute: false,
						pass.CutCells,
						pass.FillCells,
						arrows.Item1,
						arrows.Item2,
						1,
						surface));
				written++;
			}
			foreach (var cutCell in pass.CutCells)
			{
				surface[cutCell.Cell.By, cutCell.Cell.Bx] -= cutCell.DepthM;
			}
			foreach (var fillCell in pass.FillCells)
			{
				surface[fillCell.Cell.By, fillCell.Cell.Bx] += fillCell.DepthM;
			}
		}
		Assert.Equal(50, written);
	}

	/// <summary>
	/// The first Shop 2 load is drawn with the smoothed flow arrows.
	/// </summary>
	[Fact]
	public void Shop2Route1FlowArrowsAreDrawn()
	{
		string agdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = agdPath,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.Route);
		Assert.NotNull(haulBuilderResult.Grid);
		Assert.NotNull(haulBuilderResult.Flow);
		Pass pass = haulBuilderResult.Route.OrderedPasses[0];
		(double[,], double[,]) tuple = FlowRibbonSearch.SmoothedArrows(haulBuilderResult.Flow, haulBuilderResult.Grid);
		double num = 2.5908;
		byte[] array = Render.RenderSinglePairPng(haulBuilderResult.Grid, CenterlineTrack(pass, haulBuilderResult.Grid, num), num, 16, 2, ShowRoute: false, pass.CutCells, pass.FillCells, tuple.Item1, tuple.Item2);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-route1-haul"));
		Directory.CreateDirectory(fullPath);
		string text = Path.Combine(fullPath, "route-001.png");
		File.WriteAllBytes(text, array);
		Assert.True(array.Length > 1000, text);
	}

	/// <summary>
	/// The first hundred Shop 2 loads are drawn in found order.
	/// </summary>
	[Fact]
	public void Shop2FirstHundredRoutesAreDrawn()
	{
		string text = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
		HaulBuilderOptions haulBuilderOptions = new HaulBuilderOptions
		{
			AgdPath = text,
			SkipGuidance = true,
			IncludeGuidanceCsv = false
		};
		haulBuilderOptions.Optimization.EnableCycleChainOptimization = false;
		haulBuilderOptions.Optimization.PairRefinementRounds = 0;
		HaulBuilderResult haulBuilderResult = new HaulBuilderEngine().Run(haulBuilderOptions);
		Assert.NotNull(haulBuilderResult.Route);
		Assert.NotNull(haulBuilderResult.Grid);
		int num = Math.Min(100, haulBuilderResult.Route.OrderedPasses.Count);
		List<Pass> list = haulBuilderResult.Route.OrderedPasses.Take(num).ToList();
		Route route = new Route(Enumerable.Range(0, list.Count).ToList(), list, 0.0);
		AgdField field = Agd.Parse(text);
		GuidancePlanner.BuildResult buildResult = GuidancePlanner.Build(route, haulBuilderResult.Grid, field, 0.0, haulBuilderOptions);
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof", "shop2-first-100"));
		Directory.CreateDirectory(fullPath);
		double bladeWidthM = 2.5908;
		Dictionary<int, List<GuidancePoint>> dictionary = new Dictionary<int, List<GuidancePoint>>();
		foreach (GuidancePoint point in buildResult.Guidance.Points)
		{
			if (!dictionary.TryGetValue(point.PassNo, out var value))
			{
				value = new List<GuidancePoint>();
				dictionary[point.PassNo] = value;
			}
			value.Add(point);
		}
		List<string> list2 = new List<string>();
		for (int i = 0; i < list.Count; i++)
		{
			int num2 = i + 1;
			Pass pass = list[i];
			if (!dictionary.TryGetValue(num2, out var value2))
			{
				value2 = new List<GuidancePoint>();
			}
			File.WriteAllBytes(Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", num2)), Render.RenderSinglePairPng(haulBuilderResult.Grid, value2, bladeWidthM, 4, 2, ShowRoute: false, pass.CutCells, pass.FillCells));
			int num3 = Enumerable.Count(value2, (GuidancePoint Point) => Point.Action == "CUT" && Point.Blade == "DOWN");
			int num4 = Enumerable.Count(value2, (GuidancePoint Point) => Point.Action == "FILL" && Point.Blade == "DOWN");
			list2.Add(string.Format(CultureInfo.InvariantCulture, "route={0} load={1:F2} cutCells={2} fillCells={3} haul={4:F1} cutPts={5} fillPts={6}", num2, pass.LoadBcy, pass.CutCells.Count, pass.FillCells.Count, pass.HaulDistanceM, num3, num4));
			Assert.True(CellsTouch(pass.CutCells, EdgesOnly: false), "route " + num2.ToString(CultureInfo.InvariantCulture) + " cut is in separate pieces");
			Assert.True(CellsTouch(pass.FillCells, EdgesOnly: true), "route " + num2.ToString(CultureInfo.InvariantCulture) + " fill cells do not touch");
		}
		File.WriteAllLines(Path.Combine(fullPath, "index.txt"), list2);
		double num5 = haulBuilderOptions.ScraperCapacityLcy / haulBuilderOptions.SwellFactor;
		int num6 = 0;
		for (int num7 = 0; num7 < list.Count; num7++)
		{
			if (list[num7].LoadBcy >= num5 - 0.05)
			{
				num6++;
			}
		}
		Assert.Equal(num, list2.Count);
		Assert.Equal(num, num6);
	}

	/// <summary>
	/// Largest heading change between blade-down guidance points.
	/// </summary>
	private static double WorstBladeBendDeg(IReadOnlyList<GuidancePoint> Points, string Action)
	{
		double val = 0.0;
		List<GuidancePoint> list = new List<GuidancePoint>();
		int num = -2;
		for (int i = 0; i < Points.Count; i++)
		{
			bool flag = Points[i].Action == Action && Points[i].Blade == "DOWN";
			if (!flag || (list.Count > 0 && i != num + 1))
			{
				val = Math.Max(val, StrokeBendDeg(list));
				list.Clear();
			}
			if (flag)
			{
				list.Add(Points[i]);
				num = i;
			}
		}
		return Math.Max(val, StrokeBendDeg(list));
	}

	/// <summary>
	/// Length of one blade-down guidance stroke, in metres.
	/// </summary>
	private static double StrokePathM(IReadOnlyList<GuidancePoint> Points, string ActionBlade)
	{
		double num = 0.0;
		for (int i = 1; i < Points.Count; i++)
		{
			string text = Points[i].Action + ":" + Points[i].Blade;
			string text2 = Points[i - 1].Action + ":" + Points[i - 1].Blade;
			if (!(text != ActionBlade) && !(text2 != ActionBlade))
			{
				double num2 = Points[i].X - Points[i - 1].X;
				double num3 = Points[i].Y - Points[i - 1].Y;
				num += Math.Sqrt(num2 * num2 + num3 * num3);
			}
		}
		return num;
	}

	/// <summary>
	/// Heading change from the start of a stroke to its end.
	/// </summary>
	private static double StrokeBendDeg(List<GuidancePoint> Stroke)
	{
		double num = 0.0;
		for (int i = 1; i < Stroke.Count - 1; i++)
		{
			double num2 = Stroke[i].X - Stroke[i - 1].X;
			double num3 = Stroke[i].Y - Stroke[i - 1].Y;
			double num4 = Stroke[i + 1].X - Stroke[i].X;
			double num5 = Stroke[i + 1].Y - Stroke[i].Y;
			double num6 = Math.Sqrt(num2 * num2 + num3 * num3);
			double num7 = Math.Sqrt(num4 * num4 + num5 * num5);
			if (!(num6 <= 1E-06) && !(num7 <= 1E-06))
			{
				double val = (num2 * num4 + num3 * num5) / (num6 * num7);
				val = Math.Max(-1.0, Math.Min(1.0, val));
				double num8 = Math.Acos(val) * 180.0 / Math.PI;
				if (num8 > num)
				{
					num = num8;
				}
			}
		}
		return num;
	}

	/// <summary>
	/// Writes the ribbon proof images for a finished haul result.
	/// </summary>
	private static void WriteRibbonProof(HaulBuilderResult Result)
	{
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures", "FlowRibbonProof"));
		Directory.CreateDirectory(fullPath);
		double bladeWidthM = 2.5908;
		Grid grid = Result.Grid!;
		File.WriteAllBytes(Path.Combine(fullPath, "field-guidance.png"), Render.RenderGuidancePng(grid, Result.GuidancePoints, 8, bladeWidthM));
		List<int> list = (from result in Result.GuidancePoints.Select((GuidancePoint Point) => Point.PassNo).Distinct()
			orderby result
			select result).ToList();
		List<int> list2 = new List<int>();
		if (list.Count > 0)
		{
			int[] array = new int[5]
			{
				0,
				list.Count / 4,
				list.Count / 2,
				list.Count * 3 / 4,
				list.Count - 1
			};
			int[] array2 = array;
			foreach (int value in array2)
			{
				int item = list[Math.Clamp(value, 0, list.Count - 1)];
				if (!list2.Contains(item))
				{
					list2.Add(item);
				}
			}
		}
		foreach (int PassNo in list2)
		{
			List<GuidancePoint> points = Result.GuidancePoints.Where((GuidancePoint Point) => Point.PassNo == PassNo).ToList();
			File.WriteAllBytes(Path.Combine(fullPath, string.Format(CultureInfo.InvariantCulture, "route-{0:000}.png", PassNo)), Render.RenderSinglePairPng(grid, points, bladeWidthM, 10, 3, ShowRoute: false));
		}
	}

	/// <summary>
	/// Blade-down centerline for one load, cut stroke then fill stroke.
	/// </summary>
	private static List<GuidancePoint> CenterlineTrack(Pass Pass, Grid Grid, double BladeM)
	{
		List<GuidancePoint> list = new List<GuidancePoint>();
		AppendCenterline(list, CoveragePlanner.PlanRibbon(Pass.CutCells, Grid, BladeM), "CUT");
		AppendCenterline(list, CoveragePlanner.PlanRibbon(Pass.FillCells, Grid, BladeM), "FILL");
		return list;
	}

	/// <summary>
	/// Appends one planned ribbon onto the guidance track.
	/// </summary>
	private static void AppendCenterline(List<GuidancePoint> Points, CoveragePlanner.PlanResult Plan, string Action)
	{
		foreach (CoveragePlanner.Run run in Plan.Runs)
		{
			IReadOnlyList<(double, double)> readOnlyList = run.DrivePoints ?? new List<(double, double)> { run.StartXy, run.EndXy };
			if (readOnlyList.Count < 2)
			{
				continue;
			}
			if (Points.Count > 0)
			{
				(double, double) tuple = readOnlyList[0];
				Points.Add(new GuidancePoint(Points.Count + 1, 1, "MOVE", "UP", tuple.Item1, tuple.Item2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, string.Empty));
			}
			foreach (var item in readOnlyList)
			{
				Points.Add(new GuidancePoint(Points.Count + 1, 1, Action, "DOWN", item.Item1, item.Item2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, string.Empty));
			}
		}
	}

	/// <summary>
	/// Length and width of a cell strip, in cells.
	/// </summary>
	private static (double Length, double Width) StripExtent(List<((int Bx, int By) Cell, double DepthM)> Cells)
	{
		if (Cells.Count <= 1)
		{
			return (Length: Cells.Count, Width: Cells.Count);
		}
		double num = Cells[0].Cell.Bx;
		double num2 = Cells[0].Cell.By;
		double num3 = Cells[Cells.Count - 1].Cell.Bx;
		double num4 = Cells[Cells.Count - 1].Cell.By;
		double num5 = num3 - num;
		double num6 = num4 - num2;
		double num7 = Math.Sqrt(num5 * num5 + num6 * num6);
		if (num7 < 1.0)
		{
			return (Length: 1.0, Width: Math.Sqrt(Cells.Count));
		}
		double num8 = num5 / num7;
		double num9 = num6 / num7;
		double num10 = 0.0;
		double num11 = 0.0;
		foreach (var Cell in Cells)
		{
			double num12 = ((double)Cell.Cell.Bx - num) * (0.0 - num9) + ((double)Cell.Cell.By - num2) * num8;
			if (num12 < num10)
			{
				num10 = num12;
			}
			if (num12 > num11)
			{
				num11 = num12;
			}
		}
		return (Length: num7, Width: num11 - num10);
	}

	/// <summary>
	/// True when every cell in the strip meets the others through shared edges or corners.
	/// </summary>
	private static bool CellsTouch(List<((int Bx, int By) Cell, double DepthM)> Cells, bool EdgesOnly)
	{
		if (Cells.Count <= 1)
		{
			return true;
		}
		HashSet<(int, int)> hashSet = new HashSet<(int, int)>();
		foreach (var Cell in Cells)
		{
			hashSet.Add(Cell.Cell);
		}
		Stack<(int, int)> stack = new Stack<(int, int)>();
		(int, int) item = hashSet.First();
		stack.Push(item);
		hashSet.Remove(item);
		while (stack.Count > 0)
		{
			(int, int) tuple = stack.Pop();
			for (int i = -1; i <= 1; i++)
			{
				for (int j = -1; j <= 1; j++)
				{
					if ((j != 0 || i != 0) && (!EdgesOnly || j == 0 || i == 0))
					{
						(int, int) item2 = (tuple.Item1 + j, tuple.Item2 + i);
						if (hashSet.Remove(item2))
						{
							stack.Push(item2);
						}
					}
				}
			}
		}
		return hashSet.Count == 0;
	}

	/// <summary>
	/// A diagonal fill through a solid basin takes the cells between the corners.
	/// </summary>
	[Fact]
	public void DiagonalFillThroughASolidBasinSharesEdges()
	{
		double[,] array = new double[16, 24];
		double[,] target = new double[16, 24];
		bool[,] array2 = new bool[16, 24];
		double[,] array3 = new double[16, 24];
		double[,] array4 = new double[16, 24];
		double[,] phi = new double[16, 24];
		double[,] array5 = new double[16, 24];
		bool[,] array6 = new bool[16, 24];
		double num = 1.0 / Math.Sqrt(2.0);
		for (int i = 0; i < 16; i++)
		{
			for (int j = 0; j < 24; j++)
			{
				array2[i, j] = true;
				array6[i, j] = true;
				array5[i, j] = 1.0;
				array3[i, j] = num;
				array4[i, j] = 0.0 - num;
				if (j <= 4 && i >= 8)
				{
					array[i, j] = 0.2;
				}
				else if (j >= 8)
				{
					array[i, j] = -0.12;
				}
			}
		}
		double gridSizeM = 2.5908;
		Grid grid = new Grid(gridSizeM, 24, 16, 0.0, 0.0, 12, IsNorthern: true, 0.0, 0.0, array, target, array2, 1.3, 1.2, 61.0, new List<List<(double, double)>>());
		HaulWorkLedger ledger = HaulWorkLedger.Build(grid, 10.0);
		FlowField flow = new FlowField(phi, array3, array4, array5, array6, "test");
		double maximumLoadBcy = 15.384615384615383;
		HaulOptimizationResult haulOptimizationResult = FlowRibbonSearch.Optimize(grid, flow, ledger, HaulLaneBuilder.LaneAxis.UserHeading, maximumLoadBcy, DeferEdgeWork: false, null, AllowAnyDeferredRemainder: false, 2.5908);
		Assert.True(haulOptimizationResult.Validation.Success, string.Join("; ", haulOptimizationResult.Validation.Issues.Take(6)));
		Assert.NotEmpty(haulOptimizationResult.PassResult.Passes);
		Pass pass = haulOptimizationResult.PassResult.Passes[0];
		string text = string.Join(" ", pass.FillCells.Select((((int Bx, int By) Cell, double DepthM) Entry) => Entry.Cell.Bx.ToString(CultureInfo.InvariantCulture) + "," + Entry.Cell.By.ToString(CultureInfo.InvariantCulture)));
		Assert.True(CellsTouch(pass.FillCells, EdgesOnly: true), "fill " + pass.FillCells.Count.ToString(CultureInfo.InvariantCulture) + " bcy " + pass.LoadBcy.ToString("F2", CultureInfo.InvariantCulture) + " " + text);
	}

	/// <summary>
	/// Writes the 10 acre mound-and-basin field used by the ribbon checks.
	/// </summary>
	private static string WriteTenAcreAgd()
	{
		double num = Math.Sqrt(40468.564224);
		double num2 = 2.5908;
		int num3 = Math.Max(8, (int)Math.Round(num / num2));
		string fullPath = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "Fixtures"));
		Directory.CreateDirectory(fullPath);
		string text = Path.Combine(fullPath, "FlowRibbon10Acre.agd");
		using StreamWriter streamWriter = new StreamWriter(text);
		streamWriter.WriteLine("Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments");
		double sigma = (double)num3 * 0.22;
		double num4 = 0.1;
		for (int i = 0; i < num3; i++)
		{
			for (int j = 0; j < num3; j++)
			{
				double easting = 400000.0 + ((double)j + 0.5) * num2;
				double northing = 3700000.0 + ((double)i + 0.5) * num2;
				(double Lat, double Lon) tuple = Agd.LatlonFromUtm(easting, northing, 12, IsNorthern: true);
				double item = tuple.Lat;
				double item2 = tuple.Lon;
				double num5 = Gauss((double)j - (double)num3 * 0.28, (double)i - (double)num3 * 0.72, sigma);
				double num6 = Gauss((double)j - (double)num3 * 0.72, (double)i - (double)num3 * 0.28, sigma);
				double num7 = num4 * (num5 - num6);
				double num8 = 100.0 + num7;
				double num9 = 100.0;
				streamWriter.WriteLine(string.Format(CultureInfo.InvariantCulture, "{0:F8},{1:F8},{2:F4},{3:F4},{4:F4},3GRD,", item, item2, num8, num9, num9 - num8));
			}
		}
		return text;
	}

	/// <summary>
	/// Gaussian bump used to shape the ten-acre mound and basin.
	/// </summary>
	private static double Gauss(double Dx, double Dy, double Sigma)
	{
		return Math.Exp((0.0 - (Dx * Dx + Dy * Dy)) / (2.0 * Sigma * Sigma));
	}
}
