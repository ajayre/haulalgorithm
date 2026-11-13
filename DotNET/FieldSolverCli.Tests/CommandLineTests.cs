using System;
using System.IO;

using Xunit;

using FieldSolver;

using FieldSolverCli;

namespace FieldSolverCli.Tests
{
    /// <summary>
    /// Verifies the FieldSolver.exe switches. Nothing here starts the engine.
    /// </summary>
    public sealed class CommandLineTests
    {
        /// <summary>A path that is not opened. The search falls back when it cannot be read.</summary>
        private const String FIELD = "shop.agd";

        /// <summary>Verifies that <c>-h</c> asks for help and does not require a field.</summary>
        [Fact]
        public void Parse_ShortHelp_SetsShowHelp
            (
            )
        {
            CommandLine Line = CommandLine.Parse(new String[] { "-h" });

            Assert.True(Line.ShowHelp);
            Assert.Null(Line.Error);
        }

        /// <summary>Verifies that <c>--help</c> asks for help and does not require a field.</summary>
        [Fact]
        public void Parse_LongHelp_SetsShowHelp
            (
            )
        {
            CommandLine Line = CommandLine.Parse(new String[] { "--help" });

            Assert.True(Line.ShowHelp);
            Assert.Null(Line.Error);
        }

        /// <summary>Verifies that help lists the field and help, and does not take the eight tuning values.</summary>
        [Fact]
        public void WriteHelp_ListsTheInputOptions
            (
            )
        {
            using (StringWriter Writer = new StringWriter())
            {
                CommandLine.WriteHelp(Writer);
                String Text = Writer.ToString();

                Assert.Contains("--field", Text);
                Assert.Contains("--help", Text);
                Assert.Contains("-h", Text);
                Assert.DoesNotContain("--edge", Text);
                Assert.DoesNotContain("--minwork", Text);
                Assert.DoesNotContain("--axis", Text);
                Assert.DoesNotContain("--flow", Text);
                Assert.DoesNotContain("--slices", Text);
                Assert.DoesNotContain("--window", Text);
                Assert.DoesNotContain("--exchange", Text);
                Assert.DoesNotContain("--chain", Text);
            }
        }

        /// <summary>Verifies that a field alone starts the standard search, which is what finds the eight values.</summary>
        [Fact]
        public void Parse_FieldOnly_KeepsTheStandardSearch
            (
            )
        {
            CommandLine Line = CommandLine.Parse(new String[] { FIELD });
            FieldSolverRequest Request = Line.CreateRequest();

            Assert.Null(Line.Error);
            Assert.Equal(FIELD, Request.BaseOptions.AgdPath);
            Assert.Equal(2, Request.SearchSpace.EdgeMaxLaneSlicesPerCycleValues.Count);
            Assert.Equal(2, Request.SearchSpace.MinimumWorkLengthFtValues.Count);
            Assert.Equal(2, Request.SearchSpace.LaneAxisHeadingDegValues.Count);
        }

        /// <summary>Verifies that one of the eight tuning values is rejected, because the solver finds it.</summary>
        [Fact]
        public void Parse_TuningSwitch_IsAnError
            (
            )
        {
            CommandLine Line = CommandLine.Parse(new String[] { "--edge", "40", FIELD });

            Assert.Contains("Unknown option", Line.Error);
        }

        /// <summary>Verifies that a missing field is reported.</summary>
        [Fact]
        public void Parse_MissingField_IsAnError
            (
            )
        {
            CommandLine Line = CommandLine.Parse(new String[0]);

            Assert.False(Line.ShowHelp);
            Assert.NotNull(Line.Error);
        }

        /// <summary>Verifies that an unknown switch is rejected.</summary>
        [Fact]
        public void Parse_UnknownSwitch_IsAnError
            (
            )
        {
            CommandLine Line = CommandLine.Parse(new String[] { "--nope", FIELD });

            Assert.Contains("Unknown option", Line.Error);
        }

        /// <summary>Verifies that the field cannot be passed both as a switch and as a positional argument.</summary>
        [Fact]
        public void Parse_FieldGivenTwice_IsAnError
            (
            )
        {
            CommandLine Line = CommandLine.Parse(new String[] { "--field", FIELD, "other.agd" });

            Assert.NotNull(Line.Error);
        }
    }
}
