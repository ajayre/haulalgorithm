using System;
using System.Collections.Generic;
using System.IO;

using NDesk.Options;

using HaulBuilder;

using FieldSolver;

namespace FieldSolverCli
{
    /// <summary>
    /// The command line of FieldSolver.exe. The field file is the input. The
    /// eight tuning values are what the solver finds, so they are not switches.
    /// Uses <see cref="OptionSet"/> from NDesk.Options.
    /// </summary>
    internal sealed class CommandLine
    {
        /// <summary>Gets the AGD field file, from <c>--field</c> or the positional argument.</summary>
        internal String? FieldPath { get; private set; }

        /// <summary>Gets whether <c>-h</c> or <c>--help</c> was passed.</summary>
        internal Boolean ShowHelp { get; private set; }

        /// <summary>Gets the parse error, or null when the command line can be run.</summary>
        internal String? Error { get; private set; }

        /// <summary>The option set, kept so help can print the same switches that were parsed.</summary>
        private OptionSet? _Options;

        /// <summary>Field path passed with <c>--field</c>, before it is combined with the positional path.</summary>
        private String? _FieldSwitch;

        /// <summary>Field path from the positional argument, before it is combined with <c>--field</c>.</summary>
        private String? _PositionalField;

        /// <summary>Reads the arguments.</summary>
        /// <param name="Args">Arguments from <c>Main</c>, without the program name.</param>
        /// <returns>The parsed line. <see cref="Error"/> is set when it cannot be run.</returns>
        /// <exception cref="ArgumentNullException">Thrown when args is null.</exception>
        static internal CommandLine Parse
            (
            String[] Args
            )
        {
            if (Args == null) throw new ArgumentNullException(nameof(Args));
            CommandLine Line = new CommandLine();
            Line.BuildOptions();
            try
            {
                List<String> Extra = Line._Options!.Parse(Args);
                Line.TakePositionalField(Extra);
                Line.ResolveField();
            }
            catch (OptionException Ex)
            {
                // Help can still be printed when the failure came after -h.
                if (Line.Error == null) Line.Error = Ex.Message;
            }
            return Line;
        }

        /// <summary>Prints the usage line and every switch.</summary>
        /// <param name="Writer">Where the text goes, normally standard output.</param>
        /// <exception cref="ArgumentNullException">Thrown when writer is null.</exception>
        static internal void WriteHelp
            (
            TextWriter Writer
            )
        {
            if (Writer == null) throw new ArgumentNullException(nameof(Writer));
            CommandLine Line = new CommandLine();
            Line.BuildOptions();
            Writer.WriteLine("FieldSolver.exe finds the HaulBuilder settings with the shortest whole-job driving distance for one field.");
            Writer.WriteLine();
            Writer.WriteLine("Usage: FieldSolver.exe [options] <field.agd>");
            Writer.WriteLine();
            Writer.WriteLine("The field file is the input. The solver chooses the eight tuning values and prints the best ones.");
            Writer.WriteLine("Ctrl+C stops the search and prints the runs that finished.");
            Writer.WriteLine();
            Line._Options!.WriteOptionDescriptions(Writer);
        }

        /// <summary>Builds a standard-search request for the field. The solver fills in the eight tuning values.</summary>
        /// <returns>A request the solver can run.</returns>
        /// <exception cref="InvalidOperationException">Thrown when the command line has an error or no field.</exception>
        internal FieldSolverRequest CreateRequest
            (
            )
        {
            if (Error != null) throw new InvalidOperationException(Error);
            if (String.IsNullOrWhiteSpace(FieldPath)) throw new InvalidOperationException("A field file is required.");
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = FieldPath,
            };
            // The constructor installs the standard search. Nothing on the command line picks the eight values.
            return new FieldSolverRequest(Options);
        }

        /// <summary>Registers the field switch and help.</summary>
        private void BuildOptions
            (
            )
        {
            _Options = new OptionSet()
            {
                { "i|field=", "the {FILE} to solve. May instead be the one positional argument.", TakeField },
                { "h|help", "print every option and exit.", TakeHelp },
            };
        }

        /// <summary>Records the field path passed with <c>--field</c>.</summary>
        /// <param name="Text">The path.</param>
        private void TakeField
            (
            String Text
            )
        {
            if (String.IsNullOrWhiteSpace(Text))
            {
                throw new OptionException("field needs a value.", "field");
            }
            _FieldSwitch = Text.Trim();
        }

        /// <summary>Records <c>-h</c> or <c>--help</c>.</summary>
        /// <param name="Text">Unused. NDesk passes the switch name when the flag is present.</param>
        private void TakeHelp
            (
            String Text
            )
        {
            // NDesk calls this only when the flag is present. The value is the switch name.
            ShowHelp = (Text == null) || !String.IsNullOrEmpty(Text);
        }

        /// <summary>Takes the one positional argument as the field file.</summary>
        /// <param name="Extra">Arguments NDesk did not recognize as switches.</param>
        private void TakePositionalField
            (
            List<String> Extra
            )
        {
            foreach (String Item in Extra)
            {
                // NDesk leaves an unregistered switch in the extra list. It is not a field path.
                if (Item.StartsWith("-", StringComparison.Ordinal))
                {
                    Error = "Unknown option: " + Item;
                    return;
                }
                if (_PositionalField != null)
                {
                    Error = "Only one field file is allowed.";
                    return;
                }
                _PositionalField = Item;
            }
        }

        /// <summary>Combines <c>--field</c> and the positional path into <see cref="FieldPath"/>.</summary>
        private void ResolveField
            (
            )
        {
            if (Error != null) return;
            if ((_FieldSwitch != null) && (_PositionalField != null))
            {
                Error = "Pass the field file once, either as --field or as the positional argument.";
                return;
            }
            FieldPath = _FieldSwitch ?? _PositionalField;
            // Help does not need a field. Every other command does.
            if ((!ShowHelp) && String.IsNullOrWhiteSpace(FieldPath))
            {
                Error = "A field file is required.";
            }
        }
    }
}
