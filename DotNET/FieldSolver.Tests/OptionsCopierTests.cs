using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

using Xunit;

using HaulBuilder;

namespace FieldSolver.Tests
{
    /// <summary>
    /// Verifies that the options copy is complete and independent. The tests walk
    /// the options by reflection, so an option added to HaulBuilder later is
    /// covered without a change here.
    /// </summary>
    public sealed class OptionsCopierTests
    {
        /// <summary>Amount added to every number so it differs from its default.</summary>
        private const Int32 NUMBER_SHIFT = 3;

        /// <summary>Verifies every option value survives the copy when none is at its default.</summary>
        [Fact]
        public void Copy_OptionsWithNoDefaultLeft_CopiesEveryValue
            (
            )
        {
            HaulBuilderOptions Source = new HaulBuilderOptions();
            Int32 Changed = Scramble(Source);
            Source.ExcludePolygonSpecs.Add("1,2;3,4;5,6");
            Source.Ponding.OutletSpecs.Add("7,8");
            Source.Ponding.OutletPolygonSpecs.Add("9,10;11,12;13,14");

            HaulBuilderOptions Copy = OptionsCopier.Copy(Source);

            Assert.True(Changed > 0, "Scramble changed nothing.");
            Assert.True(AssertSameValues(Source, Copy, "Options") >= Changed, "Fewer values were compared than were changed.");
        }

        /// <summary>Verifies default options survive the copy, including null strings and null numbers.</summary>
        [Fact]
        public void Copy_DefaultOptions_CopiesEveryValue
            (
            )
        {
            HaulBuilderOptions Source = new HaulBuilderOptions();

            HaulBuilderOptions Copy = OptionsCopier.Copy(Source);

            Assert.True(AssertSameValues(Source, Copy, "Options") > 0, "Nothing was compared.");
        }

        /// <summary>Verifies the copy shares no option group and no list with its source.</summary>
        [Fact]
        public void Copy_Result_SharesNothingWithTheSource
            (
            )
        {
            HaulBuilderOptions Source = new HaulBuilderOptions();
            Source.ExcludePolygonSpecs.Add("1,2;3,4;5,6");
            Int32 SourceEdge = Source.Optimization.EdgeMaxLaneSlicesPerCycle;

            HaulBuilderOptions Copy = OptionsCopier.Copy(Source);

            Assert.NotSame(Source, Copy);
            Assert.NotSame(Source.Optimization, Copy.Optimization);
            Assert.NotSame(Source.Ponding, Copy.Ponding);
            Assert.NotSame(Source.ExcludePolygonSpecs, Copy.ExcludePolygonSpecs);
            Assert.NotSame(Source.Ponding.OutletSpecs, Copy.Ponding.OutletSpecs);
            // A change to the copy must not reach the source.
            Copy.ExcludePolygonSpecs.Clear();
            Copy.Optimization.EdgeMaxLaneSlicesPerCycle += 1;
            Assert.Single(Source.ExcludePolygonSpecs);
            Assert.Equal(SourceEdge, Source.Optimization.EdgeMaxLaneSlicesPerCycle);
        }

        /// <summary>Verifies a null source is refused.</summary>
        [Fact]
        public void Copy_NullSource_Throws
            (
            )
        {
            Assert.Throws<ArgumentNullException>(() => OptionsCopier.Copy(null!));
        }

        /// <summary>Gives every settable plain property of an object tree a value different from its default.</summary>
        /// <param name="Target">Options object or option group to change.</param>
        /// <returns>Number of properties changed.</returns>
        /// <exception cref="NotSupportedException">Thrown when a settable property has a type this helper does not know.</exception>
        static private Int32 Scramble
            (
            Object Target
            )
        {
            Int32 Changed = 0;
            foreach (PropertyInfo Property in Target.GetType().GetProperties(BindingFlags.Public | BindingFlags.Instance))
            {
                Type PropertyType = Property.PropertyType;
                Object? Value = Property.GetValue(Target);
                if ((Property.SetMethod == null) || (!Property.SetMethod.IsPublic))
                {
                    // Get-only option groups are scrambled member by member.
                    if ((Value != null) && PropertyType.IsClass && (PropertyType != typeof(String)) && (!(Value is List<String>)))
                    {
                        Changed += Scramble(Value);
                    }
                    continue;
                }
                Type Core = Nullable.GetUnderlyingType(PropertyType) ?? PropertyType;
                if (Core == typeof(Double)) Property.SetValue(Target, ((Double?)Value ?? 0.0) + NUMBER_SHIFT);
                else if (Core == typeof(Int32)) Property.SetValue(Target, ((Int32?)Value ?? 0) + NUMBER_SHIFT);
                else if (Core == typeof(Boolean)) Property.SetValue(Target, !((Boolean?)Value ?? false));
                else if (Core == typeof(String)) Property.SetValue(Target, ((String?)Value ?? String.Empty) + "x");
                else if (Core.IsEnum) Property.SetValue(Target, Enum.GetValues(Core).GetValue(Enum.GetValues(Core).Length - 1));
                else throw new NotSupportedException("Scramble does not know " + Property.Name);
                Changed += 1;
            }
            return Changed;
        }

        /// <summary>Asserts that two option trees hold equal values, property by property.</summary>
        /// <param name="Left">First object.</param>
        /// <param name="Right">Second object of the same type.</param>
        /// <param name="Path">Property path for failure messages.</param>
        /// <returns>Number of values compared.</returns>
        static private Int32 AssertSameValues
            (
            Object Left,
            Object Right,
            String Path
            )
        {
            Int32 Compared = 0;
            foreach (PropertyInfo Property in Left.GetType().GetProperties(BindingFlags.Public | BindingFlags.Instance))
            {
                Object? LeftValue = Property.GetValue(Left);
                Object? RightValue = Property.GetValue(Right);
                String Name = Path + "." + Property.Name;
                if ((LeftValue is List<String> LeftList) && (RightValue is List<String> RightList))
                {
                    Assert.True(LeftList.SequenceEqual(RightList), Name + " list differs.");
                    Compared += 1;
                }
                else if ((LeftValue != null) && Property.PropertyType.IsClass && (Property.PropertyType != typeof(String)))
                {
                    // An option group: compare its members.
                    Assert.True(RightValue != null, Name + " is missing in the copy.");
                    Compared += AssertSameValues(LeftValue, RightValue!, Name);
                }
                else
                {
                    Assert.True(Equals(LeftValue, RightValue), Name + " differs: " + LeftValue + " vs " + RightValue);
                    Compared += 1;
                }
            }
            return Compared;
        }
    }
}
