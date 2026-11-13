using System;
using System.Collections.Generic;
using System.Reflection;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// Copies a HaulBuilder options object. HaulBuilder has no public clone, so the
    /// copy walks the public properties. It refuses any property shape it does not
    /// understand, so a new kind of option fails loudly instead of being shared or
    /// dropped without notice.
    /// </summary>
    static internal class OptionsCopier
    {
        /// <summary>Makes an independent copy of every option, option group, and list.</summary>
        /// <param name="Source">Options to copy. Not modified.</param>
        /// <returns>A new options object with equal values and no shared lists.</returns>
        /// <exception cref="ArgumentNullException">Thrown when source is null.</exception>
        /// <exception cref="NotSupportedException">Thrown when an option has a shape this copier does not handle.</exception>
        static public HaulBuilderOptions Copy
            (
            HaulBuilderOptions Source
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            HaulBuilderOptions Target = new HaulBuilderOptions();
            CopyProperties(Source, Target);
            return Target;
        }

        /// <summary>Copies the public properties of one options object or option group.</summary>
        /// <param name="Source">Object to read.</param>
        /// <param name="Target">Object of the same type to write.</param>
        /// <exception cref="NotSupportedException">Thrown when a property has a shape this copier does not handle.</exception>
        static private void CopyProperties
            (
            Object Source,
            Object Target
            )
        {
            foreach (PropertyInfo Property in Source.GetType().GetProperties(BindingFlags.Public | BindingFlags.Instance))
            {
                // Indexers are not options.
                if (Property.GetIndexParameters().Length > 0) continue;
                Type PropertyType = Property.PropertyType;
                Object? Value = Property.GetValue(Source);
                Boolean HasPublicSetter = (Property.SetMethod != null) && Property.SetMethod.IsPublic;
                if (PropertyType.IsValueType || (PropertyType == typeof(String)))
                {
                    // Numbers, switches, enums, nullable numbers, and strings copy by value.
                    // A get-only one is computed from other options, so there is nothing to copy.
                    if (HasPublicSetter) Property.SetValue(Target, Value);
                }
                else if ((!HasPublicSetter) && (Value is List<String> SourceList))
                {
                    // Get-only lists belong to the options object: refill the target's own list.
                    List<String> TargetList = (List<String>)Property.GetValue(Target)!;
                    TargetList.Clear();
                    TargetList.AddRange(SourceList);
                }
                else if ((!HasPublicSetter) && (Value != null) && PropertyType.IsClass)
                {
                    // Get-only option groups (Optimization, Ponding) are copied member by member.
                    CopyProperties(Value, Property.GetValue(Target)!);
                }
                else
                {
                    // A settable object or an unknown member would be shared or lost.
                    throw new NotSupportedException(
                        "OptionsCopier cannot copy " + Source.GetType().Name + "." + Property.Name
                        + " of type " + PropertyType.Name + ".");
                }
            }
        }
    }
}
