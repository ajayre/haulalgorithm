# C# coding style guide
# Version 1

Personal rules for formatting and structuring C#. The emphasis is readability in large WinForms and class-library projects, practical consistency with classic .NET Framework patterns where they apply, and alignment with Visual Studio's XML documentation tooling.

**Numbering.** Normative bullets use **`section.subsection.rule`**. Nested lines under one bullet (**`2.1.4`–`2.1.9`** beneath **`2.1.3`**, **`5.1.2`–`5.1.6`** beneath **`5.1.1`**, **`3.2.1`–`3.2.4`** beneath **`3.2`**) reuse the **`section.subsection`** with the next **`rule`** index—the hierarchy is lexical, not a fourth digit. Subsection **`0`** is material under a `##` section before any `###` subsection (for example **`3.0.1`**). Numbered headings **`### N.M`** use matching subsection **`M`** (for example rules under **`### 3.2`** start at **`3.2.1`**).

---

## 1. Environment and tooling

- **1.1.1 Indentation.** Use **4 spaces** per level. Do not rely on literal tab characters for structural indentation.
- **1.1.2 Line endings.** Must match **project policy** — Windows-heavy work **must** use CRLF unless team rules deliberately specify otherwise.
- **1.1.3 Visual Studio IntelliSense docs.** Use the built-in **`///` XML documentation** snippets (summary, parameters, returns, type parameters, exceptions where useful). Maintain the usual tag order: `<summary>` first, then `<typeparam>` / `<param>` / `<returns>` / `<remarks>` / `<exception>` as needed.

---

## 2. Namespaces and file layout

- **2.1.1 Block namespace.** Use a **traditional block namespace** wrapping the whole file:

```csharp
namespace Example.Product.Ui
{
    public class WindowCoordinator
    {
        ...
    }
}
```

- **2.1.2 One primary type per file.** **One primary type per file** is the norm. **Allowed in the same compilation unit:** additional **`partial`** declarations of that type (including multiple **`partial`** blocks or **`#region`**-folded companions in one **`.cs`**), tightly coupled **nested** types, and designer splits (**`Foo.cs`**, **`Foo.Designer.cs`**). **Do not** place unrelated top-level types in the same file.
- **2.1.3 `using` placement.** **`using` directives** appear at the top, after optional file-leading comments:
  - **2.1.4** `System*` namespaces grouped together.
  - **2.1.5** Blank line.
  - **2.1.6** External / vendor assemblies.
  - **2.1.7** Blank line.
  - **2.1.8** First-party / application namespaces (company, product, feature areas).
  - **2.1.9 No `#region` around `using`s.** Do **not** wrap **`using`** directives inside **`#region` / `#endregion`**: they hide what the file depends on, fight tool defaults, and add noise for negligible folding benefit. **`global using`** (where used) likewise stays **outside** regions.

## 3. Naming conventions

- **3.0.1** These rules define the preferred naming model.

### 3.1 Types and members

| § | Element | Convention | Examples |
|---|--------|---------------|----------------------|
| **3.1.1** | Types (classes, structs, enums, delegates) | `PascalCase` | `LedgerEntry`, `WindowCoordinator`, `NetworkOptions` |
| **3.1.2** | Interfaces | `I` + `PascalCase` | `IDeepCopy<T>`, `IPlugin`, `ITimestampFactory` |
| **3.1.3** | Public properties, methods, events | `PascalCase` | `Open`, `ScanTimeout`, `ScanTimeoutMs` |
| **3.1.4** | Enum members | `PascalCase` | `AccessKind.ReadWrite` |
| **3.1.5** | **Instance fields used as property backing, locks, or internals** | **`_` + PascalCase** (`_Title`, `_Gate`, `_EntryName`)—**not** `_camelCase` (`_title`, `_gate`) | `_Title`, `_Gate`, `_LastError` |
| **3.1.6** | **`static public` helpers** | `PascalCase` method names | `AttributeHelper.ReadDescription`, `LedgerEntry.ParseKind` |
| **3.1.7** | **`const`** / **`readonly` singular constants** (**§§10.1.2–10.1.3**; not aggregates—**§10.1.5**) | **`UPPER_SNAKE`** (`A_Z` digits; words separated by `_`) | `MAX_NODES`, `SKU_FLAVOR`, `MAGIC_COOKIE` |
| **3.1.8** | **`readonly`/static aggregates**—arrays, lists, dictionaries, collection interfaces, buffered views (**§10.1.5**) | `PascalCase` | `SeedInts`, `DefaultRowTemplate`, `PinnedNodes` |

### 3.2 Locals and parameters (distinctive style)

- **3.2.1 Locals / parameters casing.** **Locals and parameters must be `PascalCase`**, treating them similarly to exported API names (`FormCount`, `OpenedForm`, `NewForm`, `Item`, `Index`, `Window`). **Never** use **`camelCase`** or **`snake_case`** for first-party locals—**except** the **`for`** index carve-out in **§3.2.4**. **`foreach`** iteration variables remain **`PascalCase`** (**`Pane`**, not **`pane`**). **Do not** confuse this with **§3.1.5** backing fields: those use a **leading `_`** plus **PascalCase** (**`_Title`**, not **`_title`**). Services, UI code, and shared libraries **must** follow this same rule.
- **3.2.2 Handler parameter names.** **Event handler parameters** in XML docs use lower camelCase placeholders where you follow VS defaults (`sender`, `e`); otherwise parameter names mirror the PascalCase convention where you chose descriptive names (`FormName`, `AddFormFunction`).
- **3.2.3 Delegate snapshot locals.** When an event or delegate is copied to a local before **`?.Invoke`** (or a classic null guard), the local **must** be **`PascalCase`** and **should** end with **`Handler`** when it holds a delegate typed as a `*Handler` or event backing field—for example **`LedgerChangedHandler? Handler = LedgerSummaryChanged`**, **`CapabilityShiftHandler? Handler = CapabilityPulse`**—**not** **`handler`**, **`h`**, or **`del`**.
- **3.2.4 `for` loop counters.** **Strongly prefer** a **meaningful `PascalCase`** counter that states what is advancing—**`Iteration`**, **`Iter`**, **`Row`**, **`Step`**, **`Pane`**—**`for (Int32 Iter = 0; Iter < Iterations; Iter += SWEEP_SIMULATION_STRIDE)`**, not opaque **`i`**/**`I`**, when the loop bound names a domain quantity (iterations, rows, nodes, etc.). Reserve **single-letter** counters (**`i`**, **`j`**, **`k`**) for **tight inner loops** where a longer name adds no intent; those letters **must** be **lowercase** (**`i`**, not **`I`**). **`foreach`** variables follow **§3.2.1** (**`PascalCase`** always).

### 3.3 Events and delegates

- **3.3.1 Delegate naming.** **Delegates** describing handlers **must** use **`PascalCase`**. Naming **must** either end with **`Handler`** **or** **must** be a concise **`PascalCase`** phrase naming the invocation shape (`OnItemAddedHandler`, `ConnectStatusChange`, `ChangedHandler`).
- **3.3.2 Event naming.** **New first-party events** use an **`On` prefix** when the name is not fixed by a framework or binding contract (**`OnStatusChange`**, **`OnScanComplete`**). **Retain** established names required by **BCL**, **`INotifyPropertyChanged`**, and similar surfaces (**`PropertyChanged`**, **`CollectionChanged`**)—do **not** rename those to **`OnPropertyChanged`** merely to satisfy the prefix rule.
- **3.3.3 Event invocation guard.** On **new** projects, use null-conditional **`Invoke`** on the backing delegate: **`Changed?.Invoke()`** when it is parameterless, or **`Changed?.Invoke(this, arguments);`** (parameters **must** match the delegate, for example **`Changed?.Invoke(this, e);`** for standard **`EventHandler`**-shaped signatures). Prefer copying the event/delegate to a **`PascalCase`** local first (**§3.2.3**), then **`Handler?.Invoke(...)`**. In **existing** projects, match the invocation style already established in that codebase (including the classic guard `if (Changed != null) { Changed(...); }`).

### 3.4 Type aliases (`string` vs `String`, `int` vs `Int32`)

- **3.4.1 Primitive alias consistency.** **Mixed** CLR type spellings (`string`/`String`, `uint`/`UInt32`) appear in mature codebases. For new work, prefer **one convention per file** (or solution-wide) to reduce noise; when editing older files, follow the prevailing spellings there.

---

## 4. Bracing and control flow

- **4.1.1 Allman braces.** **Braces follow Allman ("BSD") style**: opening `{` on its own line for types and methods.

```csharp
public class CircularBuffer<T>
{
    public virtual void Write
        (
        T Item
        )
    {
        ...
    }
}
```

- **4.1.2 Conditional / switch braces.** **`if` / `else` / `switch`**: opening brace on a new line; `case` labels indent one level inside `switch`.

```csharp
if (RetryCount > 0)
{
    ProcessPending();
}
else
{
    ClearQueue();
}

switch (ConnectionState)
{
    case ConnectionStates.Disconnected:
        AttemptReconnect();
        break;
    case ConnectionStates.Connected:
        SendHeartbeat();
        break;
    default:
        HandleUnexpectedState();
        break;
}
```

- **4.1.3 Single-branch bracing.** **Compact single-branch statements** must omit braces **only when** the body is a single simple statement **such as** an early `return`:

```csharp
if (NewForm == null) return null;

if (IsEmpty()) return Item;
```

- **4.1.4 Multi-statement branches.** Prefer adding braces when the branch grows beyond one line or when mixing multiple statements.

---

## 5. Method and constructor signatures ("split parenthesis" style)

- **5.1.1 Layout intent.** Methods, constructors, and similar constructs frequently use this layout. **Code figures in §5** illustrate **split-parenthesis layout only**; they use C# keyword aliases (**`string`**, **`int`**, **`byte?`**, etc.) as shown. **Production** signatures still use **PascalCase** parameters (**§3.2.1**) and obey **§3.4** for type spelling in each file.
  - **5.1.2** Return type / method name **or** type name for constructors on one line (or wrapped consistently).
  - **5.1.3** Opening `(` on the next line, **indented once (4 spaces)** from the enclosing scope.
  - **5.1.4** Each parameter on its own line, indented one extra level inside the parentheses.
  - **5.1.5** Closing `)` on its own line, aligned with `(`.
  - **5.1.6** Opening `{` for the body on the next line.

```csharp
public ToolWindow Open
    (
    string FormName,
    AddForm AddFormFunction,
    int MaxForms,
    XElement Settings,
    byte? NodeId
    )
{
    ...
}

public CircularBuffer(int Capacity)
{
    ...
}

public DocsAttribute
    (
    string HelpText
    )
{
    this.HelpText = HelpText;
}
```

- **5.1.7 Parameterless signature form.** **Parameterless APIs** repeat the pattern with an empty line between parentheses:

```csharp
public void CloseAll
    (
    )
{
    ...
}

static LookupTables
    (
    )
{
    ...
}
```

- **5.1.8 Rationale.** Use this layout for clarity on long parameter lists and to keep signatures visually aligned.

---

## 6. Access modifiers keyword order

- **6.1.1 Static members.** Use **`static public`** for **static** methods, fields, properties, and events in first-party code—not **`public static`** (**`static public readonly`**, **`static public void Reset()`**, **`static public Int32 Version => …`**). **§18** vendor or legacy embeds may keep foreign **`public static`** spellings untouched.

---

## 7. Classes, structs, partial types

- **7.1.1 `partial`.** **`partial`** is used for WinForms designers and designer-generated counterparts.
- **7.1.2 Nested types.** **Nested types** (`public enum AccessKind` inside `LedgerEntry`) are acceptable when tightly coupled.
- **7.1.3 Enumeration member alignment.** In handwritten **`enum`** declarations, every member that assigns a value with **`=`** must place all **`=`** tokens in **one vertical column**. Pad shorter enumerator names with spaces so **`=`** begins **exactly one ASCII space** after the **last character of the longest name** in that enum. Apply the same alignment for literals, **`1u << n`**, and composed values such as **`Capture | Emit`**. Enums with **only implicit** numbering (no **`=`** on any member) omit this layout until each member uses an explicit **`=`**.

```csharp
public enum LedgerMode : byte
{
    Idle   = 0,
    Busy   = 1,
    Failed = 2,
}

public enum CapabilityFlags : uint
{
    None           = 0,
    Capture        = 1u << 0,
    Emit           = 1u << 1,
    CaptureAndEmit = Capture | Emit,
}
```

---

## 8. Documentation comments (`///`)

- **8.0.1 XML docs scope.** Use **XML documentation** for APIs consumed across assemblies or shown in IntelliSense. **Every** handwritten **method, constructor, operator, indexer accessor body treated as a member, and `partial` method** (declaration **and** implementing part when split) **must** carry **`/// <summary>`** per **§8.0.7**—do **not** treat **§8.0.1** as permission to skip “small” or **private** members.

```csharp
/// <summary>
/// Opens a form
/// </summary>
/// <param name="FormName">Name of the form to open</param>
/// <param name="MaxForms">Maximum number of this type of form allowed</param>
/// <returns>Opened form or null for error</returns>
public ToolWindow Open(...);
```

- **8.0.2 Generic type parameters.** **`/// <typeparam name="T">`** when documenting generic APIs.
- **8.0.3 Return documentation.** **`<returns>`**: describe sentinel values (`null`, default struct) explicitly when they carry meaning ("Opened form or null for error").
- **8.0.4 Public events.** Public **events** **must** have a **`/// <summary>`** immediately above the declaration when they are part of the maintained first-party surface—the same bar as **§8.0.7** for other members.
- **8.0.5 Non-XML comments (`//` only).** Supplement **`///`** with **`//`** line comments for implementation notes (algorithm, protocol details, UI behavior). For longer prose, use **multiple consecutive `//` lines**. **Do not** use C# **delimited comments** (**`/* ... */`**, including single-line **`/* … */`**): they complicate nesting, merges, and "is this commented-out code?" reviews. **Sections 17–18** still allow leaving foreign or generator-owned text untouched if upstream uses **`/* */`**.
- **8.0.6 Type-declaration comments.** Every first-party **`class`**, **`interface`**, and **`enum`** (including **nested** ones you maintain) **must** have **`///`** documentation with a **`/// <summary>`** that states **what the type is** and **what role it plays** in the product—enough context for IntelliSense readers who do not yet know nearby call sites. **Each `partial class` (or `partial struct`) declaration**—including **companion partials** in other regions or files—**must** carry its **own** **`/// <summary>`** describing **that** fragment’s responsibility (for example “implements partial method bodies for …”), **not** only the primary partial beside the main inheritance list. **`struct`** declarations follow the **same requirement** when they remain handwritten first-party types. Typical shape: identity plus responsibilities (lifetime, collaborators, threading, or major invariants **when** unusual). Relax **this rule** **only** for **sections 17–18** files where comments are tooling- or vendor-owned.
- **8.0.7 Every method and constructor.** **Every** first-party **method** and **constructor** you maintain **must** have a **`/// <summary>`** immediately above the member—**including** **`static` constructors**, **instance constructors**, **private** helpers, **explicit interface implementations**, **`partial` method** declarations, and **`partial` method** bodies in companion partials. Add **`<param>`**, **`<returns>`**, **`<typeparam>`**, and **`<exception>`** when the signature carries meaning (**§§8.0.2–8.0.3**). **No** “obvious one-liner” exemption: if it is a **function-shaped** member, it is **documented**.
- **8.0.8 Intention comments inside methods.** Handwritten **method, constructor, accessor, and `partial` method bodies** **must** include **`//`** comments that explain **why** or **in what order**—**policy, threading, protocol, performance, or business intent**—not **what** the next line literally does. **§8.0.7** covers the member header; **§8.0.8** covers **inside the braces**. **Good:** `// Snapshot the delegate so raises remain stable if a handler unsubscribes mid-invoke.` **Bad:** `// Assign Title to _Title.` **Add** comments at **branch points**, **locks**, **non-obvious constants/masks**, **delegate/event raises**, and **multi-step algorithms**; **omit** a line only when any honest comment would merely repeat the identifier names (a single trivial assignment may rely on **`///`** alone).

### 8.1 Domain-specific attributes

- **8.1.1** The **`[Help("...")]`** attribute carries user-facing or tool-consumed descriptions for many public model types and members.
- **8.1.2** The **`[Prototype("...")]`** attribute documents example construction text where used.
- **8.1.3 Attribute upkeep.** Prefer keeping these in sync when changing public surface area.

---

## 9. Properties and fields

- **9.1.1 `INotifyPropertyChanged`.** **Observable / binding-heavy models** **must** implement **`INotifyPropertyChanged`** with a helper **`Set<T>(String PropertyName, ref T Field, T Value)`** (or equivalent **PascalCase** parameter names) centralizing change notification.
- **9.1.2 Explicit properties.** **Simple automatic properties** are rare for mutable domain data; **explicit getters/setters** with backing fields are common.
- **9.1.3 Public fields on DTOs.** **Public fields** sometimes appear on DTO-style types; prefer consistency within each layer (data transfer vs. encapsulated services).

---

## 10. Constants and `readonly`

- **10.1.1 `const` / `readonly` roles.** **`const`** for compile-time literals; **`static readonly`** for collections exposed as read-only facades (`ReadOnlyCollection` wrapping private static lists).
- **10.1.2 `const` naming.** Compile-time literals named **`const`** **must** use **`UPPER_SNAKE`**: all uppercase ASCII letters (**`A`–`Z`**), digits when needed, with underscore separators between words (**`MAGIC_COOKIE`**, **`MAX_NODES`**, **`VERSIONSUFFIX`**)—not PascalCase (**`MagicCookie`**).
- **10.1.3 `readonly` constants (singular values).** When **`const`** is impossible but the field denotes **exactly one fixed configuration token after initialization** (SKU string, timeout **`TimeSpan`**, opaque scalar sentinel, singleton hash knob), **`static`** or **`instance`** **`readonly`** **must** use **`UPPER_SNAKE`** (**`SKU_FLAVOR`**). Naming **tabular or sequential state** (**§10.1.5**) overrides this casing rule. Collaborators (locks, services, **`IDisposable`**) keep **§§3.1.5–3.2**.
- **10.1.4 No magic numbers.** Do **not** leave **unexplained numeric literals** in handwritten logic (**limits**, **timeouts**, **delays**, **masks**, **moduli**, **repeat counts**, **version ordinals** not already expressed as **`enum`** members, **exclusive upper bounds**, **non-trivial bitwise patterns**): bind them to **`private const`** (**`UPPER_SNAKE`**, **§10.1.2**) or, when **`const`** is illegal, **`static readonly`** (**§10.1.3**) with a **name that states purpose or unit**. **Narrow carve-outs**: **`0`**, **`1`**, **`-1`** as idiomatic emptiness, loop start, or arity in the immediate statement (for example **`for (Int32 i = 0;`** …, **`Iterations <= 0`**, clearing a numeric field such as **`Revision = 0L`**), unary **`++`/`--`**, **`enum`** explicit underlying values (**`Idle = 0`**), **`1u << n`** for **[Flags]** with **`0 ≤ n < 32`**, **framework named constants** (**`Int32.MaxValue`**), **`default`/`sizeof`/`nameof`**, and indexer **`[0]`** when **`0`** means "first element"—**unless** the same literal repeats or its meaning spans lines, in which case **name it anyway**.
- **10.1.5 Arrays, collections, lists, aggregates.** **`readonly`** fields (**including **`static readonly`**) whose declared type is an **aggregate of elements** (**`T[]`**, **`System.Collections`** list/map/set/`Dictionary`-style surfaces, **`IReadOnly*`** wrappers such as **`ReadOnlyCollection<T>`**, **`ImmutableArray`/`ImmutableList`/`FrozenDictionary`**-style blobs, **`Span`/`ReadOnlySpan`/`Memory`/`ReadOnlyMemory`/`ArraySegment`**, **`IEnumerable`/`IQueryable`** retained as snapshots) **`must`** use **`PascalCase`** (**`SeedInts`**, **§3.1.8**), **`not`** **`UPPER_SNAKE`**, because **`UPPER_SNAKE`** (**§§10.1.2–10.1.3**) names **singular invariant tokens**, while **collection fields name tables of mixed or runtime literals**—**`PascalCase`** applies even when **`read-only` façades** hide mutation. **Literal scalars inside **`Add`/initializer lists **`still`** obey **`§10.1.4`** (**`private const`** first).

---

## 11. Preprocessor and conditional compilation

- **11.1.1 Conditional compilation use.** Heavy use of **`#if` / `#elif` / `#endif`** for product SKUs and features (STANDARD, LITE, PROFESSIONAL, ULTIMATE, etc.).
- **11.1.2 Divergence.** Keep conditional regions **minimally divergent** to reduce merge pain; document any symbol requirements at the top of the module if non-obvious.

---

## 12. `#region`

- **12.1.1 Regions.** Use **`#region` / `#endregion`** to fold large repeating blocks (for example massive static initialization tables, property-change helpers in bindings). Name regions in **Title Case With Spaces** ("Default Object Dictionary", "Notify Property Changed"). **Never** wrap **`using`** directives—**§2.1.9**.

---

## 13. Threading and resources

- **13.1.1 Locks.** **`lock (this)`** appears in some concurrency-sensitive types. Prefer **`private readonly Object _Gate = new Object();`** and **`lock (_Gate)`** in new designs (**§3.1.5**); retain **`this`** locking only where it matches surrounding code.

---

## 14. Exceptions and resilience

- **14.1.1 Empty `catch`.** **Empty catches** (`catch { }`) may be used on non-fatal UI cleanup paths. Prefer at least a comment explaining intentional swallowing when adding similar code.
- **14.1.2 Nullable context.** Nullability **must** be enforced with **explicit checks** compatible with Classic .NET Framework code. **`#nullable` pragmas must not** appear **unless** the **solution already enables** nullable reference types **project-wide**.

---

## 15. Language features

- **15.1.1 `var`.** **Do not** use **`var`** in first-party code. **Every** local, **`foreach`** iteration variable, and similar declaration **must** use an explicit type name on the page (`ToolWindow OpenedForm = ...`, `foreach (ToolWindow Pane in DockedPages)`). When a type name is long, prefer a file- or namespace-scoped **`using` alias** or break the line rather than hiding the type behind **`var`**.
- **15.1.2 Modern syntax breadth.** **`?.` null-conditional**, **`nameof`**, **`$"..."` interpolation**, and LINQ vary by file age and layer; newer UI code may use richer C# syntax than core data types.
- **15.1.3 Pattern matching.** **Pattern matching**: use where it simplifies logic without hurting readability for contributors used to explicit casts (`as` + null check patterns appear frequently).

### 15.2 Logical operators (`&&`, `||`)

- **15.2.1 Goal.** Make compound Boolean conditions obvious **without** relying on C# **operator precedence** in your head—parentheses **`(...)`** document grouping so **`&&`**, **`||`**, **`&`**, **`|`**, comparisons, shifts, bitwise operations, arithmetic, **`as`/`is`/`??`/`await` mixes**, **`?.`/indexer/`new` mixes**, unary combinations, nested **`&&`/`||`**, and similar constructs **never** fight for precedence on the page.

- **15.2.2 `&&` / `||` operand parentheses.** Parenthesize **each operand** of **`&&`** and **`||`** when that operand is anything **other** than what **15.2.3** allows as a lone, unwrapped expression. Comparisons mixed with arithmetic, bitwise mixes (**`&`**, **`|`**, **`^`**, **`~`** with relational logic), non-trivial member-access chains (**`?.`**, **`[]`**) unless the operand is essentially one **invoke-shaped** atom, unary **`~`**, casts, **`!`** applied to something **other** than **15.2.3(b)**, **`await`** in subexpressions, nested **`&&`/`||`** as part of a larger condition, **`is`/`as`/`switch` expressions**, **`??`**, **`?:`**, etc., **must** sit inside **`(...)`** so grouping is plain.

- **15.2.3 When no extra parentheses are needed.** Omit an outer **`(...)`** wrapper around an operand of **`&&`** or **`||`** **only when** **the entire operand** is:

    - **(a)** A **method invocation** or **delegate invocation**, including **`obj.Callee(...)`** and **`Type.Static(...)`**.
    - **(b)** **`!`** placed **directly before** **(a)** with no other unary between them—for example **`!string.IsNullOrEmpty(Value)`** or **`!int.TryParse(s, out Parsed)`**. Unary **`!`** binds to that call alone; **`(!Invocation(...))`** is redundant.
    - **(c)** A **built-in unary form** that parses like a single call-shaped atom—for example **`nameof(x)`**, **`sizeof(T)`** (where permitted), or **`default(SomeType)`**, **only when that single expression is the entire operand.** When unsure, wrap in **`(...)`**; **`stackalloc`** or any operand that stitches multiple operators together **requires** parentheses.
    - **(d)** **Postfix or prefix `++` / `--`** on one **simple storage location** (**`Depth++`**, **`counts[Index]++`**, **`--RetryRemaining`**). Parentheses mandated by grammar (including **unsafe** dereferences needed for postfix on a **`ptr->`/`*(ptr)`** target) obey the compiler—they **do not count** as "optional stylistic wrapping." Do **not** add a redundant **extra** **`(...)`** solely around **`Depth++`**-style operands: prefer **`if ((Ready != 0) && Depth++)`** over **`if ((Ready != 0) && (Depth++))`** unless the outer pair is genuinely needed for a different reviewer-facing reason.

- **15.2.4 Mixing `&` and `|` with `&&` and `||`.** **`&`** and **`|`** are multiplicative-relative on **`int`/enum** and behave as Boolean **non**-short-circuit operators on **`bool`**. When combining them with **`&&`**, **`||`**, and comparisons (**bitmasks**, **flags**, **mixed primitives**), parentheses **must** make short-circuit versus eager evaluation obvious—use **`(...)`** liberally, **including inside** operands of **`&&`** and **`||`**.

---

## 16. String formatting

- **16.1.1 `String.Format`.** **`String.Format` with `{0}` placeholders** remains common alongside concatenation.

---

## 17. Designers and generated files

- **17.1.1 Designer sources.** **Do not hand-edit designer-generated `.Designer.cs` files** except through the designer or known safe mechanical edits—keep style dictated by tooling there.
- **17.1.2 Designer companion file names.** **Naming of designer companions**: Prefer consistent `*.Designer.cs` casing within a project where possible (historically some dialogs used `designer.cs`; new files should follow the dominant casing in that project).

---

## 18. Third-party and legacy exceptions

First-party rules elsewhere in this guide **do not** override foreign or vendor-owned source **except** where you deliberately adopt those files under your ownership. Typical patterns:

| § | Situation | Guidance |
|---|-----------|-----------|
| **18.1.1** | **Vendor / package surface** (published SDK snippets, interop stubs (`.cs`), analyzer or tool stubs, pinned NuGet source you must not wholesale rewrite, official samples checked in verbatim) | **Do not** mass-reformat vendor types or rename their symbols **to satisfy this guide**—surface their API honestly. Prefer **calling or wrapping from first‑party code** (facades, adapters, extensions in your namespaces, narrow partials **only when** the tooling permits safe layering) rather than rewriting their layout or identifiers. |
| **18.1.2** | **Embedded vendored `*.cs`** (forked demos, `ThirdParty/**` subtrees, git submodules of upstream C#, vendor-owned glue) | Comparable to adopting foreign C sources: indentation, bracing (**K&R** or compact variants), **`snake_case`** / **`Library_Prefix`** names, and **`camelCase`** locals **may** all differ from first-party norms. **Preserve** them **unless** the team executes a deliberate, scoped refactor. Apply this guide fully **after** those files become consciously owned and formatted. |
| **18.1.3** | **Whitespace-only churn** | **Do not** open large purely cosmetic diffs **(re-indent / rebrace only)**. When editing a vendor or legacy file for real changes, inside **edited regions only** adopt **whatever** indentation and brace depth that **file already uses**—including **two-space** indents common in OSS drops—rather than widening the change into a full conversion to **§1.1.1** (**four-space** first-party norms). |

Cross-reference **§17** if the embedded code is generator-owned (prefer not editing at all).

---

## 19. Checklist before commit (AI verification)

Treat **§§1–18** as **normative**; **§19** only **verifies** them. Use **§19.1** as the per-patch sweep—**do not** treat **§19** as the only place rules are defined.

### 19.1 Per-patch verification checklist

**Path and scope**

- **19.1.1** Classify the path: first-party (**§§1–17**) vs vendor/embed (**§18**) vs designer-generated (**§17**). Apply **§18.1.3**—no whitespace-only style churn on foreign trees.

**Layout and structure**

- **19.1.2** **Four-space** indentation (**§1.1.1**); **CRLF** if project policy requires (**§1.1.2**). **Block namespace** (**§2.1.1**). **`using`** groups and blank lines (**§2.1.3–2.1.8**); **no `#region` around `using`** (**§2.1.9**). **`#region`** names and placement (**§12**).

**Signatures and bracing**

- **19.1.3** **Allman** bracing (**§4**); compact single-branch only where **§4.1.3** allows. **Split-parentheses** on every touched constructor/method signature (**§5**), including **parameterless**, **single-parameter**, **nested**, and **`static` constructors** (**§5.1.7**).

**Naming**

- **19.1.4** **PascalCase** types, members, locals, and parameters (**§§3.1–3.2**). **`_PascalCase`** backing/lock fields (**`_Title`**, **`_Gate`**—**§§3.1.5, 9, 13**), not **`_camelCase`**. **`Handler`** (or typed name) for delegate snapshot locals (**§3.2.3**). **`for`** counters: **meaningful `PascalCase` preferred** (**`Iter`**—**§3.2.4**); else **lowercase** **`i`/`j`/`k`** only in tight inner loops. **`UPPER_SNAKE`** for **`const`** and singular **`readonly`** constants (**§§3.1.7, 10.1.2–10.1.3**); **`PascalCase`** for aggregate **`readonly`** fields (**§§3.1.8, 10.1.5**). **`string`/`String`** consistency per file (**§3.4**). **`static public`** on static members (**§6**), not **`public static`**.

**Types, enums, constants**

- **19.1.5** **`/// <summary>`** on every touched **`class`**, **`interface`**, **`enum`**, **`struct`**, and **each `partial` declaration** (**§8.0.6**). **`enum`** explicit **`=`** values **column-aligned** (**§7.1.3**). No unexplained numeric literals—**`private const`** names (**§10.1.4**). **`#if`** regions stay coherent (**§11**).

**Documentation**

- **19.1.6** **`/// <summary>`** on **every** touched method/constructor/**`partial` method** (declaration and implementing part)—**§8.0.7**; **`<param>`**, **`<returns>`**, **`<typeparam>`**, **`<exception>`** when meaningful (**§§8.0.2–8.0.3**, **§1.1.3** tag order). **`///`** on **public events** touched (**§8.0.4**). **`[Help]` / `[Prototype]`** in sync (**§8.1**). In-body **`//`** explains **intent**, not narration (**§8.0.8**). **No `/* … */`** (**§8.0.5**).

**Members and behavior**

- **19.1.7** **Delegates/events**: naming (**§3.3.1–3.3.2**); **`Handler?.Invoke`** pattern (**§§3.2.3, 3.3.3**). **Properties**: explicit backing where applicable (**§9**); **`INotifyPropertyChanged`** pattern when binding-heavy (**§9.1.1**). **Locks**: prefer **`_Gate`**, not **`lock(this)`**, in new code (**§13**).

**Language and logic**

- **19.1.8** **No `var`**; explicit types on locals and **`foreach`** (**§15.1.1**). **`&&`/`||`** operand parentheses (**§15.2**). **`#nullable` pragmas** only if solution-wide (**§14.1.2**). Empty **`catch`** documented (**§14.1.1**).

**Designer / vendor**

- **19.1.9** **`.Designer.cs`**: tooling-owned—**§17** (no casual hand-edits). **Vendor/embed**: minimal, in-region conformance only (**§§18.1.1–18.1.3**).

