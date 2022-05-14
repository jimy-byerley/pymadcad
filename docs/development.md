# Development guidelines


Some details about the architecture design, the code style, and the link of tools used.

## The overal design guidelines:

- The code in the library can be splitted in 2 categories:  
  + The code exposed to the user (which is meant to be easy to use, easy to combine, and *stable*, meaning it must not change its signature)
  + The internal code, only meant for use by the other functions in the library. It can be modified, redesigned or removed at any time
- The library module (pymadcad) contains many submodules dedicated to specific operations / topics, and the user is expected to import the specific submodules to pick the functions he wants. however the most common functions/classes of the library are also reexposed by the root module, so the user need to pick from specific submodules only for very specific functions.
- There is an implicit notion of *module level* in the library, which designate the rank of the module in the dependency tree: as a guideline, modules can rely on other modules but there should not be dependency loops.
- There should be one only, or at least one only obvious way to do each thing
- Implicit can be prefered over explicit when it simplifies the API and when there is no ambiguity on what implicit behavior is expected in the given context. This is extremely useful for instance to wrap different implementations in a single overloaded function when the operation is the same in concept but requires different implementations according to the input data type. Implicit choice should however never be dependent on a global variable, user setting, or any "global" context.
- As often as possible, there should be default values to the function parameters exposed to the user, allowing the functions with many parameters to be used for the most common usage without requiring a long study of the documentation and choices from the part of the user.
- No use-case specific features. All the functions provided by this library shall use generic data structures, generic algorithms accepting any data as input as long as they are of the right type. There could be limitations defined by the implemented algorithms, but those restrictions of input must be mathematically defined in the documentation. In particular, there should not be operation-specific data structures, like in some API where before calling any function, you must convert your raw data in the function's own data structure and convert it back as the function returns.
- As an API designer, use the architecture classics and design patterns as needed. There is no need to create abstract classes when you just need a class; no need to use a class when you could use a tuple, no need to create properties when you could use a member. Remember that both over-achitecturing and under-architecturing are decreasing the maintainability, extensibility and makes the code harder to read. An architect has to choose between both extremes.
- pymadcad is designed to follow the preceding guidelines and by taking advantage of the modern python concepts like ducktyping, dynamic typing, polymorphism, buffering, and so many others. Any future development shall be made to keep the API consistent and sound. Hopefully that should not be that difficult.
- Follow the RAII paradigm (Ressource Allocation Is Initialization), Specifically, stateful classes and factory classes are usually bad and unsuited to perform computations.
- A class might own its data, or not, but it should always be clear in the docs when it owns it or not. For instance classes like `Mesh` are only referencing buffers, that are often shared with many more instances of `Mesh`, `Web` or `Wire`
- also check out [python's zen](https://peps.python.org/pep-0020/)

## Style guidelines

- use snake_case for functions/variables and CamelCase for class names (unless they are just structures or very common classes like  `list` or `vec3`)

- use sound names for both functions, classes and even their arguments. The names should be as short as possible but with no ambiguity between them: don't use synonyms nor abreviations, but keywords

- split your code into elementary functions when it is possible (well, when it is possible without decreasing the performance. Some algorithms cannot be splitted freely, in particular when they share too much data between each of their sections)

- keep the code readable, from a human perspective (no need to use a specific formatter, nor clamp to 80 columns, you are a human, you decide what is easily readable and what is less)

- indent with tabs

