# Contributing

Issues and pull requests are welcome, and most of all from SolidCraft's maintainers. A Springcraft
maintainer reviews every pull request, because the suite's results come only from commits reviewed
as pass-through.

## What belongs here

- **Corrections.** A place where the adapter drives solidcore wrongly, or calls something other than
  the kernel's one call for a function.
- **New calls.** A function the adapter lists as unsupported that a newer SolidCraft release now
  offers in one call.

What does not belong: work the kernel does not do itself. The adapter is pass-through, as the
suite's
[contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
asks. A function assembled here out of several kernel calls would score this code and not solidcore,
so it stays unsupported.

A problem with a test, a check or the protocol belongs in
[brep-conformance](https://github.com/springcraft-ai/brep-conformance/issues). A problem in
solidcore itself belongs with solidcore.

## Licence of contributions

Unless you explicitly state otherwise, any contribution you intentionally submit for inclusion
in this repository, as defined in the Apache License, Version 2.0, is licensed under both the
MIT licence and the Apache License, Version 2.0, as the [README](README.md#licences) describes,
without any additional terms or conditions.

solidcore has a licence of its own, which the [README](README.md#licences) describes. Call the
kernel; do not copy its source, or any other project's, into this repository.
