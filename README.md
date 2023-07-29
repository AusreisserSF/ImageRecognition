7/29/2023 Archiving this project. It was an attempt at a season-independent, general-purpose image recognition project. But each new season introduced backwards compatibility problems so now there is one IJxxxVision project (e.g. IJPowerPlayVision) for each season. 

Stand-alone IntelliJ IDEA project that
recognizes the ring stacks in the FTC
2020 - 2021 game Ultimate Goal.

Includes a port to Java of the C++ ring
recognition prototype in the project
OpenCVTestbed2.

Uses Android XML parsing (non-schema) but
IntelliJ Java 14 LocalDateTime.

The project is actually mis-named: it can
be used to test any XML processing in
IntelliJ before it is ported to Android
Studio (debugging is easier) and also the
Java implementation of any image recognition
(for the same reason).
