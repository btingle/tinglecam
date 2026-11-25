# FreeCAD adaptive solver

`Adaptive.cpp` and `Adaptive.hpp` are from FreeCAD commit
`9623213a59dd3cda20fb573f8295691af23b6b68`, at
https://github.com/FreeCAD/FreeCAD/tree/9623213a59dd3cda20fb573f8295691af23b6b68/src/Mod/CAM/libarea

Copyright Kresimir Tusek and FreeCAD contributors. LGPL-2.1-or-later;
see the original headers and COPYING.LIB. These sources are distributed with
TingleCAM so builds do not download all of FreeCAD or require Python/FreeCAD.

Local changes: add the missing functional header; make development performance
counters thread-local; suppress normal tool/region status prints.
The algorithm itself is unchanged. `bridge.*` is TingleCAM's integration API.

The target privately compiles Clipper2 with USINGZ and the namespace renamed to
FreeCADClipper2Lib. No Clipper types cross the bridge: TingleCAM's XY Clipper
and FreeCAD's internally tagged XYZ Clipper must never share an ABI.
