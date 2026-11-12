# Third-party notices

The signed dihedral gradients in `src/physics/constraints.ts` are adapted from
`solve_DihedralConstraint` in [PositionBasedDynamics](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/blob/master/PositionBasedDynamics/PositionBasedDynamics.cpp).
This implementation uses a signed `atan2` angle and XPBD compliance instead of the original PBD stiffness projection.

## PositionBasedDynamics

The MIT License (MIT)

Copyright (c) 2015-present, PositionBasedDynamics contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Other dependencies and references

- Three.js: MIT; its complete license is included at `licenses/three-LICENSE`.
- Vite, TypeScript, tsx, and type definitions are development dependencies; their licenses ship with their npm packages. TypeScript is Apache-2.0; Vite and tsx are MIT.
- Origami Simulator was investigated as a design reference for triangulated bar-and-hinge origami. No code from Origami Simulator is included.
- XPBD equations are implemented from Macklin, Müller, and Chentanez (2016), linked in README.md.

## Plane-library references

The nine procedural sheet recipes and their exported FOLD geometry are independently authored Paper Flight studies, covered by the project MIT license. Fold ’N Fly is cited as an instructional reference; its photographs, printable templates and text are not bundled. These are not downloaded Fold ’N Fly assets. See [the source and reuse table](docs/plane-library.md) for verified URLs, candidate assets and unsupported designs. The Origami Simulator SVG and FOLD example assets investigated for this task are not included.
