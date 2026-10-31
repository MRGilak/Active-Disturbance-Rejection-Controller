# MATLAB Coder ADRC

The original `adrc.ADRC` class remains the reference MATLAB implementation.
`adrc_codegen.m` is a fixed-size numeric copy of its runtime algorithm. It
supports orders 1--4, up to four cascaded observers, saturation, input delay,
reference derivatives, and all tracking differentiator modes.

From MATLAB, generate both a MEX implementation and a C library:

```matlab
cd matlab/codegen
build_adrc_mex
addpath('generated')
controller = ADRCMex(2);
controller.initialize('Tsettle', 1, 'dT', 0.01, 'b0', 1);
u = controller.step(1, 0);
```

Compare the generated controller with the original MATLAB implementation:

```matlab
compare_adrc_implementations
```

The comparison uses a disturbed second-order integrator, prints maximum and
RMS control differences, measures both controller loops, prints the MEX
speedup, and plots output and control trajectories. Because the benchmark
calls the MEX entry point once per controller step, the timing includes
MATLAB/MEX call overhead.

`ADRCMex` intentionally mirrors the frequently used portion of the MATLAB
class API. The generated entry point uses fixed-size inputs and a numeric
action code because MATLAB Coder cannot compile the original handle class,
`inputParser`, and variable argument interface as a portable C entry point.
The C library is emitted under `matlab/codegen/generated/c`.

The codegen core allocates a delay history for at most 1000 controller steps.
An input delay above that limit is rejected explicitly. Up to 16
`ADRCMex` objects can be used independently in one MATLAB session.
