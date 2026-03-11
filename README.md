# OpenSMC

**A modular, composable MATLAB toolbox for Sliding Mode Control research and benchmarking.**

No equivalent open-source SMC toolbox exists. OpenSMC provides orthogonal, mix-and-match components — swap any sliding surface with any reaching law on any plant, and benchmark with standardized metrics.

## Quick Start

```matlab
% From the OpenSMC root directory
plant = plants.DoubleIntegrator('x0', [0; 0]);

ctrl = controllers.ClassicalSMC( ...
    surfaces.LinearSurface('c', 10), ...
    reaching.SuperTwisting('k1', 15, 'k2', 10));

arch = architectures.DirectSMC(ctrl);

runner = benchmark.BenchmarkRunner('dt', 1e-4, 'T', 5);
runner.add_architecture('SMC', arch);
runner.add_plant('DI', plant, ...
    utils.references.step_ref(1, 2), ...
    utils.disturbances.none(2));

results = runner.run_all();
runner.print_table(results);
```

## Architecture

```
+core/              6 abstract interfaces (SlidingSurface, ReachingLaw, Plant, Controller, Estimator, Architecture)
+surfaces/          9 sliding surfaces (Linear, Terminal, Nonsingular, Fast, Integral, Hierarchical, PID, Integral, NonlinearDamping)
+reaching/          5 reaching laws (Constant, Exponential, Power, SuperTwisting, Saturation)
+plants/            6 dynamical systems (DoubleIntegrator, InvertedPendulum, Cranes, Quadrotor6DOF, Nanopositioner)
+controllers/       8 controllers (Classical, Adaptive, Dynamic, 3x Hierarchical, ITSMC, NFTSMC)
+estimators/        6 estimators (None, DOB, ESO, HighGain, ICD, RBF-ELM)
+architectures/     2 composition patterns (Direct, Cascaded)
+benchmark/         Simulator (RK4/Euler) + 12 metrics + BenchmarkRunner
+utils/             Reference generators + disturbance profiles
examples/           7 worked examples
tests/              132 unit + integration tests
```

## Key Features

- **Composable**: any surface × any reaching law × any plant × any estimator
- **Benchmarkable**: 12 standardized metrics, automated comparison tables and plots
- **No dependencies**: pure MATLAB R2020b+, no toolboxes required
- **Tested**: 132 tests covering unit → integration levels
- **Research-ready**: includes controllers from 3 textbooks + 2 research manuscripts

## Components

### Sliding Surfaces
| Surface | Formula |
|---------|---------|
| Linear | `s = edot + c*e` |
| Terminal | `s = edot + β|e|^(p/q)sign(e)` |
| Nonsingular Terminal | `s = e + (1/β)|edot|^(q/p)sign(edot)` |
| Fast Terminal | `s = edot + αe + β|e|^(q/p)sign(e)` |
| Integral Terminal | `s = edot + c1*e + c2∫|e|^(p/q)sign(e)` |
| + 4 more | Hierarchical, Integral, PID, NonlinearDamping |

### Plants
| Plant | States | Inputs | Source |
|-------|--------|--------|--------|
| DoubleIntegrator | 2 | 1 | Canonical benchmark |
| InvertedPendulum | 4 | 1 | Underactuated |
| SinglePendulumCrane | 4 | 1 | Qian & Yi (2015) |
| DoublePendulumCrane | 6 | 1 | Qian & Yi (2015) |
| Quadrotor6DOF | 12 | 4 | ITSMC-ELM manuscript |
| DualStageNanopositioner | 4 | 1 | NFTSMC manuscript |

### Controllers
ClassicalSMC, AdaptiveSMC, DynamicSMC, AggregatedHSMC, IncrementalHSMC, CombiningHSMC, ITSMC (with RBF-ELM), NFTSMC (with feedforward)

## Examples

| Example | Description |
|---------|-------------|
| `example_quick_start.m` | 4 SMC variants on double integrator |
| `example_surface_swap.m` | 5 surfaces, same reaching law |
| `example_crane_hsmc.m` | 3 hierarchical SMC methods on crane |
| `example_itsmc_quadrotor.m` | ITSMC-ELM on 6-DOF quadrotor |
| `example_benchmark_crane.m` | Automated benchmark: 3 HSMC × 2 scenarios |
| `example_benchmark_quadrotor.m` | Automated benchmark: ITSMC × 2 scenarios |
| `example_nftsmc_nanopositioner.m` | NFTSMC vibration suppression on piezo stage |

## Running Tests

```matlab
cd D:/OpenSMC
run('tests/run_all_tests.m')
```

## References

- Liu, J. & Wang, X. (2013). *Advanced Sliding Mode Control for Mechanical Systems*. Springer.
- Qian, D. & Yi, J. (2015). *Hierarchical Sliding Mode Control for Under-actuated Cranes*. Springer.
- Bandyopadhyay, B. et al. (2009). *Sliding Mode Control Using Novel Sliding Surfaces*. Springer.

## License

MIT License. See [LICENSE](LICENSE).

## Author

**Ali Al Ghanimi** — University of Kufa, Iraq
