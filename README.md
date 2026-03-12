# OpenSMC

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18965650.svg)](https://doi.org/10.5281/zenodo.18965650)

**A modular, composable MATLAB toolbox for Sliding Mode Control research and benchmarking.**

No equivalent open-source SMC toolbox exists. OpenSMC provides orthogonal, mix-and-match components — swap any sliding surface with any reaching law on any plant, and benchmark with standardized metrics. All implementations are written from scratch from published mathematical formulations — no code is copied from any existing repository.

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
+surfaces/          11 sliding surfaces (Linear, Terminal, Nonsingular, Fast, Integral, Hierarchical, PID, Integral, NonlinearDamping, Global, PredefinedTime)
+reaching/          5 reaching laws (Constant, Exponential, Power, SuperTwisting, Saturation)
+plants/            9 dynamical systems (DoubleIntegrator, InvertedPendulum, Cranes, Quadrotor6DOF, Nanopositioner, TwoLinkArm, PMSM, SurfaceVessel)
+controllers/       11 controllers (Classical, Adaptive, Dynamic, 3x Hierarchical, ITSMC, NFTSMC, Fuzzy, Discrete, FixedTime)
+estimators/        7 estimators (None, DOB, ESO, HighGain, ICD, RBF-ELM, LevantDifferentiator)
+architectures/     2 composition patterns (Direct, Cascaded)
+benchmark/         Simulator (RK4/Euler) + 12 metrics + BenchmarkRunner
+utils/             Reference generators + disturbance profiles
examples/           9 worked examples
tests/              170 unit + integration tests, 40 analytical validation tests
```

## Key Features

- **Composable**: any surface × any reaching law × any plant × any estimator
- **Benchmarkable**: 12 standardized metrics, automated comparison tables and plots
- **No dependencies**: pure MATLAB R2020b+, no toolboxes required
- **Tested**: 210 tests (170 unit/integration + 40 analytical validation)
- **Research-ready**: includes controllers from 3 textbooks + 2 research manuscripts + 6 seminal papers
- **Original code**: all implementations written from published math, not copied from existing repos

## Components

### Sliding Surfaces
| Surface | Formula | Source |
|---------|---------|--------|
| Linear | `s = edot + c*e` | Utkin (1977) |
| Terminal | `s = edot + β|e|^(p/q)sign(e)` | Yu & Zhihong (2002) |
| Nonsingular Terminal | `s = e + (1/β)|edot|^(q/p)sign(edot)` | Feng et al. (2002) |
| Fast Terminal | `s = edot + αe + β|e|^(q/p)sign(e)` | Feng et al. (2002) |
| Integral Terminal | `s = edot + c1*e + c2∫|e|^(p/q)sign(e)` | |
| Global | `s = edot + c*e - (edot₀+c*e₀)exp(-αt)` | Bartoszewicz (1998) |
| Predefined-Time | `s = edot + (π/2Tc)/cos(πt/2Tc) * e` | Sánchez-Torres et al. (2018) |
| + 4 more | Hierarchical, Integral, PID, NonlinearDamping | |

### Plants
| Plant | States | Inputs | Source |
|-------|--------|--------|--------|
| DoubleIntegrator | 2 | 1 | Canonical benchmark |
| InvertedPendulum | 4 | 1 | Underactuated |
| SinglePendulumCrane | 4 | 1 | Qian & Yi (2015) |
| DoublePendulumCrane | 6 | 1 | Qian & Yi (2015) |
| Quadrotor6DOF | 12 | 4 | ITSMC-ELM manuscript |
| DualStageNanopositioner | 4 | 1 | NFTSMC manuscript |
| TwoLinkArm | 4 | 2 | Slotine & Li (1991) |
| PMSM | 4 | 2 | Krause et al. (2013) |
| SurfaceVessel | 6 | 3 | Fossen (2011) |

### Controllers
ClassicalSMC, AdaptiveSMC, DynamicSMC, AggregatedHSMC, IncrementalHSMC, CombiningHSMC, ITSMC (with RBF-ELM), NFTSMC (with feedforward), FuzzySMC (Mamdani inference, no toolbox), DiscreteSMC (Gao 1995), FixedTimeSMC (Polyakov 2012)

### Estimators
NoEstimator, DisturbanceObserver, ExtendedStateObserver, HighGainObserver, IntegralChainDifferentiator, RBF_ELM, LevantDifferentiator (arbitrary-order HOSM, Levant 2003)

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
| `example_robot_arm_benchmark.m` | Classical vs Fuzzy vs FixedTime SMC on 2-link arm |
| `example_predefined_time.m` | Linear vs Global vs PredefinedTime surfaces |

## Reproducibility

All code listings in the [SoftwareX paper](docs/softwarex_paper.pdf) are verbatim excerpts from the example files in this repository. To reproduce the paper's benchmark tables:

```matlab
cd D:/OpenSMC/examples
example_surface_swap           % → Table 2 (surface swap results)
example_benchmark_crane        % → Table 3 (HSMC crane results)
example_itsmc_quadrotor        % → Table 4 (quadrotor results)
```

## Running Tests

```matlab
cd D:/OpenSMC
run('tests/run_all_tests.m')           % 170 unit + integration tests
run('tests/validate_all_new_modules.m') % 40 analytical validation tests
```

## References

### Textbooks
- Liu, J. & Wang, X. (2013). *Advanced Sliding Mode Control for Mechanical Systems*. Springer.
- Qian, D. & Yi, J. (2015). *Hierarchical Sliding Mode Control for Under-actuated Cranes*. Springer.
- Bandyopadhyay, B. et al. (2009). *Sliding Mode Control Using Novel Sliding Surfaces*. Springer.
- Slotine, J.-J. E. & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall.
- Fossen, T. I. (2011). *Handbook of Marine Craft Hydrodynamics and Motion Control*. Wiley.
- Krause, P. C. et al. (2013). *Analysis of Electric Machinery and Drive Systems*. 3rd ed., Wiley.

### Seminal Papers
- Polyakov, A. (2012). Fixed-time stabilization of linear control systems. *IEEE TAC*, 57(8), 2106-2110.
- Gao, W. et al. (1995). Discrete-time variable structure control systems. *IEEE TIE*, 42(2), 117-122.
- Levant, A. (2003). Higher-order sliding modes, differentiation and output-feedback control. *IJC*, 76(9-10), 924-941.
- Sánchez-Torres, J. D. et al. (2018). Predefined-time stability of dynamical systems. *Math. Probl. Eng.*
- Bartoszewicz, A. (1998). Discrete-time quasi-sliding-mode control strategies. *IEEE TIE*, 45(4), 633-637.
- Khanesar, M. A. et al. (2021). Fuzzy sliding mode control. In *Fuzzy Logic in Its 50th Year*. Springer.

## License

MIT License. See [LICENSE](LICENSE).

## Author

**Ali Al Ghanimi** — University of Kufa, Iraq
