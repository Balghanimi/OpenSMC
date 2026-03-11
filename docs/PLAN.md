# OpenSMC — Master Plan

> **Created:** 2026-03-11
> **Updated:** 2026-03-11
> **Owner:** Ali Al Ghanimi
> **Location:** `D:/OpenSMC/`
> **Status:** Phase 0 ✅ | Phase 1 partial (3/4 books) | Phase 2 ✅ (ITSMC + NFTSMC done) | Tests ✅

---

## Vision

The first comprehensive, modular, open-source sliding mode control toolbox. No equivalent exists — the closest is `XernicRose/basic_sliding_mode_controller_lib` (15 variants, 35 stars, Simulink-only, dead). OpenSMC will be the `do-mpc` of sliding mode control.

---

## Current Inventory (62 MATLAB files, ~5,800 LOC)

### Core Interfaces (6)
- [x] `SlidingSurface` — `s = f(e, edot, eint, t)`
- [x] `ReachingLaw` — `u_r = g(s, t)`
- [x] `Plant` — `xdot = dynamics(t, x, u, d)`
- [x] `Controller` — `u = ueq + ur - dhat`
- [x] `Estimator` — `dhat = estimate(t, x, u, y)`
- [x] `Architecture` — Direct / Cascaded / Hierarchical composition

### Sliding Surfaces (9)
- [x] `LinearSurface` — `s = edot + c*e`
- [x] `TerminalSurface` — `s = edot + beta*|e|^(p/q)*sign(e)`
- [x] `NonsingularTerminalSurface` — `s = e + (1/beta)*|edot|^(q/p)*sign(edot)`
- [x] `FastTerminalSurface` — `s = edot + alpha*e + beta*|e|^(q/p)*sign(e)`
- [x] `IntegralTerminalSurface` — `s = edot + c1*e + c2*integral(|e|^(p/q)*sign(e))`
- [x] `HierarchicalSurface` — `S = s_actuated + lambda*s_unactuated`
- [x] `IntegralSlidingSurface` — `s = C*(e - E)`, eliminates reaching phase (Qian App C)
- [x] `PIDSurface` — `s = alpha*edot + beta*e + gamma*eint` (Qian App E)
- [x] `NonlinearDampingSurface` — `s = edot + (c + Psi(y))*e` (Bandyopadhyay Ch 2)

### Reaching Laws (5)
- [x] `ConstantRate` — `-k*sign(s)`
- [x] `ExponentialRate` — `-k*sign(s) - q*s`
- [x] `PowerRate` — `-k*|s|^alpha*sign(s)`
- [x] `SuperTwisting` — `-k1*|s|^(1/2)*sign(s) + integral(-k2*sign(s))`
- [x] `Saturation` — `-k*sat(s/phi)`

### Plants (6)
- [x] `DoubleIntegrator` — 2-state, 1-input (canonical benchmark)
- [x] `InvertedPendulum` — 4-state, 1-input (underactuated)
- [x] `SinglePendulumCrane` — 4-state, 1-input (Qian Ch 2-3, with f1/f2/b1/b2 decomposition)
- [x] `DoublePendulumCrane` — 6-state, 1-input (Qian Ch 2-3, mass matrix formulation)
- [x] `Quadrotor6DOF` — 12-state, 4-input (Newton-Euler, from ITSMC-ELM manuscript)
- [x] `DualStageNanopositioner` — 4-state, 1-input (2-mode piezo, from NFTSMC manuscript)

### Controllers (8)
- [x] `ClassicalSMC` — general (any surface + any reaching law)
- [x] `AdaptiveSMC` — adaptive gain with projection
- [x] `DynamicSMC` — dynamic sliding variable σ = ṡ + λs
- [x] `AggregatedHSMC` — two-layer hierarchical (Qian Sec 4.2)
- [x] `IncrementalHSMC` — three-layer with sign-switching (Qian Sec 4.3)
- [x] `CombiningHSMC` — intermediate variable approach (Qian Sec 4.4)
- [x] `ITSMC` — integral terminal SMC + RBF-ELM estimation
- [x] `NFTSMC` — nonsingular fast terminal SMC + feedforward + integral action

### Estimators (6)
- [x] `NoEstimator` — zero (pure SMC switching)
- [x] `DisturbanceObserver` — slow time-varying DOB (Liu Ch 8)
- [x] `ExtendedStateObserver` — ADRC-style ESO (Liu Ch 8)
- [x] `HighGainObserver` — high-gain observer
- [x] `IntegralChainDifferentiator` — exact differentiator
- [x] `RBF_ELM` — RBF-ELM neural network (OS-ELM online update)

### Architectures (2)
- [x] `DirectSMC` — single controller, fully actuated
- [x] `CascadedSMC` — outer ITSMC + control allocation + inner PD (quadrotor)

### Benchmark (3)
- [x] `Simulator` — fixed-step RK4/Euler closed-loop engine
- [x] `Metrics` — 12 metrics (RMSE, MAE, ISE, IAE, settling, overshoot, effort, chattering, reaching, SS error, max_swing, residual_swing)
- [x] `BenchmarkRunner` — run all arch × plant combinations, table + plot

### Utilities (2)
- [x] `references` — step, ramp, sinusoidal, multi-step, transport, circular_3d
- [x] `disturbances` — none, step, sinusoidal, random_band, composite

### Examples (7)
- [x] `example_quick_start.m` — 4 SMC variants on double integrator
- [x] `example_surface_swap.m` — 5 surfaces, same reaching law
- [x] `example_crane_hsmc.m` — 3 HSMC methods on crane (manual loop)
- [x] `example_itsmc_quadrotor.m` — ITSMC-ELM on 6-DOF quadrotor
- [x] `example_benchmark_crane.m` — BenchmarkRunner: 3 HSMC × 2 scenarios
- [x] `example_benchmark_quadrotor.m` — BenchmarkRunner: ITSMC × 2 scenarios
- [x] `example_nftsmc_nanopositioner.m` — NFTSMC vibration suppression on piezo stage

### Tests (8 files, 132 test methods)
- [x] `test_surfaces.m` — 20 tests for all 9 surfaces
- [x] `test_reaching.m` — 19 tests for all 5 reaching laws
- [x] `test_plants.m` — 33 tests for all 6 plants
- [x] `test_controllers.m` — 15 tests for all 8 controllers
- [x] `test_estimators.m` — 15 tests for all estimators
- [x] `test_benchmark.m` — 17 tests for framework + utils
- [x] `test_integration.m` — 13 full pipeline tests
- [x] `run_all_tests.m` — test runner script

---

## What Is Left

### Phase 1: Code Extraction from Books (3/4 done)

#### ✅ Source 1: Liu & Wang (2013) — "Advanced SMC: MATLAB Simulation"
- Extracted: AdaptiveSMC, DynamicSMC, FastTerminalSurface, DisturbanceObserver, ESO, HighGainObserver, IntegralChainDifferentiator
- **Yield:** 1 surface, 2 controllers, 4 estimators

#### ✅ Source 2: Qian & Yi (2015) — "Hierarchical SMC for Cranes"
- Extracted: SinglePendulumCrane, DoublePendulumCrane, AggregatedHSMC, IncrementalHSMC, CombiningHSMC, IntegralSlidingSurface, PIDSurface
- **Yield:** 2 plants, 3 controllers, 2 surfaces

#### ✅ Source 3: Bandyopadhyay (2009) — "SMC Using Novel Sliding Surfaces"
- Extracted: NonlinearDampingSurface
- **Yield:** 1 surface

#### ❌ Source 4: Ben (2013) — "SMC and Observation" (369 pages)
- [ ] Extract sliding mode observer (SMO)
- [ ] Implement `+estimators/SlidingModeObserver.m`
- [ ] Implement `+architectures/ObserverBasedSMC.m`
- **Expected yield:** 1 estimator, 1 architecture

### Phase 2: Port Existing Code (2/2 done ✅)

#### ✅ ITSMC-ELM Quadrotor Manuscript
- Ported: RBF_ELM, ITSMC, Quadrotor6DOF, CascadedSMC
- Example: example_itsmc_quadrotor.m

#### ✅ NFTSMC Vibration Manuscript
- [x] Port plant → `+plants/DualStageNanopositioner.m` (4-state, 2-mode piezo)
- [x] Port controller → `+controllers/NFTSMC.m` (feedforward + integral action)
- [x] Example: `example_nftsmc_nanopositioner.m` (3 scenarios: step, vibration, tracking)
- [x] Tests: 8 plant tests + 4 controller tests + 2 integration tests
- [ ] Validate against manuscript results (695 nm RMS target) — needs MATLAB run

### Phase 3: Additional Surfaces & Reaching Laws (Priority: MEDIUM)

#### Surfaces to Add
- [ ] `FractionalOrderSurface` — `s = D^alpha(e) + c*e`
- [ ] `GlobalSurface` — no reaching phase
- [ ] `PrescribedPerformanceSurface` — error constrained within funnel
- [ ] `AdaptiveSurface` — online gain adaptation
- [ ] `RLDiscoveredSurface` — RL agent output (placeholder for Phase 5)

#### Reaching Laws to Add
- [ ] `HyperbolicTangent` — `-k*tanh(s/phi)`
- [ ] `SigmoidRate` — `-k*s/(|s|+epsilon)`
- [ ] `AdaptiveGain` — adapts to disturbance bound
- [ ] `DiscreteTimeSMC` — discrete reaching law

### Phase 4: Additional Plants (Priority: MEDIUM)
- [x] `DualStageNanopositioner` — 4-state, 1-input (NFTSMC manuscript) ← moved to Phase 2
- [ ] `RobotArm2DOF` — 4-state, 2-input (fully actuated)
- [ ] `DCMotor` — 2-state, 1-input (teaching)
- [ ] `MassSpringDamper` — 2-state, 1-input
- [ ] `PMSM` — permanent magnet synchronous motor

### Phase 5: RL Integration (Priority: HIGH — feeds RL paper)
- [ ] Define `RLDiscoveredSurface` class
- [ ] Create Gymnasium wrapper (Python bridge or pure Python port)
- [ ] Implement PPO/SAC training loop
- [ ] Train RL agent to discover surfaces on DoubleIntegrator
- [ ] Compare RL vs ALL classical surfaces in benchmark
- [ ] Extend to Quadrotor and Crane
- [ ] Write paper

### Phase 6: Publication (Priority: HIGH)

#### Paper 1: Toolbox (SoftwareX)
- [x] Write paper draft (docs/softwarex_paper.tex)
- [ ] Create GitHub release with DOI (Zenodo)
- [ ] Submit to SoftwareX
- **Timeline:** After MATLAB validation

#### Paper 2: Benchmark Study (Control Engineering Practice / ISA Transactions)
- [ ] Run full benchmark: 8+ variants × 5+ plants
- [ ] Generate comparison tables and figures
- [ ] Write "first standardized benchmark for SMC" paper

#### Paper 3: RL-Discovered Sliding Surfaces (Automatica / IEEE TAC)
- [ ] Demonstrate RL agent discovering novel surfaces
- [ ] Show they outperform ALL classical designs
- [ ] Prove stability (or empirical evidence)

### Phase 7: Community & Maintenance (Priority: LOW)
- [ ] Push to GitHub as `Balghanimi/OpenSMC`
- [ ] README with installation, quick start, citation
- [ ] LICENSE (MIT or BSD-3)
- [ ] MATLAB File Exchange submission
- [ ] Python port (`opensmc` pip package)
- [ ] CI/CD (GitHub Actions for MATLAB tests)

---

## Architecture Diagram

```
                    +core/ (Abstract Interfaces)
                    ├── SlidingSurface
                    ├── ReachingLaw
                    ├── Plant
                    ├── Controller
                    ├── Estimator
                    └── Architecture
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                  │
   +architectures/   +controllers/     +benchmark/
   ├── DirectSMC     ├── ClassicalSMC   ├── Simulator
   ├── CascadedSMC   ├── AdaptiveSMC    ├── Metrics (12)
   │                 ├── DynamicSMC     └── BenchmarkRunner
   │                 ├── AggregatedHSMC
   │                 ├── IncrementalHSMC
   │                 ├── CombiningHSMC
   │                 ├── ITSMC
   │                 └── NFTSMC
         │
    ┌────┴────┐
    │         │
+surfaces/ +reaching/  +plants/           +estimators/     +utils/
(9)        (5)         ├── DoubleInt      ├── NoEstimator  ├── references
                       ├── InvPend        ├── DOB          └── disturbances
                       ├── SPCrane        ├── ESO
                       ├── DPCrane        ├── HighGain
                       ├── Quad6DOF       ├── ICD
                       └── Nanopositioner └── RBF_ELM
```

---

## Source Material Inventory

| Source | Pages | Status | Yield |
|--------|-------|--------|-------|
| Liu & Wang (2013) | 366 | ✅ EXTRACTED | 1 surface, 2 ctrl, 4 est |
| Qian & Yi (2015) | 210 | ✅ EXTRACTED | 2 plants, 3 ctrl, 2 surf |
| Bandyopadhyay (2009) | 190 | ✅ EXTRACTED | 1 surface |
| Ben (2013) | 369 | ❌ NOT EXTRACTED | (SMO, ObserverBased arch) |
| Argha (2021) | ? | ❌ NOT EXTRACTED | (discrete-time SMC) |
| ITSMC-ELM manuscript | 6 files | ✅ PORTED | 1 plant, 1 ctrl, 1 est, 1 arch |
| NFTSMC manuscript | 1 Python file | ✅ PORTED | 1 plant, 1 ctrl, 1 example |

---

## How This Feeds Other Projects

```
OpenSMC Toolbox
    │
    ├──→ RL-Discovered Sliding Surfaces (all baselines provided)
    ├──→ Master student ITSMC-ELM thesis (module + benchmarks)
    ├──→ Quadrotor notebooks (Quadrotor6DOF plant shared)
    ├──→ Teaching at University of Kufa (student assignments)
    └──→ Future students (each thesis = new module)
```
