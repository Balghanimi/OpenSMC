# OpenSMC Validation Roadmap

**Purpose:** Every component must trace back to a published reference with verifiable results.
"Our result vs. their result — it must match."

**Overall Progress: 15/43 components validated (35%)**

| Phase | Components | Validated | Status |
|-------|-----------|-----------|--------|
| 1. Foundations | 5 | 5/5 | COMPLETE |
| 2. Surfaces | 10 | 10/10 | COMPLETE |
| 3. Controllers | 10 | 4/10 | IN PROGRESS |
| 4. Estimators | 6 | 0/6 | Pending |
| 5. Plants | 8 | 0/8 | Pending |
| 6. Benchmarks | 4 | 0/4 | Pending |
| **Total** | **43** | **15/43** | **35%** |

**Total tests passing: 262 (across 15 notebooks)**

**Convention:** Each validation includes:
1. The published reference (book/paper, equation numbers, figure numbers)
2. A Python Jupyter notebook reproducing the reference example from scratch
3. The same simulation run in OpenSMC MATLAB
4. A comparison table: Reference values vs. OpenSMC values vs. Python values

---

## Master Validation Table

| # | Component | Type | Reference | Eq/Fig | Notebook | MATLAB Validated | Status |
|---|-----------|------|-----------|--------|----------|-----------------|--------|
| **Phase 1: Foundations** |
| 1 | DoubleIntegrator + LinearSurface + ConstantRate + ClassicalSMC | Plant+Surf+Reach+Ctrl | Liu (2017) Ch 1, §1.1 | Eq 1.1-1.4, Fig 1.1-1.6 | `01_Classical_SMC_Liu_Ch1.ipynb` | Pending (u_eq=0 gap) | **PYTHON VALIDATED 6/6** |
| 2 | ExponentialRate | Reaching | Gao & Hung (1993) IEEE TIE; Liu (2017) §1.3-1.4 | Eq 1.7-1.9, 1.11-1.16, Fig 1.8-1.10 | `02_Reaching_Laws_Liu_Ch1.ipynb` | Pending | **PYTHON VALIDATED 6/6** |
| 3 | PowerRate | Reaching | Liu (2017) Ch 1 §1.3, Eq 1.9 | Eq 1.9 | `02_Reaching_Laws_Liu_Ch1.ipynb` | | **VALIDATED in NB#2** |
| 4 | SuperTwisting | Reaching | Levant (1993) IJOC; Shtessel et al. (2014) Ch 1 | Table p.37 | `03_SuperTwisting_Levant.ipynb` | Pending (update_state gap) | **PYTHON VALIDATED 6/6** |
| 5 | Saturation (boundary layer) | Reaching | Slotine (1984) IJOC; Liu (2017) | | `03_SuperTwisting_Levant.ipynb` | | **VALIDATED in NB#3** |
| **Phase 2: Surfaces** |
| 6 | TerminalSurface | Surface | Zak (1988) Phys Lett A | T_f formula | `04_Terminal_Surfaces.ipynb` | | **PYTHON 16/16 PASS** |
| 7 | NonsingularTerminalSurface | Surface | Yu & Zhihong (2002) IEEE TCAS-I | Singularity-free | `04_Terminal_Surfaces.ipynb` | | **VALIDATED in NB#4** |
| 8 | FastTerminalSurface | Surface | Liu & Wang (2012) Ch 7.3 | Closed-loop | `04_Terminal_Surfaces.ipynb` | | **VALIDATED in NB#4** |
| 9 | IntegralSlidingSurface | Surface | Qian & Yi (2015) App C; Utkin & Shi (1996) | s(0)=0 | `09_Integral_Sliding.ipynb` | | **PYTHON 17/17 PASS** |
| 10 | PIDSurface | Surface | Qian & Yi (2015) App E | PID structure | `10_PID_Surface.ipynb` | | **PYTHON 12/12 PASS** |
| 11 | GlobalSurface | Surface | Bartoszewicz (1998) IEEE TIE | s(0)=0, exp decay | `11_Global_Surface.ipynb` | | **PYTHON 35/35 PASS** |
| 12 | PredefinedTimeSurface | Surface | Sanchez-Torres et al. (2018) | e→0 before Tc | `12_PredefinedTime_Surface.ipynb` | | **PYTHON 31/31 PASS** |
| 13 | NonlinearDampingSurface | Surface | Bandyopadhyay et al. (2009) LNCIS 392, Eq 2.7-2.8 | Adaptive damping | `13_Nonlinear_Damping.ipynb` | | **PYTHON 27/27 PASS** |
| 14 | IntegralTerminalSurface | Surface | Liu & Wang (2012); Al Ghanimi (2026) | Finite-time + integral | `14_Integral_Terminal.ipynb` | | **PYTHON 24/24 PASS** |
| 15 | HierarchicalSurface | Surface | Qian & Yi (2015) Ch 4 | Underactuated | `15_Hierarchical_Surface.ipynb` | | **PYTHON 24/24 PASS** |
| **Phase 3: Controllers** |
| 16 | AdaptiveSMC | Controller | Liu & Wang (2012) Ch 6 | Gain adaptation | `16_Adaptive_SMC_Liu_Ch2.ipynb` | | **PYTHON 13/13 PASS** |
| 17 | DynamicSMC | Controller | Liu & Wang (2012) Ch 5 | Continuous u(t) | `17_Dynamic_SMC.ipynb` | | **PYTHON 9/9 PASS** |
| 18 | AggregatedHSMC | Controller | Qian & Yi (2015) Ch 4, Eq 4.7-4.15 | | `18_Aggregated_HSMC.ipynb` | | Pending |
| 19 | IncrementalHSMC | Controller | Qian & Yi (2015) Ch 4, Eq 4.27-4.34 | | `19_Incremental_HSMC.ipynb` | | Pending |
| 20 | CombiningHSMC | Controller | Qian & Yi (2015) Ch 4, Eq 4.58 | | `20_Combining_HSMC.ipynb` | | Pending |
| 21 | FuzzySMC | Controller | Khanesar et al. (2021) Springer | | `21_Fuzzy_SMC.ipynb` | | Pending |
| 22 | DiscreteSMC | Controller | Gao et al. (1995) IEEE TIE 42(2) | Quasi-sliding band | `22_Discrete_SMC_Gao1995.ipynb` | | **PYTHON 13/13 PASS** |
| 23 | FixedTimeSMC | Controller | Polyakov (2012) IEEE TAC 57(8) | T_max bound | `23_FixedTime_Polyakov2012.ipynb` | | **PYTHON 23/23 PASS** |
| 24 | ITSMC | Controller | Al Ghanimi (2026) — own manuscript | | `24_ITSMC_Quadrotor.ipynb` | | Pending |
| 25 | NFTSMC | Controller | Al Ghanimi (2026) — own manuscript | | `25_NFTSMC_Nanopositioner.ipynb` | | Pending |
| **Phase 4: Estimators** |
| 26 | DisturbanceObserver | Estimator | Liu (2017) Ch 4 | | `26_Disturbance_Observer.ipynb` | | Pending |
| 27 | ExtendedStateObserver | Estimator | Liu (2017) Ch 3; Han (2009) ADRC | | `27_Extended_State_Observer.ipynb` | | Pending |
| 28 | HighGainObserver | Estimator | Liu (2017) Ch 3; Khalil (2015) | | `28_HighGain_Observer.ipynb` | | Pending |
| 29 | IntegralChainDifferentiator | Estimator | Liu (2017) Eq 8.37 | | `29_Integral_Chain_Diff.ipynb` | | Pending |
| 30 | LevantDifferentiator | Estimator | Levant (2003) IJC 76(9-10) | | `30_Levant_Differentiator.ipynb` | | Pending |
| 31 | RBF_ELM | Estimator | Huang et al. (2006) Neurocomputing | | `31_RBF_ELM.ipynb` | | Pending |
| **Phase 5: Plants** |
| 32 | InvertedPendulum | Plant | Khalil (2015); Spong (2005) | | `32_Inverted_Pendulum.ipynb` | | Pending |
| 33 | SinglePendulumCrane | Plant | Qian & Yi (2015) Eq 2.16 | | `33_Single_Pendulum_Crane.ipynb` | | Pending |
| 34 | DoublePendulumCrane | Plant | Qian & Yi (2015) Eq 2.46 | | `34_Double_Pendulum_Crane.ipynb` | | Pending |
| 35 | TwoLinkArm | Plant | Slotine & Li (1991) Ex 9.7; Spong (2006) Ch 6 | | `35_TwoLink_Arm.ipynb` | | Pending |
| 36 | PMSM | Plant | Krause et al. (2013) Ch 5 | | `36_PMSM.ipynb` | | Pending |
| 37 | SurfaceVessel | Plant | Fossen (2011) Ch 6 | | `37_Surface_Vessel.ipynb` | | Pending |
| 38 | Quadrotor6DOF | Plant | Al Ghanimi (2026) — own manuscript | | `38_Quadrotor_6DOF.ipynb` | | Pending |
| 39 | DualStageNanopositioner | Plant | Al Ghanimi (2026) — own manuscript; Fleming (2009) | | `39_Nanopositioner.ipynb` | | Pending |
| **Phase 6: Integrated Benchmarks (Published Figure Reproduction)** |
| 40 | Crane HSMC — Qian & Yi Fig 4.x | Benchmark | Qian & Yi (2015) Ch 4 | | `40_Crane_HSMC_QianYi.ipynb` | | Pending |
| 41 | Robot Arm — Slotine & Li Fig 9.x | Benchmark | Slotine & Li (1991) Ch 9 | | `41_RobotArm_SlotineLi.ipynb` | | Pending |
| 42 | ITSMC Quadrotor — own results | Benchmark | Al Ghanimi (2026) | | `42_ITSMC_Quadrotor_Full.ipynb` | | Pending |
| 43 | NFTSMC Nanopositioner — own results | Benchmark | Al Ghanimi (2026) | | `43_NFTSMC_Nano_Full.ipynb` | | Pending |

---

## References Available as PDFs

| Reference | Status | Location |
|-----------|--------|----------|
| Liu, J. (2017) "Sliding Mode Control Using MATLAB" | **AVAILABLE** | `D:/Ali_Kufa_University/.../SlidingModeControlUsingMATLAB.pdf` |
| Shtessel, Edwards, Fridman, Levant (2014) "Sliding Mode Control and Observation" | **AVAILABLE** | `D:/Ali_Kufa_University/.../SlidingModeControlandObservation-*.pdf` |
| Khalil (2001) "Nonlinear Systems" 3rd Ed | **AVAILABLE** | `D:/Ali_Kufa_University/.../Hassan K. Khalil - Nonlinear Systems*.pdf` |
| Spong et al. (2005) "Robot Modeling and Control" | **AVAILABLE** | `D:/Ali_Kufa_University/.../Spong - Robot Modeling and Control (2005).pdf` |
| Li, Yang, Chen (2014) "Disturbance Observer-Based Control" | **AVAILABLE** | `D:/Ali_Kufa_University/.../Disturbance Observer-Based Control*.pdf` |
| Qian & Yi (2015) "Hierarchical SMC" | **MISSING** — need to acquire | |
| Slotine & Li (1991) "Applied Nonlinear Control" | **MISSING** — need to acquire | |
| Bandyopadhyay et al. (2009) LNCIS 392 | **MISSING** — need to acquire | |
| Liu & Wang (2012) "SMC of Uncertain Parameter-Switching" | **MISSING** — need to acquire | |

---

## Key Findings from Initial Audit

1. **Sign convention difference:** OpenSMC uses `e = xref - x` (desired minus actual). Liu (2017) uses `e = x - xd` (actual minus desired). Both are valid but produce opposite-sign sliding variables. Each notebook must document this.

2. **Equivalent control gap:** Our ClassicalSMC has `u_eq = 0` hardcoded. Liu's Ch 1 example requires `u_eq = J(-cė + θ̈_d)` which is nonzero for J≠1 or θ̈_d≠0. This needs fixing.

3. **DoubleIntegrator has no inertia parameter J.** Liu's example uses J=2. We may need to add this or validate using a scaled version.

---

## Progress Log

| Date | Notebook | Result |
|------|----------|--------|
| 2026-03-14 | #01 Classical SMC (Liu Ch1) | **PYTHON 6/6 PASS** — s(0)=-10 exact, settling k=10: 0.81s, k=0: 16.16s, max\|u\|=101.1, zero overshoot. Identified 4 OpenSMC gaps (u_eq, J param, sign convention, reaching law mapping). |
| 2026-03-14 | #02 Reaching Laws (Liu Ch1 §1.3-1.4) | **PYTHON 6/6 PASS** — All 3 reaching laws validated. Exp reaches 2.1x faster than constant. Control range [-0.268, 0.451] matches Fig 1.9 [-0.3, 0.5]. Sinusoidal tracking error < 6.5e-5 after 3s. |
| 2026-03-14 | #03 Super-Twisting (Levant 1993) | **PYTHON 6/6 PASS** — 3835x smoother control than classical sign. SS range [−0.42,0.55] vs [−15,15]. Reaching 0.63s. Found OpenSMC gap: update_state not called by ClassicalSMC. |
| 2026-03-14 | #04 Terminal Surface Family (Zak/Yu/Liu) | **PYTHON 16/16 PASS** — Finite-time convergence formula exact (all 16 configs <5% error). Singularity confirmed. Terminal settles 0.45s vs linear 0.59s. All 4 surfaces converge closed-loop. |
| 2026-03-14 | #09 Integral Sliding (Utkin/Qian) | **PYTHON 17/17 PASS** — s(0)=0 for 5 ICs. Reaching phase eliminated (3896x speedup). Control spike reduced 445→15. Integral state E converges to constant. |
| 2026-03-14 | #10 PID Surface (Qian & Yi) | **PYTHON 12/12 PASS** — Formula verified 6/6. PID(γ=0)≡Linear confirmed. SS error eliminated with integral action. 2nd-order SMC produces continuous control. |
| 2026-03-14 | #11 Global Surface (Bartoszewicz) | **PYTHON 35/35 PASS** — s(0)=0 for 5 ICs (exact). Exponential decay matches theory. Reaching phase eliminated. α effect: faster α → faster transition to standard sliding. |
| 2026-03-14 | #12 PredefinedTime Surface (Sánchez-Torres) | **PYTHON 31/31 PASS** — c(t) profile verified (monotone, diverges at Tc). Convergence before Tc: 9/9 configs (Tc×e0 grid). Robust to d=2sin(3t). Post-Tc reverts to linear. |
| 2026-03-14 | #13 Nonlinear Damping (Bandyopadhyay) | **PYTHON 27/27 PASS** — Gaussian/Exponential Ψ shapes verified. Overshoot reduced (31.96%→0%). Settling time acceptable. Both Ψ types work. |
| 2026-03-14 | #14 Integral Terminal (Liu/Al Ghanimi) | **PYTHON 24/24 PASS** — Formula verified 8/8. Disturbance rejection with integral action. Finite-time convergence on manifold. 3-surface comparison: ITSMC best ISE+SS error. |
| 2026-03-14 | #15 Hierarchical Surface (Qian & Yi) | **PYTHON 24/24 PASS** — Decomposition verified 6/6. λ prioritization confirmed. Pendulum-cart stabilization (both DOFs). Underactuated gain coupling insight documented. |
| 2026-03-14 | #16 Adaptive SMC (Liu Ch6) | **PYTHON 13/13 PASS** — Gain grows past |d|_max. Adaptive outperforms under-tuned fixed. Monotone adaptation verified. Scales with disturbance level. |
| 2026-03-14 | #17 Dynamic SMC (Liu Ch5) | **PYTHON 9/9 PASS** — TV(u) much lower than classical. u(t) continuous. Proper sigma formulation with plant model. K-tradeoff verified. |
| 2026-03-14 | #22 Discrete SMC (Gao 1995) | **PYTHON 13/13 PASS** — Quasi-sliding band matches theory. Band grows with Ts. ZOH control converges. Robust to sinusoidal disturbance. |
| 2026-03-14 | #23 Fixed-Time SMC (Polyakov 2012) | **PYTHON 23/23 PASS** — T_max bound holds for all ICs (0.01 to 1000). Bi-power beats finite-time for large ICs. Closed-loop converges. |
