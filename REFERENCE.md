# GPOPS-II — Full Reference

Detailed guides for warm start, EKF integration, obstacle penalty design, mesh tuning, and SNOPT handler modification. Loaded only when LLM explicitly reads this file.

---

## Continuous Function Signature

```matlab
function phaseout = myContinuous(input)
    x = input.phase.state(:, 1:n);
    u = input.phase.control(:, 1:m);
    xdot = ...;
    phaseout.dynamics = xdot;
    phaseout.integrand = 0.5 * u.^2;   % Lagrange cost
end
```

## Endpoint Function Signature

```matlab
function output = myEndpoint(input)
    output.eventgroup(1).event = [];
    output.objective = 0;           % Mayer cost (adds to Lagrange)
end
```

---

## Path Persistence

```bash
sudo cp ~/.matlab/R2025a/pathdef.m <MATLAB_ROOT>/toolbox/local/pathdef.m
```

---

## Initial Guess Sensitivity (Even for LQR)

For linear dynamics + quadratic cost problems, the continuous-time solution is unique but the discretized NLP may converge to different minima depending on:

| Factor | Effect |
|--------|--------|
| Initial mesh structure (single vs multi-interval) | Different collocation grids → different NLP paths |
| Initial guess quality | "Smarter" guess ≠ better result |
| Mesh refinement path | hp-adaptive from different grids → different discretization paths |

**Recommendation**: Always run 2-3 different initial guesses, compare costs, keep lowest.

```matlab
% Simple linear guess (often works well)
setup.guess.phase(1).time    = [t0; tf];
setup.guess.phase(1).state   = [x0, y0; xf, yf];
setup.guess.phase(1).control = zeros(2, ncontrols);

% Physics-based guess (may or may not be better)
N = 50; t_guess = linspace(t0, tf, N)';
setup2.guess.phase(1).state = state_guess;
```

---

## Mesh Tolerance Critical Effect

| Tolerance | Typical Outcome | Grid Points |
|-----------|----------------|-------------|
| 1e-6 | May hit 25-iteration limit with error ~4e-6 | 500-700 |
| 1e-5 | Clean convergence in 15-21 iterations | 200-400 |
| 1e-4 | Very fast convergence, loose accuracy | 100-200 |

Start with 1e-5. Tighten only when verified.

---

## Obstacle Penalty Function Smoothness (C¹ Design)

SNOPT uses sparse central differences — non-smooth cost/constraint causes gradient discontinuity, Hessian degradation, feasibility stagnation.

**Original (non-smooth)**:
```matlab
p = r_obs - distance;
penalty = 10000 * max(0, p)^2;         % C⁰ only — gradient jump at p=0
```

**Fixed (C¹-smooth)**:
```matlab
p = r_obs - distance;
p_safe = max(0, p);
weight = 100;                          % Match control energy scale
phi = p_safe^2 / (1 + p_safe^2);       % C¹: φ(0)=0, φ'(0)=0, φ(∞)→1
penalty = weight * phi;
```

Weight should be ~10-100× typical control effort. Weight=10000 is excessive.

---

## Warm Start (Hot Start) Implementation

```matlab
prevSolution = [];
for cycle = 1:N_cycles
    setup = configureCycle(setup, cycle);
    if isempty(prevSolution)
        setup.guess.phase = guess(setup, num_guess_points);
    else
        setup.guess.phase = warmStartGuess(prevSolution, setup);
    end
    [~, output] = evalc('gpops2(setup)');
    prevSolution = output.result.solution;
end
```

Warm start interpolation:
```matlab
function guess = warmStartGuess(prevSoln, setup, numGuessPoints)
    if nargin < 3, N = length(prevSoln.phase.time); else, N = numGuessPoints; end
    t_prev = prevSoln.phase.time;
    t_new = linspace(setup.bounds.phase.initialtime.lower, ...
                     setup.bounds.phase.finaltime.lower, N)';
    guess.phase(1).time    = t_new;
    guess.phase(1).state   = interp1(t_prev, prevSoln.phase.state, t_new, 'linear');
    guess.phase(1).control = interp1(t_prev, prevSoln.phase.control, t_new, 'linear');
    guess.phase(1).integral = prevSoln.phase.integral(end);
end
```

Performance expectations (obstacle avoidance):
| Quality | Time/Cycle | SNOPT Major Iters | Grid Points |
|---------|-----------|-------------------|-------------|
| Ideal | 0.06-0.10s | 3-5 | ~21 |
| Moderate | 0.3-1.0s | 10-30 | 21-30 |
| Poor | 1-5s | 50-100 (exit 41/13) | 30-100 |
| Failed | 5-30s | 100 (exit 13) | 100-550 |

Warm start failure modes:
- **Exit 41**: Stationary point near guess — solution usually usable
- **Exit 13**: Cannot find feasible direction — improve guess or increase penalty weight
- **Mesh explosion**: Exit 13 → recursion → limit `mesh.maxiteration=8`

---

## EKF Closed-Loop Guidance Integration

Typical GNC timing (example values):
```
Cycle duration:   T_cycle
EKF update step:  T_ekf
Updates/cycle:    N_updates
Total cycles:     N_cycles
```

Per-cycle sequence:
1. EKF prediction (N_updates steps, using prev control)
2. EKF update (lidar: range ρ, azimuth α, elevation β)
3. Extract X_est(k)
4. Call gpops2: horizon = remaining time, initial state = X_est(k)
5. Extract u*(0) from solution
6. Apply as constant control for next EKF step
7. Save solution as warm start for cycle k+1

EKF measurement model (lidar):
```matlab
rho = sqrt(r(1)^2 + r(2)^2 + r(3)^2);
alpha = atan2(r(2), r(1));
beta = atan(r(3) / sqrt(r(1)^2 + r(2)^2));
sigma_rho = 0.1; sigma_alpha = 0.001; sigma_beta = 0.001;   % Example values
z = [rho; alpha; beta] + [sigma_rho; sigma_alpha; sigma_beta] .* randn(3,1);
```

Validation metrics:
```matlab
    pos_err = norm(solution.phase.state(end, 1:3));
    vel_err = norm(solution.phase.state(end, 4:6));
    J = 0.5 * trapz(solution.phase.time, sum(solution.phase.control.^2, 2));
```

---

## SNOPT Parameter Mapping (Source-Verified)

Only `setup.nlp.options.tolerance` is recognized:
```
setup.nlp.options.tolerance → snsetr('Optimality Tolerance', tol)
                           → snsetr('Feasibility Tolerance', 2*tol)
```

Hard-coded SNOPT parameters (cannot change via setup):
| Parameter | Value | Handler File:Line |
|-----------|-------|-------------------|
| Iteration Limit | 100000 | Both handlers:54/53 |
| Major Iterations | 100000 | Both handlers:55/54 |
| Minor Iterations | 100000 | Both handlers:56/55 |
| Derivative Option | 1 | Both handlers:62/61 |
| Hessian Limited Memory | enabled | Both handlers:50/49 |
| Verify Level | -1 | Both handlers:53/52 |
| Timing level | 3 | Both handlers:49/48 |

To change iteration limits, edit handler source:
```matlab
% <GPOPS2_ROOT>/lib/gpopsRPMDifferentiation/gpopsSnoptRPMD/gpopsSnoptHandlerRPMD.m
% (also the RPMI variant)
snseti('Iteration Limit', 2000);
snseti('Major Iterations Limit', 100);
snseti('Minor Iterations Limit', 2000);
```

---

## SNOPT Exit Code Reference (Extended)

Exit codes in `output.result.nlpinfo` = EXIT×1000 + INFO. Most positive codes are non-fatal.

| nlpinfo | EXIT | INFO | Meaning | Action |
|---------|------|------|---------|--------|
| 1 | 0 | 1 | Optimal | Normal |
| 1001 | 1 | 1 | Optimal, weak dual infeasibility | OK |
| 11001 | 11 | 1 | Infeasible — linear constraints | Check bounds |
| 13001 | 13 | 1 | Infeasible — nonlinear constraints | Relax penalty or tol |
| 13010 | 10 | 13 | Nonlinear infeas minimized (common) | Usable if violation < tol |
| 32032 | 32 | 32 | Iteration limit reached | Fix guess |
| 41040 | 40 | 41 | Cannot improve (common) | Local optimum; usable |
| 42042 | 42 | 42 | Cannot find feasible elastic | Overconstrained |

Practical handling:
```matlab
[~, output] = evalc('gpops2(setup)');
nlpinfo = output.result.nlpinfo;
if nlpinfo > 0 && nlpinfo ~= 1
    warning('SNOPT exit code %d, using current solution', nlpinfo);
end
```

---

## parfor and evalc Real World Example

Wrapper function (separate `.m` file, parfor-safe):

```matlab
function output = silent_solver_wrapper(varargin)
    [~, output] = evalc('main_solver(varargin{:})');
end
```

Call from parfor:
```matlab
parfor i = 1:N
    result(i) = silent_solver_wrapper(setup_params);
end
```

See `SKILL.md` for transparency-violation explanation, alternative approaches, and full troubleshooting.

---

## Changelog

- **2026-06-04**: Added evalc silent call pattern. Revised §13 mesh field name after source verification.
- **2026-05-26**: Added parfor+evalc real-world example. Corrected §12 after source inspection. Trimmed redundancy.
- **2026-05-26**: Added §12-17 (SNOPT mapping, mesh field name, obstacle smoothness, warm start, EKF, exit codes).
- **2025-12-01**: Initial creation.
