---
name: gpops2-matlab
description: Use when installing, configuring, troubleshooting, or writing optimal control problems for GPOPS-II v1.0 with MATLAB R2025a. Developed on Linux/WSL but platform-independent — works on Windows and macOS. Covers setup struct requirements, SNOPT solver quirks, output structure, path persistence, and common error patterns.
---

# GPOPS-II + MATLAB Optimal Control

Expert-level guidance for **GPOPS-II v1.0** with **MATLAB R2025a** — hp-adaptive Radau pseudospectral methods + SNOPT solver. Developed on Linux/WSL; platform-independent patterns work on Windows and macOS.

**Full reference**: See `REFERENCE.md` for warm start, EKF integration, obstacle penalty design, and detailed workflow code.

## Prerequisites

- MATLAB at `<MATLAB_ROOT>/`
- GPOPS-II at e.g. `<GPOPS2_ROOT>/`
- `snoptcmex.mexa64` under `Gpops/nlp/snopt/` — **only Linux NLP solver** (IPOPT only ships Windows binaries)
- Path persistence: `sudo cp ~/.matlab/R2025a/pathdef.m <MATLAB_ROOT>/toolbox/local/pathdef.m`

---

## Critical Setup Fields (Getting These Wrong = Silent Failure)

- **Terminal states** → `setup.bounds.phase.finalstate.lower/upper` (NOT event constraints)
- **Guess** → `setup.guess.phase(1).time/state/control` (WITH index `(1)`, even single-phase)
- **Exit flag** → `output.result.nlpinfo` (numeric: 1 = success). There is NO `solverinfo` field.
- **Mesh history** → `output.meshhistory` (top level, NOT inside `output.result`)
- **Multiple initial intervals** → `setup.mesh.phase.colpoints = 6*ones(1,5)` and `setup.mesh.phase.fraction = ones(1,5)/5`
- **Parameter passing** → ONLY `setup.auxdata = auxdata` — read via `input.auxdata` in functions. Do NOT use global variables.

## Mesh Field Name: `maxiteration` (NO trailing 's')

```matlab
setup.mesh.maxiteration = 8;     % ✓ Correct — GPOPS-II reads this
setup.mesh.maxiterations = 8;    % ✗ Silently ignored — defaults to 25
```

`gpopsDefaults.m` checks `isfield(setup.mesh, 'maxiteration')` — the plural form never matches. Default 25 refinements causes mesh explosion (21 → 228+ grid points).

## SNOPT: Only `setup.nlp.options.tolerance` Works

GPOPS-II v1.0's SNOPT handler recognizes **exactly one** user-settable parameter:

```matlab
setup.nlp.options.tolerance = 1e-4;   % Optimality tol; Feasibility = 2× this
```

**All other fields** in `setup.nlp.snoptoptions.*` or `setup.nlp.options.*` are silently ignored. Iteration limits (100000), derivative options, etc. are hard-coded in `gpopsSnoptHandlerRPMD.m`.

To truly change SNOPT iteration limits, edit the handler source directly (see `REFERENCE.md` SNOPT Parameter Mapping section).

## Mesh Tolerance vs SNOPT Tolerance Must Match

| Scene | mesh.tolerance | nlp.options.tolerance | mesh.maxiteration |
|---|---|---|---|
| Quick prototype | 5e-3 | 3e-3 | 5-8 |
| Medium accuracy | 1e-4 | 1e-4 | 15 |
| High accuracy | 1e-6 | 1e-6 | 25+ |

If `mesh.tolerance` < what SNOPT can achieve → infinite refinement loop. Recommended: `mesh.tolerance ≈ nlp.options.tolerance`.

## Output Structure

```
output.result.solution.phase → .time [N×1], .state [N×ns], .control [N×nc], .costate, .timeRadau, .controlRadau
output.result.nlpinfo          → SNOPT exit code (1 = optimal)
output.result.maxerror         → final mesh error
output.result.interpsolution   → high-res interpolation
output.meshiterations          → number of refinements
output.meshhistory(i).result   → per-iteration results
output.totaltime
```

## SNOPT Exit Code Quick Reference

| nlpinfo | Meaning | Action |
|---------|---------|--------|
| 1 | **Optimal** | Normal |
| 1001 | Optimal, weak dual feasibility | Usable |
| 13010 | **Nonlinear infeas minimized** (common) | Usable if violation < tol |
| 41040 | **Cannot improve** (common) | Usable — local optimum reached |
| 11001 | Linear infeas | Check bounds/obstacles |
| 32032 | Iteration limit | Fix guess or increase limit |

**Rule**: Never discard solution solely on nlpinfo ≠ 1. Inspect constraint violations and objective.

## Quick Workflow

### Solve (silent mode)

```matlab
[~, output] = evalc('gpops2(setup)');   % No terminal output
```

**Important**: `evalc('gpops2(setup)')` does NOT work inside `parfor` — see [parfor and evalc Transparency Violation](#parfor-and-evalc-transparency-violation) for the fix.

### Setup skeleton

```matlab
setup.name = 'MyProblem';
setup.functions.continuous = @myContinuous;
setup.functions.endpoint   = @myEndpoint;
setup.nlp.solver           = 'snopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.method               = 'RPMintegration';
setup.mesh.method          = 'hp1';
setup.mesh.tolerance       = 1e-5;
setup.mesh.maxiteration    = 15;        % NOT 'maxiterations'!
setup.mesh.phase.colpoints = 4 * ones(1, 5);
setup.mesh.phase.fraction  = ones(1, 5) / 5;
setup.nlp.options.tolerance = 1e-4;     % Only effective SNOPT param
```

### Continuous & Endpoint Functions (MUST be local functions in same file)

```matlab
function phaseout = myContinuous(input)
    x = input.phase.state(:, 1:n);
    u = input.phase.control(:, 1:m);
    phaseout.dynamics = xdot;
    phaseout.integrand = 0.5 * u.^2;   % Lagrange cost
end

function output = myEndpoint(input)
    output.eventgroup(1).event = [];
    output.objective = 0;              % Mayer cost
end
```

### SNOPT Objective = 0 → Compute Lagrange Manually

SNOPT only shows Mayer term. Total cost:
```matlab
J_total = output.result.objective + trapz(t, integrand_values);
```

## parfor and evalc Transparency Violation

`evalc` in a parfor body triggers "Transparency Violation" — parfor cannot verify the dynamic workspace at compile time:

```matlab
parfor i = 1:N
    [~, output] = evalc('gpops2(setup)');   % ✗  VIOLATION
end
```

**Fix**: wrap `evalc` in a separate `.m` file. The wrapper has its own scope, invisible to parfor's transparency check:

```matlab
% silent_gpops2.m
function output = silent_gpops2(setup)
    [~, output] = evalc('gpops2(setup)');
end

% main script
parfor i = 1:N
    output = silent_gpops2(setup);          % ✓  OK
end
```

For a full parfor example, see [REFERENCE.md](REFERENCE.md#parfor-and-evalc-real-world-example).

| Alternative | Feasible | Notes |
|-------------|----------|-------|
| Separate `.m` wrapper | ✅ Best | No side effects, reusable |
| Direct call (no evalc) | ✅ | Worker output hidden but buffered |
| `diary('/dev/null')` | ⚠️ | May also violate transparency |
| Edit SNOPT handler print level | ⚠️ | Invasive, affects all users |
| `evalc` directly in parfor | ❌ | Immediate error |

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `未定义函数 'gpops2'` | Copy pathdef.m to system location; verify `which gpops2` |
| Mesh iteration limit reached | Relax tolerance 1e-6→1e-5; increase initial colpoints |
| Grid explosion 21→228+ | Use `maxiteration` (no 's') — the plural is silently ignored |
| SNOPT objective = 0 | Compute Lagrange cost manually via `trapz` |
| `solverinfo` field not found | It doesn't exist — use `output.result.nlpinfo` |
| SNOPT exit 13, feasibility stuck ~3.4E-03 | Obstacle penalty gradient discontinuity — use C¹ smooth penalty |
| `-batch` parse error | Write `.m` file and `run()` it — never inline multiline commands |
| `Transparency Violation` error in parfor + evalc | Wrap evalc in separate `.m` function file (see [parfor and evalc](#parfor-and-evalc-transparency-violation) section) |
| Different cost from same problem | Initial guess/mesh → different NLP local minima. Compare ≥2 guesses. |
| IPOPT not found | No Linux MEX shipped — use `snopt` |
| auxdata not reaching function | Confirm `input.auxdata` in function; avoid globals |
| Warm start cycle 30s+ | Limit `mesh.maxiteration=8`; improve warm start guess |

## Reference

For warm start implementation, EKF closed-loop guidance, obstacle penalty function design (C¹ smooth), mesh tolerance tuning details, SNOPT handler source modification, and full changelog — see **`REFERENCE.md`**.

Key files:
- Built-in examples: `<GPOPS2_ROOT>/examples/`
