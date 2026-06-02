"""PyrokiSolver — JAX-based nonlinear IK optimization with collision avoidance."""

from __future__ import annotations

import os

import jax
import jax.numpy as jnp
import jaxls
import jaxlie
from typing import Any, Callable
from openarm_ik_solver.robot import BaseRobot

# Persistent compilation cache — survives server restarts.
# After first run, JIT warmup reads compiled XLA from disk instead of recompiling.
_CACHE_DIR = os.environ.get(
    "JAX_COMPILATION_CACHE_DIR",
    os.path.expanduser("~/.cache/jax_compilation"),
)
jax.config.update("jax_compilation_cache_dir", _CACHE_DIR)
jax.config.update("jax_persistent_cache_min_compile_time_secs", 0)
jax.config.update("jax_persistent_cache_min_entry_size_bytes", -1)

# Monkey-patch jaxls to use stable hashes instead of object identity.
# Without this, the JAX persistent cache keys change every process restart
# because id() and object.__hash__() return memory addresses.
# Note: jaxls ships _py310/ for Python 3.10 compat — patch that version.
import dis
import jaxls._py310._variables as _jaxls_vars
import jaxls._py310._problem as _jaxls_prob


def _callable_signature(func):
    """Compute a stable structural signature for a callable."""
    closure = getattr(func, "__closure__", None)
    if closure is not None:
        closure_vars = tuple(sorted(str(c.cell_contents) for c in closure))
    else:
        closure_vars = ()
    instance = getattr(func, "__self__", None)
    instance_id = (
        hash(type(instance).__module__ + "." + type(instance).__qualname__)
        if instance is not None else None
    )
    bytecode = dis.Bytecode(func)
    bytecode_tuple = tuple((i.opname, i.argrepr) for i in bytecode)
    return bytecode_tuple, closure_vars, instance_id


class _StableCallable:
    """Wraps a callable so it compares by bytecode signature instead of identity.

    This makes jdc.Static[Callable] pytree treedefs stable across process restarts,
    which is required for JAX persistent compilation cache to hit.
    """
    __slots__ = ("fn", "_sig")

    def __init__(self, fn):
        self.fn = fn
        self._sig = _callable_signature(fn)

    def __eq__(self, other):
        if isinstance(other, _StableCallable):
            return self._sig == other._sig
        return NotImplemented

    def __hash__(self):
        return hash(self._sig)

    def __call__(self, *args, **kwargs):
        return self.fn(*args, **kwargs)


def _stable_meta_hash(cls):
    return hash(cls.__module__ + "." + cls.__qualname__)


def _stable_meta_lt(cls, other):
    lhs = cls.__module__ + "." + cls.__qualname__
    rhs = other.__module__ + "." + other.__qualname__
    return lhs < rhs


_jaxls_vars._HashableSortableMeta.__hash__ = _stable_meta_hash
_jaxls_vars._HashableSortableMeta.__lt__ = _stable_meta_lt
_jaxls_prob._get_function_signature = _callable_signature

# Re-register jaxls pytree classes that have jdc.Static[Callable] fields.
# The original _flatten puts raw callables in the treedef tuple, which compare
# by identity (object address) and change every restart. We wrap them in
# _StableCallable so the treedef is structurally equal across runs.
from jax._src.tree_util import _registry as _pytree_registry


def _reregister_with_stable_callables(cls):
    """Patch a pytree class to wrap Callable static fields stably."""
    entry = _pytree_registry.get(cls)
    if entry is None:
        return
    orig_flatten = entry.to_iter
    orig_unflatten = entry.from_iter

    def patched_flatten(obj):
        children, treedef = orig_flatten(obj)
        stable_treedef = tuple(
            _StableCallable(v) if callable(v) and not isinstance(v, type) else v
            for v in treedef
        )
        return children, stable_treedef

    def patched_unflatten(treedef, children):
        unwrapped = tuple(
            v.fn if isinstance(v, _StableCallable) else v for v in treedef
        )
        return orig_unflatten(unwrapped, children)

    _pytree_registry[cls] = type(entry)(patched_flatten, patched_unflatten)


import jaxls._py310._cost as _cost_mod
import jaxls._py310._analyzed_cost as _acost_mod

_reregister_with_stable_callables(_cost_mod.Cost)
_reregister_with_stable_callables(_acost_mod._AnalyzedCost)


class PyrokiSolver:
    """
    Inverse Kinematics (IK) solver using Pyroki and jaxls.

    This solver uses optimization (Least Squares) to find joint configurations
    that satisfy target poses for the robot's end-effectors.
    It leverages JAX for high-performance, JIT-compiled solving.
    """

    robot: BaseRobot
    _jit_solve: Callable[
        [jaxlie.SE3 | None, jaxlie.SE3 | None, jaxlie.SE3 | None, jnp.ndarray],
        jnp.ndarray,
    ]
    warmup_complete: bool
    warmup_error: str | None
    warmed_target_patterns: tuple[tuple[bool, bool, bool], ...]

    def __init__(self, robot: BaseRobot, solver_config: Any = None):
        """
        Initialize the solver with a robot model.

        Args:
            robot: The robot model providing kinematic info and costs.
            solver_config: Optional solver configuration dict/object with
                max_iterations, linear_solver, cost_weights, etc.
        """
        self.robot = robot
        self.solver_config = solver_config
        self.warmup_complete = False
        self.warmup_error = None
        self.warmed_target_patterns = tuple()

        # JIT compile the solve function
        self._jit_solve = jax.jit(self._solve_internal)

        # Warmup to trigger JIT compilation
        self._warmup()

    def _solve_internal(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
        target_Head: jaxlie.SE3 | None,
        q_current: jnp.ndarray,
    ) -> jnp.ndarray:
        """Internal solve function that will be JIT-compiled."""
        # 1. Build costs from the robot
        costs = self.robot.build_costs(target_L, target_R, target_Head, q_current)

        # 2. Get the joint variable (single timestep index 0)
        var_joints = self.robot.joint_var_cls(jnp.array([0]))

        # 3. Construct initial values
        initial_vals = jaxls.VarValues.make(
            [var_joints.with_value(q_current[jnp.newaxis, :])]
        )

        # 4. Construct and solve the LeastSquaresProblem
        problem = jaxls.LeastSquaresProblem(costs, [var_joints])

        # Read solver hyperparameters from config if available
        cfg = self.solver_config
        max_iters = getattr(cfg, "max_iterations", 15) if cfg is not None else 15
        lin_solver = getattr(cfg, "linear_solver", "dense_cholesky") if cfg is not None else "dense_cholesky"

        solution = problem.analyze().solve(
            initial_vals=initial_vals,
            verbose=False,
            linear_solver=lin_solver,
            termination=jaxls.TerminationConfig(max_iterations=max_iters),
        )

        # Return the optimized joint configuration for timestep 0
        return solution[var_joints][0]

    def solve(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
        target_Head: jaxlie.SE3 | None,
        q_current: jnp.ndarray,
    ) -> jnp.ndarray:
        """
        Solve the IK problem for the given targets and current configuration.

        Args:
            target_L: Target pose for the left end-effector.
            target_R: Target pose for the right end-effector.
            target_Head: Target pose for the head (unused for OpenArm, pass None).
            q_current: Current joint configuration (initial guess).

        Returns:
            jnp.ndarray: Optimized joint configuration.
        """
        return self._jit_solve(target_L, target_R, target_Head, q_current)

    def _warmup(self) -> None:
        """Triggers JIT compilation by running solve with dummy data."""
        import time
        print(f"[PyrokiSolver] Warmup starting (device={jax.default_backend()}, cache_dir={_CACHE_DIR})")
        t0 = time.perf_counter()
        try:
            q_dummy = self.robot.get_default_config()
            target_dummy = jaxlie.SE3.identity()

            # Head is always None for OpenArm — only warmup L/R patterns
            warmup_inputs = (
                (target_dummy, target_dummy, None),   # both arms active
                (target_dummy, None, None),            # left only
                (None, target_dummy, None),            # right only
                (None, None, None),                    # neither active
            )

            for target_L, target_R, target_Head in warmup_inputs:
                warmed = self.solve(target_L, target_R, target_Head, q_dummy)
                jax.block_until_ready(warmed)

            self.warmed_target_patterns = tuple(
                (
                    target_L is not None,
                    target_R is not None,
                    target_Head is not None,
                )
                for target_L, target_R, target_Head in warmup_inputs
            )
            elapsed = time.perf_counter() - t0
            print(f"[PyrokiSolver] Warmup complete in {elapsed:.1f}s ({len(warmup_inputs)} patterns compiled)")
            self.warmup_complete = True
            self.warmup_error = None
        except Exception as exc:
            self.warmup_complete = False
            self.warmup_error = str(exc)
            self.warmed_target_patterns = tuple()
            print(f"[PyrokiSolver] Warmup failed: {exc}")
