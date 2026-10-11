"""Small offline SE(2) graph. Poses are [world x metres, world y metres, CCW yaw radians].

Pose composition maps local coordinates to world. Twist order is [vx, vy, omega].
Derived tangent headings are geometric references, never native TIC-VLA yaw.
"""
from time import perf_counter
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from research.route_geometry import interpolate, wrap


def compose(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([a[..., 0]+c*b[..., 0]-s*b[..., 1],
                     a[..., 1]+s*b[..., 0]+c*b[..., 1], wrap(a[..., 2]+b[..., 2])], axis=-1)


def inverse(a):
    a = np.asarray(a, float); c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([-c*a[..., 0]-s*a[..., 1], s*a[..., 0]-c*a[..., 1], wrap(-a[..., 2])], axis=-1)


def between(a, b):
    return compose(inverse(a), b)


def exp(twist):
    v = np.asarray(twist, float); w = v[..., 2]
    a = np.sinc(w/np.pi); b = .5*w*np.sinc(w/(2*np.pi))**2
    return np.stack([a*v[..., 0]-b*v[..., 1], b*v[..., 0]+a*v[..., 1], wrap(w)], axis=-1)


def log(pose):
    p = np.asarray(pose, float); w = wrap(p[..., 2])
    a = np.sinc(w/np.pi); b = .5*w*np.sinc(w/(2*np.pi))**2; den = a*a+b*b
    return np.stack([(a*p[..., 0]+b*p[..., 1])/den, (-b*p[..., 0]+a*p[..., 1])/den, w], axis=-1)


def rigid_align(reference, boundary):
    return compose(compose(boundary, inverse(reference[0])), reference)


def relative_error(reference, output):
    return log(between(between(reference[:-1], reference[1:]), between(output[:-1], output[1:])))


def tangent_headings(times, xy, policy):
    """Clipped +/-0.2s chord; one-sided at endpoints. Never move any XY sample.

    Net chord / traversed arc is a direction-coherence diagnostic. Backward
    micro-segments are counted, not individually converted to robot yaw or flipped.
    """
    times, xy = np.asarray(times, float), np.asarray(xy, float)
    if len(times) < 2 or not np.isfinite(xy).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Finite ordered polyline required')
    headings, diagnostics = [], []
    for t in times:
        lo, hi = max(times[0], t-policy['half_window_s']), min(times[-1], t+policy['half_window_s'])
        q = np.unique(np.r_[lo, times[(times > lo) & (times < hi)], hi])
        p = interpolate(times, xy, q); chord = p[-1]-p[0]
        length = float(np.linalg.norm(chord)); arc = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        coherence = length/arc if arc > 0 else 0.
        reason = ('short chord' if length < policy['minimum_chord_m'] else
                  'incoherent/backtracking chord' if coherence < policy['minimum_coherence'] else None)
        headings.append(np.arctan2(chord[1], chord[0]) if reason is None else np.nan)
        diagnostics.append(dict(window_start=float(lo), window_end=float(hi), chord_m=length, arc_m=arc,
                                coherence=coherence, valid=reason is None, reason=reason))
    return np.asarray(headings), diagnostics


def reference_at_switch(world, observation_xy, observation_time, switch_time, config):
    dt = config['native_dt_s']; knots = observation_time+dt*np.arange(len(world)+1)
    # Existing boundary_seam interpolation convention: derived tau=0 observation
    # anchor, kept outside native arrays. No extrapolation past either endpoint.
    points = np.vstack([observation_xy, world])
    if switch_time < knots[0] or switch_time >= knots[-1]:
        raise ValueError('Switch outside positive remaining FRESH horizon; no extrapolation')
    query = np.r_[switch_time, knots[knots > switch_time+config['time_epsilon_s']]]
    if len(query) < 3:
        raise ValueError('Fewer than two editable FRESH nodes')
    xy = interpolate(knots, points, query)
    yaw, diagnostics = tangent_headings(query, xy, config['tangent'])
    return query, np.column_stack([xy, yaw]), diagnostics


def correspondence(times, reference, old_world, old_observation_time, config):
    """Only supported editable nominal times; no OLD yaw, nearest points or extrapolation."""
    knots = old_observation_time+config['native_dt_s']*np.arange(1, len(old_world)+1)
    tau = times-times[0]; cfg = config['correspondence']; eps = config['time_epsilon_s']
    mask = (tau > eps) & (tau < cfg['support_after_switch_s']-eps) & (times >= knots[0]) & (times <= knots[-1])
    ids = np.flatnonzero(mask); xy = interpolate(knots, old_world, times[ids]) if len(ids) else np.empty((0, 2))
    disagreement = np.linalg.norm(xy-reference[ids, :2], axis=1)
    locality = (1-tau[ids]/cfg['support_after_switch_s'])**2
    confidence = 1/(1+(disagreement/cfg['disagreement_scale_m'])**2)
    return dict(indices=ids, old_xy=xy, weights=locality*confidence, locality=locality,
                confidence=confidence, reference_disagreement_m=disagreement, absolute_times=times[ids])


class Graph:
    """X0 is excluded. Editable poses use Xj=Fj Exp(xij), not Euclidean yaw blending."""
    def __init__(self, reference, boundary, times, corr, direction, config, use_correspondence=True):
        self.f = np.array(reference, dtype=float, copy=True); self.b = np.array(boundary, dtype=float, copy=True)
        self.times = np.asarray(times, float); self.corr = corr; self.direction = direction; self.cfg = config
        self.use_corr = use_correspondence
        if self.f.ndim != 2 or self.f.shape[1] != 3 or len(self.f) < 3 or not np.isfinite(self.f).all() or not np.isfinite(self.b).all():
            raise ValueError('Invalid finite SE(2) reference/boundary')
        if len(times) != len(self.f) or np.any(np.diff(times) <= 0):raise ValueError('Invalid node times')
        self.df = between(self.f[:-1], self.f[1:]); s = config['scales']
        self.rel_scale = np.array([s['relative_translation_m']]*2+[np.radians(s['relative_rotation_deg'])])
        self.intent_scale = np.array([s['intent_translation_m']]*2+[np.radians(s['intent_rotation_deg'])])
        tau = self.times[1:]-self.times[0]; c = config['intent']
        self.intent_weight = c['near_weight']+(1-c['near_weight'])*np.minimum(tau/c['full_weight_after_s'], 1)**2
        self.calls = 0

    def decode(self, variables):
        return np.vstack([self.b, compose(self.f[1:], exp(np.asarray(variables).reshape(-1, 3)))])

    def families(self, x):
        w = self.cfg['weights']; s = self.cfg['scales']; blocks = {}
        if w['relative']:
            blocks['relative'] = np.sqrt(w['relative'])*log(between(self.df, between(x[:-1], x[1:])))/self.rel_scale
        if w['intent']:
            blocks['intent'] = np.sqrt(w['intent']*self.intent_weight)[:, None]*log(between(self.f[1:], x[1:]))/self.intent_scale
        if w['correspondence'] and self.use_corr and len(self.corr['indices']):
            r = (x[self.corr['indices'], :2]-self.corr['old_xy'])/s['correspondence_m']
            delta = self.cfg['correspondence']['radial_pseudo_huber_delta']; z = np.sum(r*r, axis=1)/delta**2
            # Norm-isotropic pseudo-Huber, invariant under world rotation.
            robust = np.sqrt(2/(np.sqrt(1+z)+1))
            blocks['correspondence'] = (np.sqrt(w['correspondence']*self.corr['weights'])*robust)[:, None]*r
        if w['join'] and self.direction is not None:
            d = x[1, :2]-x[0, :2]
            angle = wrap(np.arctan2(d[1], d[0])-self.direction) if np.linalg.norm(d) > 1e-12 else np.pi
            blocks['join'] = np.array([np.sqrt(w['join'])*angle/np.radians(s['join_deg'])])
        return blocks

    def residual(self, variables):
        self.calls += 1
        blocks = self.families(self.decode(variables))
        return np.concatenate([v.ravel() for v in blocks.values()])

    def sparsity(self):
        dependencies = []; w = self.cfg['weights']; n = len(self.f)
        if w['relative']:
            for j in range(n-1):dependencies.extend([[j, j+1]]*3)
        if w['intent']:
            for j in range(1, n):dependencies.extend([[j]]*3)
        if w['correspondence'] and self.use_corr:
            for j in self.corr['indices']:dependencies.extend([[j]]*2)
        if w['join'] and self.direction is not None:dependencies.append([1])
        a = lil_matrix((len(dependencies), 3*(n-1)), dtype=int)
        for i, nodes in enumerate(dependencies):
            for j in nodes:
                if j: a[i, 3*(j-1):3*j] = 1
        return a.tocsr()


def breakdown(blocks):
    return {k:dict(residual_count=v.size, weighted_residuals=v.tolist(), half_squared_norm=float(.5*np.sum(v*v))) for k, v in blocks.items()}


def solve_graph(reference, boundary, times, corr, direction, config, use_correspondence=True, initial=None):
    start = perf_counter(); graph = None
    try:
        graph = Graph(reference, boundary, times, corr, direction, config, use_correspondence)
        initial = rigid_align(reference, boundary) if initial is None else np.asarray(initial, float)
        x0 = log(between(reference[1:], initial[1:])).ravel()
        initial_blocks = graph.families(graph.decode(x0)); initial_cost = sum(v['half_squared_norm'] for v in breakdown(initial_blocks).values())
        result = least_squares(graph.residual, x0, jac_sparsity=graph.sparsity(), x_scale='jac', **config['solver'])
        x = graph.decode(result.x); finite = bool(np.isfinite(x).all() and np.isfinite(result.fun).all())
        blocks = breakdown(graph.families(x))
        return dict(success=bool(result.success and finite), status=int(result.status), message=result.message,
                    poses=x if finite else None, nfev=int(result.nfev), njev=int(result.njev) if result.njev is not None else None,
                    iterations=None, iterations_note='SciPy least_squares 1.11 does not expose an iteration count; nfev/njev and actual callback calls reported.',
                    actual_residual_calls=graph.calls, runtime_ms=1000*(perf_counter()-start),
                    initial_objective=initial_cost, final_objective=float(result.cost), optimality=float(result.optimality),
                    initial_factors=breakdown(initial_blocks), final_factors=blocks, numerical_failure=not finite)
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
        return dict(success=False, status=-99, message=repr(exc), poses=None, nfev=None, njev=None, iterations=None,
                    actual_residual_calls=graph.calls if graph else 0, runtime_ms=1000*(perf_counter()-start),
                    initial_objective=None, final_objective=None, initial_factors={}, final_factors={}, numerical_failure=True)
