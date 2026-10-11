"""Independent saved-chunk fits in observation-local forward/left metres.

Times and derivatives use the model's nominal target-time convention. They are
not native timestamps, velocity predictions, robot yaw, or an execution model.
"""
import warnings
import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import UnivariateSpline
from research.route_geometry import rotation, wrap, interpolate


def methods(config):
    return [('RAW', None), *[(f'SPLINE_{round(s*100):02d}cm', s) for s in config['spline_budgets_m']], ('POLY3_DIAGNOSTIC', None)]


class Curve:
    """A fit never receives another chunk, an observation pose, or switch B."""
    def __init__(self, native, method, budget, config):
        self.native = np.array(native, dtype=float, copy=True)
        if self.native.shape != (config['expected_points'], 2) or not np.isfinite(self.native).all():
            raise ValueError('Expected finite native (30,2), no synthetic replacement')
        self.times = config['native_dt_s']*np.arange(1, len(native)+1)
        self.method, self.config, self.budget = method, config, budget
        self.models = []
        self.info = dict(method=method, budget_m=budget, domain_s=[float(self.times[0]), float(self.times[-1])], status='PASS', reason=None)
        if method == 'RAW':
            return
        with warnings.catch_warnings(record=True) as captured:
            warnings.simplefilter('always')
            if method.startswith('SPLINE'):
                s = len(native)*budget**2
                self.models = [UnivariateSpline(self.times, self.native[:, j], w=np.ones(len(native)), k=3, s=s, ext=2, check_finite=True) for j in range(2)]
                self.info.update(s_per_coordinate_m2=s, achieved_ssr_m2=[float(m.get_residual()) for m in self.models],
                    coefficients=[m.get_coeffs().tolist() for m in self.models], knots=[m.get_knots().tolist() for m in self.models], degree=3)
            elif method == 'POLY3_DIAGNOSTIC':
                self.models = [Polynomial.fit(self.times, self.native[:, j], deg=config['polynomial_degree']).convert() for j in range(2)]
                self.info.update(degree=config['polynomial_degree'], power_coefficients=[m.coef.tolist() for m in self.models])
            else:
                raise ValueError('Unknown frozen fitting method')
        if captured:
            raise ValueError('Spline/polynomial fitting warning: '+'; '.join(str(w.message) for w in captured))
        if not np.isfinite(self.value(self.times)).all():
            raise ValueError('Nonfinite fitted coordinates')

    def value(self, query, derivative=0, allow_bridge=False):
        scalar = np.ndim(query) == 0
        q = np.atleast_1d(np.asarray(query, float)).copy()
        lo, hi = self.times[0], self.times[-1]
        lower = 0. if allow_bridge else lo
        eps = self.config['time_roundoff_tolerance_s']
        if not np.isfinite(q).all() or (q < lower-eps).any() or (q > hi+eps).any():
            raise ValueError('No temporal extrapolation')
        q = np.clip(q, lower, hi)  # Only floating-point endpoint roundoff.
        bridge = q < lo
        main = np.maximum(q, lo)
        if self.method == 'RAW':
            if derivative == 0:
                out = interpolate(self.times, self.native, main)
            elif derivative == 1:
                idx = np.minimum(np.searchsorted(self.times, main, side='right')-1, len(self.times)-2)
                out = np.diff(self.native, axis=0)[idx]/np.diff(self.times)[idx, None]
            else:
                out = np.full((len(q), 2), np.nan)  # No invented second derivative at raw knots.
        elif self.method.startswith('SPLINE'):
            out = np.column_stack([m(main, nu=derivative, ext=2) for m in self.models])
        else:
            out = np.column_stack([m.deriv(derivative)(main) for m in self.models])
        if bridge.any():
            first = self.value(lo)
            out[bridge] = q[bridge, None]/lo*first if derivative == 0 else first/lo if derivative == 1 else 0.
        return out[0] if scalar else out

    def world(self, query, pose, derivative=0, allow_bridge=False):
        out = self.value(query, derivative, allow_bridge) @ rotation(pose[2]).T
        return out + np.asarray(pose[:2]) if derivative == 0 else out


def tangent(derivative, minimum_speed):
    d = np.asarray(derivative, float)
    speed = np.linalg.norm(d, axis=-1)
    valid = np.isfinite(d).all(axis=-1) & (speed >= minimum_speed)
    unit = np.full_like(d, np.nan)
    np.divide(d, speed[..., None], out=unit, where=valid[..., None])
    return unit, valid, speed


def project_residual(residual, derivative, minimum_speed):
    unit, valid, speed = tangent(derivative, minimum_speed)
    normal = np.stack([-unit[..., 1], unit[..., 0]], axis=-1)
    return np.sum(residual*unit, axis=-1), np.sum(residual*normal, axis=-1), valid, speed


def rms(a):
    a = np.asarray(a, float); a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a*a))) if len(a) else None


def mad(a):
    a = np.asarray(a, float); a = a[np.isfinite(a)]
    return float(1.4826*np.median(np.abs(a-np.median(a)))) if len(a) else None


def angle_gap(a, b):
    return np.abs(np.degrees(wrap(np.asarray(a)-np.asarray(b))))


def chord_basis(raw_curve, queries, policy):
    """Same raw FRESH temporal chord basis for all methods; native domain only."""
    heads, diagnostics = [], []
    for t in queries:
        lo = max(raw_curve.times[0], t-policy['half_window_s'])
        hi = min(raw_curve.times[-1], t+policy['half_window_s'])
        q = np.unique(np.r_[lo, raw_curve.times[(raw_curve.times > lo)&(raw_curve.times < hi)], hi])
        p = raw_curve.value(q); d = p[-1]-p[0]
        length = np.linalg.norm(d); arc = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
        valid = length >= policy['minimum_chord_m'] and arc > 0 and length/arc >= policy['minimum_coherence']
        heads.append(np.arctan2(d[1], d[0]) if valid else np.nan)
        diagnostics.append(dict(valid=bool(valid), chord_m=float(length), arc_m=float(arc)))
    heads = np.array(heads)
    return np.column_stack([np.cos(heads), np.sin(heads)]), diagnostics


def chunk_metrics(curve, raw, config):
    q = curve.times; p = curve.value(q); dp = curve.value(q, 1); ddp = curve.value(q, 2)
    residual = raw.native-p
    rf, rl, valid, speed = project_residual(residual, dp, config['minimum_derivative_m_per_nominal_s'])
    xy_error = np.linalg.norm(residual, axis=1)
    grid = np.linspace(q[0], q[-1], round((q[-1]-q[0])/config['dense_dt_s'])+1)
    dense = curve.value(grid); d1 = curve.value(grid, 1); d2 = curve.value(grid, 2)
    _, dense_valid, dense_speed = tangent(d1, config['minimum_derivative_m_per_nominal_s'])
    raw_segments = np.diff(raw.native, axis=0); fit_segments = np.diff(p, axis=0)
    raw_len = np.linalg.norm(raw_segments, axis=1); fit_len = np.linalg.norm(fit_segments, axis=1)
    common = (raw_len >= .01) & (fit_len >= .01)
    a = np.arctan2(raw_segments[:, 1], raw_segments[:, 0]); b = np.arctan2(fit_segments[:, 1], fit_segments[:, 0])
    tv = common[1:] & common[:-1]
    fitted_arc = float(np.linalg.norm(np.diff(dense, axis=0), axis=1).sum()); native_arc = float(raw_len.sum())
    row = dict(forward_rmse_m=rms(rf), lateral_rmse_m=rms(rl), forward_mad_m=mad(rf), lateral_mad_m=mad(rl),
        max_abs_forward_m=float(np.nanmax(np.abs(rf))) if valid.any() else None,
        max_abs_lateral_m=float(np.nanmax(np.abs(rl))) if valid.any() else None,
        xy_rmse_m=rms(xy_error), max_waypoint_displacement_m=float(xy_error.max()), endpoint_displacement_m=float(xy_error[-1]),
        reliable_tangent_samples=int(valid.sum()), invalid_tangent_samples=int((~valid).sum()),
        native_arc_length_m=native_arc, fitted_arc_length_m=fitted_arc, arc_length_change_m=fitted_arc-native_arc,
        relative_arc_length_change=(fitted_arc-native_arc)/native_arc if native_arc > 1e-8 else None,
        waypoint_segment_heading_change_rmse_deg=rms(angle_gap(a[common], b[common])),
        waypoint_turn_shape_change_rmse_deg=rms(angle_gap(wrap(np.diff(a))[tv], wrap(np.diff(b))[tv])),
        reliable_shape_segments=int(common.sum()), derivative_min_m_per_nominal_s=float(speed.min()),
        dense_invalid_derivatives=int((~dense_valid).sum()), finite_second_derivatives=int((np.isfinite(d2).all(axis=1)&dense_valid).sum()),
        raw_backward_forward_segments=int((raw_segments[:, 0]<0).sum()),
        fitted_backward_forward_segments=int((fit_segments[:, 0]<0).sum()))
    detail = dict(fit=curve.info, nominal_times=q, fitted_local_xy=p, residual_local_xy=residual,
        derivative_local=dp, second_derivative_local=ddp, residual_forward=rf, residual_lateral=rl,
        tangent_valid=valid, grid_nominal_times=grid, grid_local_xy=dense,
        grid_derivative_local=d1, grid_second_derivative_local=d2, grid_derivative_valid=dense_valid,
        grid_speed_m_per_nominal_s=dense_speed)
    return row, detail
