# Track geometry consistency

Online geometry uses only the periodic x(s)/y(s) spline. CSV psi and kappa are diagnostic cross-checks and CSV vx/ax are not race OCP references.

| Metric | Value |
|---|---:|
| Heading wrapped RMSE | 0.000035 rad |
| Heading max absolute difference | 0.000307 rad |
| Heading high-curvature RMSE | 0.000086 rad |
| Curvature RMSE | 0.001690 1/m |
| Curvature max absolute difference | 0.015004 1/m |
| Curvature high-curvature RMSE | 0.004177 1/m |
| Spline heading seam difference | 0.000e+00 rad |
| Spline curvature seam difference | 0.000e+00 1/m |

Plot: `../../plots/geometry_consistency.png`
