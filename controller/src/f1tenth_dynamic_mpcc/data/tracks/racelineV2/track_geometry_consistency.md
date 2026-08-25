# Track geometry consistency

Online geometry uses only the periodic x(s)/y(s) spline. CSV psi and kappa are diagnostic cross-checks and CSV vx/ax are not race OCP references.

| Metric | Value |
|---|---:|
| Heading wrapped RMSE | 0.007351 rad |
| Heading max absolute difference | 0.024080 rad |
| Heading high-curvature RMSE | 0.011491 rad |
| Curvature RMSE | 0.025058 1/m |
| Curvature max absolute difference | 0.123640 1/m |
| Curvature high-curvature RMSE | 0.043286 1/m |
| Spline heading seam difference | 0.000e+00 rad |
| Spline curvature seam difference | 0.000e+00 1/m |

Plot: `../../plots/kappa_csv_vs_spline.png`
