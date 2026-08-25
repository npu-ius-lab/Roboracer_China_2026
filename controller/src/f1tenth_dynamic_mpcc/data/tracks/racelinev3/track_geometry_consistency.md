# Track geometry consistency

Online geometry uses only the periodic x(s)/y(s) spline. CSV psi and kappa are diagnostic cross-checks and CSV vx/ax are not race OCP references.

| Metric | Value |
|---|---:|
| Heading wrapped RMSE | 0.006608 rad |
| Heading max absolute difference | 0.023637 rad |
| Heading high-curvature RMSE | 0.011158 rad |
| Curvature RMSE | 0.020415 1/m |
| Curvature max absolute difference | 0.108935 1/m |
| Curvature high-curvature RMSE | 0.042740 1/m |
| Spline heading seam difference | 0.000e+00 rad |
| Spline curvature seam difference | 0.000e+00 1/m |

Plot: `../../plots/kappa_csv_vs_spline.png`
