"""Small robust ridge residual model with a deployment-friendly YAML format."""

from pathlib import Path

import numpy as np
import yaml


OUTPUT_NAMES = ("vx_accel_residual", "vy_accel_residual", "yaw_accel_residual")


def _weighted_ridge(x, y, weights, alpha):
    root = np.sqrt(np.maximum(weights, 1.0e-8))[:, None]
    xw, yw = x * root, y * root
    regularizer = alpha * np.eye(x.shape[1])
    regularizer[0, 0] = 0.0
    return np.linalg.solve(xw.T @ xw + regularizer, xw.T @ yw)


class ResidualModel:
    def __init__(self, feature_names, mean, scale, coefficients, output_limits,
                 feature_abs_z_limit=None, ood_fade_ratio=1.5, metadata=None):
        self.feature_names = list(feature_names)
        self.mean = np.asarray(mean, dtype=float)
        self.scale = np.asarray(scale, dtype=float)
        self.coefficients = np.asarray(coefficients, dtype=float)
        self.output_limits = np.asarray(output_limits, dtype=float)
        self.feature_abs_z_limit = np.asarray(
            feature_abs_z_limit if feature_abs_z_limit is not None
            else np.full(len(self.feature_names), np.inf), dtype=float)
        self.ood_fade_ratio = float(ood_fade_ratio)
        self.metadata = metadata or {}

    @classmethod
    def fit(cls, x, y, feature_names, alpha=4.0, robust_iterations=4,
            huber_delta=2.5, output_limits=(4.0, 8.0, 18.0), metadata=None):
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        mean = np.mean(x, axis=0)
        scale = np.std(x, axis=0)
        mean[0], scale[scale < 1.0e-8] = 0.0, 1.0
        normalized = (x - mean) / scale
        weights = np.ones(len(x))
        coefficients = None
        for _ in range(max(1, int(robust_iterations))):
            coefficients = _weighted_ridge(normalized, y, weights, float(alpha))
            errors = y - normalized @ coefficients
            magnitude = np.sqrt(np.mean(errors * errors, axis=1))
            median = np.median(magnitude)
            sigma = 1.4826 * np.median(np.abs(magnitude - median)) + 1.0e-9
            threshold = float(huber_delta) * sigma
            weights = np.minimum(1.0, threshold / np.maximum(magnitude, 1.0e-9))
        normalized_abs = np.abs(normalized)
        envelope = np.maximum(np.percentile(normalized_abs, 99.5, axis=0), 1.0)
        envelope[0] = max(envelope[0], 1.0)
        return cls(feature_names, mean, scale, coefficients, output_limits,
                   envelope, 1.5, metadata)

    def confidence(self, features):
        features = np.asarray(features, dtype=float)
        normalized = np.abs((features - self.mean) / self.scale)
        ratio = np.max(normalized / self.feature_abs_z_limit, axis=-1)
        return np.clip((self.ood_fade_ratio - ratio) /
                       max(self.ood_fade_ratio - 1.0, 1.0e-6), 0.0, 1.0)

    def predict(self, features):
        features = np.asarray(features, dtype=float)
        prediction = ((features - self.mean) / self.scale) @ self.coefficients
        prediction = np.clip(prediction, -self.output_limits, self.output_limits)
        confidence = self.confidence(features)
        return prediction * np.expand_dims(confidence, axis=-1)

    def save(self, path):
        payload = {
            "schema_version": 1,
            "model_type": "robust_ridge_derivative_residual",
            "feature_names": self.feature_names,
            "output_names": list(OUTPUT_NAMES),
            "feature_mean": self.mean.tolist(),
            "feature_scale": self.scale.tolist(),
            "coefficients_feature_by_output": self.coefficients.tolist(),
            "output_limits": self.output_limits.tolist(),
            "feature_abs_z_limit": self.feature_abs_z_limit.tolist(),
            "ood_fade_ratio": self.ood_fade_ratio,
            "metadata": self.metadata,
        }
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as stream:
            yaml.safe_dump(payload, stream, sort_keys=False)

    @classmethod
    def load(cls, path):
        with Path(path).open("r") as stream:
            payload = yaml.safe_load(stream)
        if payload.get("schema_version") != 1:
            raise ValueError("unsupported residual model schema")
        return cls(payload["feature_names"], payload["feature_mean"],
                   payload["feature_scale"], payload["coefficients_feature_by_output"],
                   payload["output_limits"], payload.get("feature_abs_z_limit"),
                   payload.get("ood_fade_ratio", 1.5), payload.get("metadata", {}))
