# LighterBEV weight profiles and descriptor caches

The vehicle keeps two independent localization profiles:

- `original`: `models/pca_kitti_best.pt`
- `mid360-finetuned`: `models/pca_mid360_finetuned.pt`

Select one when starting the vehicle stack:

```bash
./start_mid360_tianracer_localization.sh --bev-model mid360-finetuned
./start_mid360_tianracer_localization.sh --bev-model original
```

The model is loaded once at process startup, so changing profiles requires a
normal localization restart. The start script refuses to claim a switch when
a different profile is already running.

Descriptor databases are deliberately isolated:

- original: `lighterbev_descriptors.bin`
- MID360 fine-tuned: `lighterbev_descriptors_pca_mid360_finetuned.bin`

The vehicle start script selects the model and cache as one profile, verifies
both SHA256 values, and passes the cache path explicitly to the localization
node. A mismatched or modified pair is rejected before any node is started.

The cache contains map descriptors only. Runtime BEV/REIN input remains one
current lidar frame for both profiles.
