The default elevation directory is now project-root `hgt/` (HGT and GeoTIFF).
This older directory remains usable with `--terrain-dir assets/terrain/hgt`.
Place user-supplied, uncompressed SRTM HGT tiles here, for example `N34W112.hgt`.
Supported dimensions are 1201 × 1201 or 3601 × 3601 signed big-endian int16 samples.
No elevation dataset is bundled or downloaded. Check the provider's attribution and
redistribution terms before adding data to a distributed build. Files can instead
remain outside this repository; use `--terrain-dir /path/to/hgt`.

See [terrain documentation](../../../docs/TERRAIN_LOD.md) for geographic origin,
height offsets, rendering limits, and collision limitations.
