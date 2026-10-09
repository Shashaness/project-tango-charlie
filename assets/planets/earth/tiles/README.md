Geographic asset registry, with canonical one-degree directories such as
`N34W112/`. A cell may contain local HGT/GeoTIFF elevation files, discovered from
metadata, and an optional `manifest.json`:

```json
{
  "version": 1,
  "elevation": ["elevation.tif"],
  "cities": ["city.json"],
  "airports": ["airport.json"],
  "objects": ["objects.json"]
}
```

All entries are relative paths inside the cell. City/airport/object manifests are
registered but not instantiated by M20.5. Without an elevation list, directly
contained `.hgt`, `.tif`, and `.tiff` files are discovered. Legacy project-root
`hgt/` remains supported. No external datasets are included, copied, moved, or
downloaded. See [PLANETARY_WORLD.md](../../../../docs/PLANETARY_WORLD.md).
