"""Metadata-only planetary asset registry. Manifests are data, never executable."""
from dataclasses import dataclass
from pathlib import Path
import json
from game.geography import GeographicTile

@dataclass(frozen=True)
class GeographicAssets:
    tile: GeographicTile
    elevation: tuple = ()
    cities: tuple = ()
    airports: tuple = ()
    objects: tuple = ()

class GeographicRegistry:
    def __init__(self, directory, legacy_directory=None):
        self.directory=Path(directory);self.cells={};self.warnings=[]
        elevation=[]
        for cell in sorted(self.directory.glob('*')):
            if not cell.is_dir(): continue
            try:
                tile=GeographicTile.parse(cell.name)
                manifest=cell/'manifest.json'
                if manifest.exists():
                    if manifest.stat().st_size>1024*1024: raise ValueError('Manifest exceeds 1 MiB')
                    value=json.loads(manifest.read_text())
                    if not isinstance(value,dict) or value.get('version',1)!=1: raise ValueError('Expected version 1 object manifest')
                else: value={}
                def resolve(kind):
                    entries=value.get(kind,[])
                    if not isinstance(entries,list) or any(not isinstance(v,str) for v in entries): raise ValueError(f'{kind} must be a list of relative paths')
                    paths=[]
                    for entry in entries:
                        path=(cell/entry).resolve()
                        if not path.is_relative_to(cell.resolve()): raise ValueError('Manifest paths must stay within the geographic cell')
                        if not path.is_file(): self.warnings.append(f'{tile.identifier}: missing {kind} asset {entry}'); continue
                        paths.append(path)
                    return tuple(paths)
                sources=resolve('elevation') if 'elevation' in value else tuple(sorted(p.resolve() for p in cell.glob('*') if p.suffix.lower() in ('.hgt','.tif','.tiff')))
                assets=GeographicAssets(tile,sources,resolve('cities'),resolve('airports'),resolve('objects'))
                self.cells[tile]=assets;elevation.extend(sources)
            except (ValueError,OSError,json.JSONDecodeError) as error:
                self.warnings.append(f'{cell.name}: {error}')
        if legacy_directory is not None:
            elevation.extend(p.resolve() for p in sorted(Path(legacy_directory).glob('*')) if p.suffix.lower() in ('.hgt','.tif','.tiff'))
        self.elevation_paths=tuple(dict.fromkeys(elevation))
    def assets_at(self, latitude, longitude):
        tile=GeographicTile.at(latitude,longitude)
        return self.cells.get(tile,GeographicAssets(tile))
