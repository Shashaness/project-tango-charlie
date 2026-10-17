"""Continuous geographic relief and real-source confidence blending."""

import numpy as np

from game.geography import GeographicTile


class ProceduralElevation:
    """Generate seeded continuous planetary relief from noise on unit-sphere coordinates."""

    def __init__(self, seed=205):
        """Retain the integer seed for deterministic spherical relief.

        Args:
            seed: Integer seed for deterministic variation.
        """
        self.seed = int(seed)

    def _noise(self, points):
        """Interpolate deterministic hashed lattice values with quintic smoothing.

        Args:
            points: Query point array in the coordinate system used by the operation.

        Returns:
            Noise array with the final XYZ input axis removed.
        """
        base = np.floor(points).astype(np.int64)
        fraction = points - base
        # Quintic interpolation has zero first/second derivatives at lattice edges,
        # keeping octave relief smooth without a stored global height raster.
        smooth = fraction**3 * (10 + fraction * (-15 + 6 * fraction))
        result = np.zeros(points.shape[:-1])
        for x in (0, 1):
            for y in (0, 1):
                for z in (0, 1):
                    corner = (base + np.array([x, y, z])).astype(np.uint64) & np.uint64(0xFFFFFFFF)
                    h = (
                        (corner[..., 0] * np.uint64(73856093))
                        ^ (corner[..., 1] * np.uint64(19349663))
                        ^ (corner[..., 2] * np.uint64(83492791))
                        ^ np.uint64(self.seed & 0xFFFFFFFF)
                    )
                    h ^= h >> np.uint64(13)
                    h &= np.uint64(0xFFFFFFFF)
                    h *= np.uint64(1274126177)
                    h ^= h >> np.uint64(16)
                    value = (h & np.uint64(0xFFFFFFFF)).astype(float) / 2147483647.5 - 1
                    weight = np.ones(result.shape)
                    for axis, offset in enumerate((x, y, z)):
                        weight *= smooth[..., axis] if offset else 1 - smooth[..., axis]
                    result += value * weight
        return result

    def sample(self, latitude, longitude):
        """Evaluate geographic elevation using the configured source and blending rules.

        Args:
            latitude: Geographic latitude(s) in degrees.
            longitude: Geographic longitude(s) in degrees.

        Returns:
            Broadcast elevation array in source-reference meters.
        """
        latitude, longitude = np.broadcast_arrays(
            np.asarray(latitude, float), np.asarray(longitude, float)
        )
        if (
            not np.isfinite(latitude).all()
            or not np.isfinite(longitude).all()
            or np.any(np.abs(latitude) > 90)
        ):
            raise ValueError("Invalid procedural geographic coordinates")
        phi = np.radians(latitude)
        theta = np.radians((longitude + 180) % 360 - 180)
        # Noise is evaluated on the sphere: the dateline shares one coordinate,
        # and longitude cannot introduce distinct values at the poles.
        cosine = np.where(np.abs(latitude) == 90, 0.0, np.cos(phi))
        direction = np.stack((cosine * np.cos(theta), cosine * np.sin(theta), np.sin(phi)), axis=-1)
        result = np.full(latitude.shape, 1200.0)
        for frequency, amplitude in ((64, 900), (128, 450), (256, 200), (512, 80)):
            noise = self._noise(direction * frequency)
            result += amplitude * noise
            if frequency == 256:
                result += 350 * (1 - np.abs(noise)) ** 2
        return result


class PlanetaryElevation:
    """Blend procedural relief with available real elevation and void confidence."""

    def __init__(self, dataset, projection, seed=205, blend_width=750.0):
        """Bind real sources and procedural relief with a geographic blend width.

        Args:
            dataset: Bounded real-elevation source collection.
            projection: Regional geographic projection.
            seed: Integer seed for deterministic variation.
            blend_width: Transition band width in meters.
        """
        self.dataset = dataset
        self.projection = projection
        self.procedural = ProceduralElevation(seed)
        self.blend_width = blend_width

    def sample(self, latitude, longitude):
        """Evaluate geographic elevation using the configured source and blending rules.

        Args:
            latitude: Geographic latitude(s) in degrees.
            longitude: Geographic longitude(s) in degrees.

        Returns:
            Broadcast elevation array in source-reference meters.
        """
        latitude, longitude = np.broadcast_arrays(
            np.asarray(latitude, float), np.asarray(longitude, float)
        )
        procedural = self.procedural.sample(latitude, longitude)
        real = self.dataset.sample(latitude, longitude)
        weight = self.dataset.coverage_weight(
            latitude,
            longitude,
            self.projection.east_scale,
            self.projection.north_scale,
            self.blend_width,
        )
        weight *= self.dataset.confidence(
            latitude,
            longitude,
            self.projection.east_scale,
            self.projection.north_scale,
            self.blend_width,
        )
        weight = np.where(np.isfinite(real), weight, 0.0)
        return procedural * (1 - weight) + np.where(np.isfinite(real), real, 0) * weight

    def source_at(self, latitude, longitude):
        """Classify the effective elevation source at one geographic point.

        Args:
            latitude: Geographic latitude(s) in degrees.
            longitude: Geographic longitude(s) in degrees.

        Returns:
            PROCEDURAL, BLEND, or SRTM source label.
        """
        real = float(self.dataset.sample(latitude, longitude))
        if not np.isfinite(real):
            return "PROCEDURAL"
        weight = float(
            self.dataset.coverage_weight(
                latitude,
                longitude,
                self.projection.east_scale,
                self.projection.north_scale,
                self.blend_width,
            )
        )
        weight *= float(
            self.dataset.confidence(
                latitude,
                longitude,
                self.projection.east_scale,
                self.projection.north_scale,
                self.blend_width,
            )
        )
        return "SRTM" if weight >= 1 - 1e-8 else "BLEND"
