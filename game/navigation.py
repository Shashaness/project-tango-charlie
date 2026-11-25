"""Terrestrial navigation and append-only local survey records."""

import csv
import math
import os
from dataclasses import dataclass

from engine.asset_paths import PROJECT_ROOT
from game.player_control import ControlMode

FIELDS = (
    "marker_id",
    "world_id",
    "layout_revision",
    "campaign_id",
    "position_ref",
    "x_m",
    "y_m",
    "z_m",
    "heading_deg",
    "simulation_time_s",
)
DIRECTIONS = dict(enumerate(("N", "NE", "E", "SE", "S", "SW", "W", "NW")))


def heading(forward):
    """World-grid bearing clockwise from −Z, independent of the geographic frame."""
    east, _, south = forward
    if not math.isfinite(east) or not math.isfinite(south) or math.hypot(east, south) < 1e-6:
        return None
    return math.degrees(math.atan2(east, -south)) % 360 % 360


def tape(bearing, span=60):
    for mark in range(
        math.ceil((bearing - span) / 5) * 5, math.floor((bearing + span) / 5) * 5 + 1, 5
    ):
        degree = mark % 360
        label = (
            DIRECTIONS[degree // 45]
            if degree % 45 == 0
            else (f"{degree:03d}" if degree % 30 == 0 else "")
        )
        yield mark - bearing, label


@dataclass(frozen=True)
class Fix:
    position: tuple
    heading: float | None
    reference: str


def read_fix(control):
    if control.mode is ControlMode.ON_FOOT:
        actor, reference = control.pilot, "person"
    elif control.mode is ControlMode.GROUND_VEHICLE:
        actor, reference = control.active.vehicle, "vehicle"
    else:
        return None
    return Fix(tuple(float(v) for v in actor.position), heading(actor.forward), reference)


def append_marker(path, fix, world, layout, campaign, time):
    import fcntl

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", encoding="utf-8", newline="") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        stream.seek(0)
        rows = csv.reader(stream, strict=True)
        header = next(rows, None)
        marker = 1
        if header is not None:
            if tuple(header) != FIELDS:
                raise ValueError("Unexpected survey CSV header")
            for row in rows:
                if len(row) != len(FIELDS) or row[0] != f"{marker:03d}":
                    raise ValueError("Invalid survey marker sequence")
                marker += 1
        stream.seek(0, os.SEEK_END)
        writer = csv.writer(stream)
        if header is None:
            writer.writerow(FIELDS)
        elif stream.tell():
            # Reject an unterminated record instead of concatenating two rows.
            with path.open("rb") as check:
                check.seek(-1, os.SEEK_END)
                if check.read(1) != b"\n":
                    raise ValueError("Incomplete final survey record")
        writer.writerow(
            (
                f"{marker:03d}",
                world,
                layout,
                campaign,
                fix.reference,
                *(f"{value:.9f}" for value in fix.position),
                "" if fix.heading is None else f"{round(fix.heading, 9) % 360:.9f}",
                f"{time:.9f}",
            )
        )
        stream.flush()
        os.fsync(stream.fileno())
    return marker


class Navigation:
    def __init__(self, control, world, campaign, directory=None):
        self.control = control
        self.world = world
        self.campaign = campaign
        self.directory = directory or PROJECT_ROOT / "surveys"
        self.elapsed = 0.0
        self.notice = ""
        self.notice_until = 0.0

    @property
    def layout(self):
        if self.campaign is None:
            return "sandbox-v1" if self.world == "tango_charlie" else "manifest-v1"
        return getattr(self.campaign, "layout_revision", "q12")

    def snapshot(self):
        notice = self.notice if self.elapsed < self.notice_until else ""
        return read_fix(self.control), notice

    def record(self):
        fix = read_fix(self.control)
        if fix is None:
            return
        path = self.directory / (
            "arrival_locations.csv" if self.campaign else "sandbox_locations.csv"
        )
        try:
            marker = append_marker(
                path,
                fix,
                self.world,
                self.layout,
                self.campaign.definition.identity if self.campaign else "",
                self.campaign.time if self.campaign else self.elapsed,
            )
            self.notice = f"LOCATION MARKER {marker:03d} RECORDED"
        except (OSError, ValueError, csv.Error, ImportError):
            self.notice = "SURVEY RECORDING FAILED"
        self.notice_until = self.elapsed + 3
