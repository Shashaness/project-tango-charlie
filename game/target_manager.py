"""Living test-target selection; list indices are independent of persistent IDs."""

import math

import numpy as np


class TargetManager:
    """Maintain stable selected-target identity while cycling currently valid targets."""

    def __init__(self):
        """Initialize an empty living-target collection and no selection."""
        self.available = []
        self.current_target = None

    @staticmethod
    def valid(target):
        """Check target liveness and finite kinematics/collision radius.

        Args:
            target: Target entity, desired value, or configuration.

        Returns:
            Whether the target is eligible for selection.
        """
        return (
            target.alive
            and target.position.shape == (3,)
            and target.velocity.shape == (3,)
            and np.isfinite(target.position).all()
            and np.isfinite(target.velocity).all()
            and math.isfinite(target.radius)
            and target.radius > 0
        )

    @property
    def count(self):
        return len(self.available)

    @property
    def index(self):
        return (
            self.available.index(self.current_target) + 1
            if self.current_target in self.available
            else None
        )

    def refresh(self, targets):
        """Update available targets and retain or replace the current selection.

        Args:
            targets: Ordered target collection.
        """
        self.available = [target for target in targets if self.valid(target)]
        if self.current_target not in self.available:
            start = targets.index(self.current_target) + 1 if self.current_target in targets else 0
            self.current_target = next(
                (
                    targets[(start + offset) % len(targets)]
                    for offset in range(len(targets))
                    if targets[(start + offset) % len(targets)] in self.available
                ),
                None,
            )

    def cycle(self, targets):
        """Select the next valid target in collection order.

        Args:
            targets: Ordered target collection.

        Returns:
            Selected target, or None when the collection has no valid targets.
        """
        previous = self.current_target
        self.refresh(targets)
        if previous is None or previous not in self.available:
            return self.current_target
        self.current_target = self.available[
            (self.available.index(previous) + 1) % len(self.available)
        ]
        return self.current_target
