"""Session-only presentation of terrestrial campaign and interaction state."""

from collections import deque
from textwrap import wrap

from game.player_control import ControlMode
from game.quests.manager import Status

TABS = ("GENERAL", "QUESTS", "LOG")
# Fade in, hold, fade out, in seconds.
COMPLETION_TIMING = (0.0, 2.8, 0.7)
CONTEXT_TIMING = (0.2, 3.0, 0.6)
INSPECTION_TIMING = (0.2, 5.0, 0.6)
CONTEXT_HEADINGS = {
    "DOOR LOCKED - KEY REQUIRED": "ACCESS DENIED",
    "ACCESS DENIED": "ACCESS RESTRICTED",
    "OPENING OBSTRUCTED": "OBSTRUCTED",
    "OBJECT IS OUT OF REACH OR OBSTRUCTED": "INTERACTION BLOCKED",
    "ACTION UNAVAILABLE": "INTERACTION UNAVAILABLE",
}
DISCOVERIES = {
    "discover": "UTILITY JEEP DISCOVERED",
    "office": "MAINTENANCE OFFICE DISCOVERED",
    "key_seen": "KEY DISCOVERED",
    "panel": "GATE CONTROL DISCOVERED",
    "crane_seen": "CRANE DISCOVERED",
    "trolley_seen": "RECOVERY TROLLEY DISCOVERED",
}


def paginate(lines, columns, rows, page):
    expanded = [
        line
        for label in lines
        for line in (wrap(label, width=columns, break_on_hyphens=False) or [""])
    ]
    total = max(1, (len(expanded) + rows - 1) // rows)
    page %= total
    return expanded[page * rows : (page + 1) * rows], page + 1, total


def notification(card, age, timing):
    fade_in, hold, fade_out = timing
    remaining = sum(timing) - age
    if card is None or remaining <= 0:
        return None
    opacity = min(1.0, remaining / fade_out, age / fade_in if fade_in else 1.0)
    return (*card, opacity)


def visible_selection(campaign):
    selected = getattr(campaign, "selection", None)
    return selected if selected and campaign.objects.visible(selected) else None


def selection_lines(campaign, selected):
    action = campaign.objects.action(selected)
    controls = []
    if campaign.selection_position[1] > 1:
        controls.append("[O] NEXT OBJECT")
    if action != "INSPECT":
        controls.append("[SHIFT+T] INSPECT")
    if controls:
        return f"[T] {action}", "   ".join(controls)
    return (f"[T] {action}",)


class HelmetInformation:
    def __init__(self):
        self.tab = self.page = 0
        self.history = deque(maxlen=32)
        self.pending = deque(maxlen=8)
        self.toast = None
        self.started = 0.0
        self.elapsed = 0.0
        self.status = {}
        self.done = set()
        self.facts = set()
        self.feedback = set()
        self.context = None
        self.context_started = 0.0
        self.context_timing = CONTEXT_TIMING
        self.context_selection = None
        self.inspections = {}
        self.interaction_deadlines = (0.0, 0.0)

    def observe(self, campaign, control, command, elapsed):
        self.elapsed = elapsed
        if command.helmet_tab_pressed:
            self.tab = (self.tab + 1) % len(TABS)
            self.page = 0
        if command.helmet_page_down:
            self.page += 1
        if command.helmet_page_up:
            self.page -= 1
        if self.toast is not None and self.banner() is None:
            self.toast = None
        if campaign is not None:
            self._observe_campaign(campaign)
            self._observe_selection(campaign, command)
        feedback = [control.message] if control.message else []
        if command.interact_pressed and control.message:
            self._show_context("VEHICLE ACCESS", control.message)
        if campaign is not None:
            detail_until = getattr(campaign, "detail_until", 0)
            last_detail, last_inspection = self.interaction_deadlines
            self.interaction_deadlines = (detail_until, campaign.inspection_until)
            if campaign.inspection_until > campaign.time and campaign.can_inspect:
                from game.campaign import JEEP_PROFILE

                inspection = f"{JEEP_PROFILE.title} / {JEEP_PROFILE.condition}"
                feedback.append(inspection)
                if campaign.inspection_until != last_inspection:
                    self._show_inspection(
                        JEEP_PROFILE.title,
                        (JEEP_PROFILE.description, JEEP_PROFILE.condition, JEEP_PROFILE.actions),
                    )
            if detail_until > campaign.time:
                message = campaign.notice_detail
                feedback.append(message)
                if detail_until != last_detail and message:
                    inspected = getattr(campaign, "inspected_object", None)
                    selected = visible_selection(campaign)
                    heading = CONTEXT_HEADINGS.get(message, "INTERACTION")
                    if message.startswith("COLLECTED:"):
                        heading = "ITEM ACQUIRED"
                    elif message.startswith("DOOR UNLOCKED"):
                        heading = "ACCESS GRANTED"
                    if selected and inspected == selected.identity:
                        self.inspections[inspected] = message
                        self._show_inspection(
                            selected.spec["name"], (message, *selection_lines(campaign, selected))
                        )
                    else:
                        self._show_context(heading, message)
            if campaign.notice_until > campaign.time and not campaign.notice.startswith(
                ("COMPLETED:", "Q001 COMPLETE")
            ):
                feedback.append(campaign.notice)
        for message in feedback:
            if message and message not in self.feedback:
                self.history.append(message)
        self.feedback = set(feedback)
        if self.toast is None and self.pending:
            self.toast = self.pending.popleft()
            self.started = elapsed

    def _observe_campaign(self, campaign):
        manager = campaign.manager
        first = not self.status
        objectives, quests = [], []
        for fact in campaign.facts:
            if not first and fact not in self.facts and fact in DISCOVERIES:
                self.history.append(DISCOVERIES[fact])
        self.facts = set(campaign.facts)
        for key, definition in manager.definitions.items():
            state = manager.status[key]
            if state == Status.ACTIVE and self.status.get(key) != state:
                self.history.append(f"ACTIVE / {key} {definition.title}")
            for objective in definition.objectives:
                item = (key, objective.id)
                if manager.progress[key][objective.id] == objective.count and item not in self.done:
                    self.done.add(item)
                    if not first:
                        self.history.append(f"DONE / {objective.title}")
                        objectives.append(("OBJECTIVE COMPLETED", objective.title))
            if not first and state == Status.COMPLETED and self.status.get(key) != state:
                self.history.append(f"COMPLETED / {key} {definition.title}")
                quests.append((f"QUEST COMPLETED / {key}", definition.title))
            self.status[key] = state
        self.pending.extend(objectives)
        self.pending.extendleft(reversed(quests))

    def banner(self):
        return notification(self.toast, self.elapsed - self.started, COMPLETION_TIMING)

    def _observe_selection(self, campaign, command):
        selected = visible_selection(campaign)
        if not selected or selected.identity != self.context_selection:
            if self.context_selection is not None:
                self.context = self.context_selection = None
        if not selected:
            return
        if command.object_next_pressed:
            self.context_selection = selected.identity
            self.context_started = self.elapsed
            self.context_timing = CONTEXT_TIMING
        if self.context_selection == selected.identity:
            slot, count = campaign.selection_position
            lines = [selected.spec["name"]]
            if selected.locked:
                lines.append("LOCKED")
            elif "travel" in selected.spec:
                lines.append(selected.state)
            lines.extend(selection_lines(campaign, selected))
            self.context = (f"OBJECT SELECTED\t{slot} / {count}", "\n".join(lines))

    def _show_context(self, heading, message, *, timing=CONTEXT_TIMING, restart=False):
        card = heading, message
        if restart or card != self.context or self.context_banner() is None:
            self.context_started = self.elapsed
        self.context_selection = None
        self.context_timing = timing
        self.context = card

    def _show_inspection(self, name, lines):
        self._show_context(
            "OBJECT INSPECTION", "\n".join((name, *lines)), timing=INSPECTION_TIMING, restart=True
        )

    def context_banner(self):
        return notification(self.context, self.elapsed - self.context_started, self.context_timing)

    def content(self, control, campaign):
        if campaign is not None and campaign.journal_open:
            return "JOURNAL", campaign.hud_lines()
        if TABS[self.tab] == "LOG":
            return "LOG", tuple(self.history) or ("NO RECENT EVENTS",)
        if TABS[self.tab] == "QUESTS":
            return "QUESTS", self.quest_lines(campaign)
        return "GENERAL", self.general_lines(control, campaign)

    @staticmethod
    def quest_lines(campaign):
        if campaign is None:
            return ("NO CAMPAIGN QUESTS",)
        manager = campaign.manager
        lines = []
        for key, quest in manager.definitions.items():
            state = manager.status[key]
            if state not in (Status.ACTIVE, Status.COMPLETED):
                continue
            lines.append(f"{state.value} / {key} {quest.title}")
            ready = manager.ready(key)
            for objective in quest.objectives:
                if manager.progress[key][objective.id] == objective.count:
                    lines.append(f"DONE / {objective.title}")
                elif objective in ready:
                    tag = "CURRENT" if objective is ready[0] else "AVAILABLE"
                    lines.append(f"{tag} / {objective.title}")
        return tuple(lines) or ("NO ACTIVE OR COMPLETED QUESTS",)

    def general_lines(self, control, campaign):
        mode = "ON FOOT" if control.mode == ControlMode.ON_FOOT else "GROUND VEHICLE"
        prompt = control.prompt()
        lines = [mode]
        if prompt and prompt != mode:
            lines.append(prompt)
        if campaign is None:
            return lines
        manager = campaign.manager
        tracked = next(
            (q for key, q in manager.definitions.items() if manager.status[key] == Status.ACTIVE),
            None,
        )
        if tracked is not None:
            lines.extend((f"{tracked.id} / {tracked.title}", campaign.objective_label))
        elif campaign.objective_label:
            lines.append(campaign.objective_label)
        selected = visible_selection(campaign)
        if selected:
            slot, count = campaign.selection_position
            lines.append(f"{selected.spec['name']} / {slot}/{count}")
            action = campaign.objects.action(selected)
            lines.append(f"[T] {action}" + ("   [O] CYCLE OBJECT" if count > 1 else ""))
            if action != "INSPECT":
                lines.append("[SHIFT+T] INSPECT")
            if selected.identity in self.inspections:
                lines.append(self.inspections[selected.identity])
        elif campaign.can_inspect:
            lines.append("[T] INSPECT UTILITY JEEP")
        if getattr(campaign, "detail_until", 0) > campaign.time:
            lines.append(campaign.notice_detail)
        if campaign.inspection_until and campaign.can_inspect:
            from game.campaign import JEEP_PROFILE

            lines.extend((JEEP_PROFILE.description, JEEP_PROFILE.condition))
        if campaign.notice_until > campaign.time and not campaign.notice.startswith(
            ("COMPLETED:", "Q001 COMPLETE")
        ):
            lines.append(campaign.notice)
        lines.append("[I] JOURNAL   [U] HINT")
        if campaign.time < 12:
            from game.campaign import OPENING

            lines.extend(OPENING)
        return tuple(line for line in dict.fromkeys(lines) if line)
