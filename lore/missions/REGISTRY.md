# Mission development registry

These development statuses describe the implementation. Each campaign save tracks Kara's progress separately.

| Status | Meaning |
| --- | --- |
| PROPOSED | Under consideration. |
| PLANNED | Reserved for future development. |
| IN DEVELOPMENT | Implementation has started. |
| PARTIALLY IMPLEMENTED | A playable portion exists and requirements remain. |
| IMPLEMENTED | All intended gameplay exists and awaits verification. |
| VERIFIED | The implemented mission has passed end-to-end verification. |

| ID and title | Classification and assignee | Status and prerequisites | Purpose, systems, and remaining work |
| --- | --- | --- | --- |
| Q001 — Remnants of Civilization | Main campaign · Kara Voss | **VERIFIED** · Campaign start | Find and operate the Jeep and reach Hangar A. Arrival zones, discovery, inspection, occupancy and powered movement are playable. [Mission](Q001_remnants_of_civilization.md) |
| Q002 — The Means of Recovery | Main campaign · Kara Voss | **PARTIALLY IMPLEMENTED** · Q001 | Access Hangar B and inspect the equipment. Q1.3A adds the maintenance access card, Hangar C, physical battery transport, and ATAT-773 power restoration. Crane authorization and operation, towing, stabilization, and aircraft securing remain. [Mission](Q002_means_of_recovery.md) |
| Q003 — Bring Her Home | Main campaign · Kara Voss | **PLANNED** · Q002 | Recover TC167 and deliver it to the repair station. Requires towing, loading, securing, cargo transport and unloading. [Trolley](../technology/recovery_trolley.md) |
| Q004 — Against All Odds | Main campaign · Kara Voss | **PLANNED** · Q003 | Assess aircraft subsystem damage and establish a reconstruction project. Requires assessment and persistent repair requirements. [TC167](../technology/tc167.md) |
| Q005 — The Signal | Parallel narrative · Kara Voss | **PLANNED** · Campaign initialization and the transmission record | Review the transmission and establish monitoring. Requires communications equipment, monitoring and future investigation content. Its origin remains open. [First entry](../journals/kara_voss/001_first_entry.md) · [Mysteries](../MYSTERIES.md) |
| Q006 — Running on Borrowed Time | Personal equipment and survival · Kara Voss | **PLANNED** · Undetermined | Find and retain compatible helmet batteries. Requires power consumption, battery inventory, compatibility and replacement. [Mission](Q006_running_on_borrowed_time.md) |
| Q007 — A Source of Power | Personal equipment and infrastructure · Kara Voss | **PLANNED** · Undetermined | Establish reliable recharging. Requires electrical infrastructure, equipment restoration, charging, and persistent battery state. [Mission](Q007_a_source_of_power.md) |

The [Arrival quest definitions](../../assets/quests/arrival.json) contain executable Q001 and Q002. Update this registry whenever a mission is proposed, implemented, modified, or verified.
