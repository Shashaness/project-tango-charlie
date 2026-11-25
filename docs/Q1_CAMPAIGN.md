# Arrival campaign

```sh
python main.py --sandbox
python main.py --campaign
python main.py --campaign --campaign-save /path/to/arrival.json
python main.py --campaign --new-campaign
```

The default remains sandbox. Campaign mode loads a separate Arrival manifest and starts on foot beside a grounded TC167. TC167 is physically present and blocked from boarding. The campaign never routes controls into the aircraft's flight and transformation simulation. Sandbox reset and inspection presets remain sandbox features.

On the legacy layout, the maintenance hangar is about 540 m away and Q001 begins at the airbase. On the expanded layout, Q001 begins at the outlying compound. The sandbox layout and Jeep placement are separate.

- **T** inspects a discovered entity within reach. Discovery is automatic on foot and requires proximity and a clear sight line. Walls and other solid objects block it. The inspection panel shows the Jeep's description, apparent condition, and available actions.
- **Enter** boards or exits, **P** releases the parking brake, **WASD** drives, and **Space** brakes.
- **I** opens the journal, **N** advances its entries while open, and **U** cycles general hints. The HUD provides the current objective and transient completion messages. It uses no undiscovered entity waypoints.

Q001 requires all seven objectives and 12 m of actual powered Jeep movement by default. The `drive` objective's `distance_m` controls the threshold. The campaign measures horizontal travel from the driving simulation while Kara occupies the Jeep, applies throttle, and has wheel contact. Parking-brake and stationary checks prevent boarding or idling from counting. On legacy Q1.1 saves, Q002 becomes available and displays as awaiting future content.

## Quest schema and lifecycle

`assets/quests/arrival.json` contains `schema_version: 1` and a `quests` list. Each quest requires `id`, `title`, `description`, `owner`, `classification` (main or secondary), and `objectives`. Optional fields are `prerequisites`, `entities`, `activation`, `completion`, `journal` (initial, complete, and optional objective IDs), and `hints`. Follow-on quests declare the completed quest as a prerequisite.

Each objective requires an `id`, `title`, registered `type`, and manifest-entity `target`. Its optional `count`, `stage`, and `optional` fields default to 1, 0, and false. Required objectives in a stage can progress in parallel. The next stage unlocks after the previous required objectives finish. Optional objectives do not gate completion. The registry validates versions, fields, duplicate IDs, entity references, and cycles before initializing.

A quest is locked until its prerequisites complete and available until its activation condition is set. It then activates, and completes when its required objectives and completion condition are satisfied. Explicit failure is terminal. Recoverable world problems should remain in the gameplay system.

## Events and extension

A structured event includes a stable event ID, actor entity ID, target ID, position, simulation time and optional payload. A producer emits it only after a validated action or observation. Q001 producers use the approach zone, building interiors, unobstructed discovery and inspection, boarding, and actual Jeep motion. The campaign remembers one observation per objective with its original location and time. Valid early observations are delivered when their quest stage opens. The manager routes events by owner and stage, matches the objective event kind and target, and deduplicates by ID across saves. Producers that retry a logical event must reuse its ID.

Add an objective type by supplying a type-to-event mapping to `load_quests` and emitting that event from the validated gameplay service. Assignment and conditions use world identity and world state. A future autonomous character can use the same service and publish the same event without input or camera dependencies.

## Saves

The default local file is `saves/arrival.json`, which is ignored by Git. A missing file starts a new campaign. An existing compatible save resumes automatically. Use `--new-campaign` to explicitly replace the selected save. A malformed or incompatible file raises an error and remains available for inspection.

Legacy schema v2 stores the campaign identity, elapsed simulation time, Kara's position and facing, Jeep occupancy and motion state, aircraft position and subsystem health, and quest status, counts, event IDs, conditions, and journal references. It also persists exploration observations and accumulated powered travel. The journal uses stable template IDs and observation order, so a resume never generates a second copy of an entry. Save writes use a flushed temporary file and atomic replacement. The campaign saves at initialization, quest observations, occupancy changes, every ten seconds of simulation, and orderly window close. Use a separate save path per running session. Future schema or quest-definition changes need explicit migrations. Time advances while a campaign is running.

The v1 migration preserves arrival, player progress, damage, and the pose and occupancy of a Jeep already moved or occupied. A Jeep still stationary and parked at its original Q1.1 spawn adopts the authored garage spawn once during migration. V2 resume always uses the saved Jeep placement.

Inventory, recovery trolley, cargo, repair, and signal investigation belong to their designated submilestones. [Initial lore](../lore/world/initial_world_state.md) records the start scenario and open continuity questions.

## Q1.2A expansion and save compatibility

The expanded Arrival world introduced the four-building compound. In the Q1.2C layout, the crash is 89 m from the compound courtyard and 559 m from Hangar A. The nearest city boundary is 4.4 km from the wreck. The compound has four buildings and a paved route to the airport. Q001 now ends at the maintenance hangar after Kara has operated the Jeep and reached the airport. Q002 covers the office key, personnel access, gate opening, and equipment inspection. Q1.3A adds playable trolley power restoration. Towing remains a future objective.

Use **T** for the selected contextual action, **O** to cycle nearby objects, and **Shift+T** to inspect a selected object without changing its state. **I**, **N**, and **U** retain journal and hint functions. Unlock and open the personnel door with separate actions. The gate control is inside Hangar B. Both panels lift with their collisions and may reopen if their closing paths are occupied.

Schema v3 persists object positions along their animation, targets, lock flags, key ownership and discovery, control use, journal observations, and campaign state. The loader retains v1/v2 saves on the Q1.1/Q1.2 geography and quest definitions, preserving actors and completed history. To play the new geography while retaining the old file, run `python main.py --campaign --campaign-save saves/arrival-expanded.json`. Explicit `--new-campaign` replaces the selected save. See the [interactive asset requirements](Q12A_ASSET_REQUIREMENTS.md).

New saves use the Q1.2C layout. Existing Q1.2A and Q1.2B saves retain their geographical revision and saved actor positions. Q1.3A adds recovery equipment to the corresponding airport. To start separately, run `python main.py --campaign --campaign-save saves/arrival-q12c.json`. See the [Arrival layout map](ARRIVAL_WORLD_LAYOUT.md) for measured placements and a separate-save launch command.

## Navigation and survey controls

The helmet HUD is visible on foot and in ground vehicles. Press **F6** to append a survey marker. See [NAV1 Terrestrial Navigation](NAV1_TERRESTRIAL_NAVIGATION.md) for coordinate references and file details.

When proposing, implementing, modifying, or verifying missions, update the [mission development registry](../lore/missions/REGISTRY.md) alongside the applicable quest data and narrative documents.

## Terrestrial information panel

**F7** cycles **General**, **Quests**, and **Log** on foot and in ground vehicles. **Page Up** and **Page Down** turn panel pages when content overflows. General shows the current objective, nearby interaction, and relevant controls. Quests shows available and completed objectives from the campaign. Log shows recent feedback and observations from this session. Quest and objective completion banners appear briefly below the compass.

**I** still opens Kara's narrative journal and **N** advances its entries. The Log's short feedback history is session-specific. Campaign progress and narrative journal entries still use the existing save. **T**, **O**, **Shift+T**, **U**, and **F6** retain their previous functions.

Interaction results briefly appear at the upper left and remain available through the existing General feedback and session Log. The upper-left card stacks below an active completion banner when both cards need the same horizontal space.

**O** also shows the selected object, its position in the nearby list, and the current action in the upper-left card. **Shift+T**, or **T** when the current action is Inspect, shows the inspection result there. General keeps the current action and any inspection result for the selected object after the card fades. See the [terrestrial interface reference](HUD1_TERRESTRIAL_INTERFACE.md).

## Q1.3A battery recovery

After the Hangar B access and equipment objectives, inspect ATAT-773 and use the Jeep to bring a compatible battery from Hangar C. The original power unit remains a physical object after removal. Verify the restored controls by inspecting the trolley. Q002 remains active for later crane and recovery preparation. The crane card can be collected early at the control tower without activating crane operation.

Expanded saves migrate from schema 3 to 4 when next written. Existing actors, inventory, and geographical layout IDs carry forward. See [recovery equipment](Q13_RECOVERY_EQUIPMENT.md), [power](PROTONIC_ENERGY_ARCHITECTURE.md), and [cargo](PHYSICAL_CARGO_ARCHITECTURE.md).
