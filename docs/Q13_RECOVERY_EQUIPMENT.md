# Q1.3A recovery equipment

Q002 extends the original access and equipment inspection objectives with ATAT-773 power restoration. Inspect the trolley, look for the service station and a compatible battery, bring the Jeep nearby, and use the rear cargo area to transfer it. Remove the depleted battery at the ATAT-773 bay, set it down in clear space, unload the replacement, install it and inspect the trolley to verify power. **O** cycles visible targets, **T** performs the displayed action and **Shift+T** inspects.

Hangar C uses the existing west-open hangar asset. In Q1.2C world coordinates the reference is **(300, 0, 4610) m**, navigation heading **180°**. The west entrance faces **270°**. The service station is at (306, 1, 4600) with the two industrial units on a low rack and a clear maneuvering bay to its west. The control tower had no authored placement in Arrival, so a provisional tower base is at (190, 0, 4270). Its card rests on the west-side shelf at (186.99, 1.035, 4269.8). The tower has no interior.

The maintenance card retains ID `office_key`. Its facility permission remains Hangar B. The physical `crane_access_card` grants eligibility for CRN-329 specifically. It can be found early and stored across sessions. The crane can be inspected in Q1.3A and authorization activation is deferred.

## Production handoff

`trolly_ATAT_773.blend` and `crane_CRN_329.blend` remain in `assets/source/`. Both supplied files contain a single combined mesh with no tagged mechanical hierarchy. Q1.3A uses scene placeholders. For production, copy each original into `assets/blender/` under the same filename and use the [Vehicle SDK](M25_1_VEHICLE_AUTHORING_SDK.md) to create a stable mechanical root and independent visual children. Author a trolley tow attachment, power tray, support, stabilizer and cargo restraint pivots. Author crane wheels, seat and access anchors, stabilizers, turntable, boom and hoist pivots. Export derived GLBs under `assets/models/vehicles/`. Preserve the current IDs when connecting their placements and interactions.

Crane control and operation, power-assisted stabilization, towing, and aircraft loading and securing belong to subsequent Q002 checkpoints. TC167 remains disabled and in place after power restoration.
