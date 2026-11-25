# M23.3 pilot rig standardization

## Diagnosis and deform rig

The old GLB encoded pivot translations and axis-aligned joint transforms,
without Blender edit-bone tails. On import, the display bones followed the
exported +Y axis. In the old Blender source, the left forearm head was at
1.16 m but its tail reached 1.361 m, toward the shoulder. The left thigh
started at 0.93 m and pointed up to 1.35 m. The next joint heads were below
both bones. This was an armature representation problem. The mesh weights
followed the pivots, which is why rig markers could look wrong even when
runtime animation bent in the intended direction. The old mesh also used
rigid weights at elbow and knee seams and a high thigh attachment.

The authoritative source remains `assets/blender/pilot.blend`. **Pilot
Deform** has 19 deform bones, with this hierarchy:

```text
Hips
├─ Spine > Chest > Neck > Head
│          ├─ Clavicle.L > UpperArm.L > Forearm.L > Hand.L
│          └─ Clavicle.R > UpperArm.R > Forearm.R > Hand.R
├─ Thigh.L > Shin.L > Foot.L
└─ Thigh.R > Shin.R > Foot.R
```

Hips is the pelvis and export root. Each clavicle starts at the upper
torso and ends at the shoulder. The upper arms and forearms point down
toward elbow and wrist. Thighs begin to either side of the pelvis and
lead to knees, ankles, and forward-pointing feet. Limb positions mirror
across the midline. At the elbow and knee seams, vertices share a small
amount of weight with the adjacent bone. The pilot retains the existing
segmented silhouette and materials.

Blender uses +Z up and the pilot faces −Y. The exported GLB uses +Y up
and +Z forward. Bone-local +Y goes from head to tail. To reason about
flexion, project the character's lateral +X into each bone’s bind basis.
Positive rotation about that axis bends the knee behind the thigh.
Negative rotation flexes the elbow forward. The four in-place Actions
were retargeted into the new local bases while preserving their clip
names, key times, loop closures, and limb phase.

## Rigify authoring

The installed Blender 5.0.1 distribution supplies Rigify 0.6.10. The
**Pilot Rigify Authoring** collection contains a fitted **Pilot Metarig**
from Rigify's Basic Human template and a generated **Pilot Rigify
Controls** armature. Rigify owns the generated limb IK/FK constraints and
widgets. **Pilot Asset** contains the skinned mesh and the separate runtime
deform rig. The pilot loads and animates in the game without the add-on.

1. Enable Rigify in Blender Preferences. Use the metarig to adjust the
   control reference or inspect the Basic Human limb construction.
2. Unhide **Pilot Rigify Controls** in the Outliner. Select **Pilot Deform**
   and set its **rigify_preview** object custom property to 1 to preview
   the control rig through the skinned mesh. Use the built-in controls in
   Pose Mode. The generated UI script may require trusted-file execution
   for its panels. The underlying hand/foot IK and FK controls can also
   be posed directly without loading that script.
3. Limb output drives local rotations on the deform bones. The torso and
   root remain editable directly on **Pilot Deform**. For a runtime clip,
   bake to an Action on that rig with **Pose** and **Visual Keying**,
   with **Clear Constraints** disabled. Turn **rigify_preview** back to
   0, select the baked Action, and inspect it in the Action Editor. Keep
   the existing four canonical names and keep motion in place.
4. Keep the metarig and generated controls outside **Pilot Asset**.
   If you regenerate controls, verify the deform preview targets still
   point to the generated object and its matching DEF limb bones.

The generated controls and limb transfer were tested in Blender. This
workflow uses limb rotation transfer. It does not automatically retarget
a full Rigify Action, torso motion, or root translation to the deform rig.
The original Rigify manual is useful background on metarigs and control
generation. [Blender Rigify manual](https://docs.blender.org/manual/en/2.81/addons/rigging/rigify.html)

## Export and validation

Use the CLI from the repository root:

```sh
BLENDER=/Applications/Blender.app/Contents/MacOS/Blender
"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/export_ground_assets.py -- pilot
"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/export_ground_assets.py -- pilot --check
"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/test_pilot_rig_blender.py
.venv/bin/python -m tools.validate_ground_assets pilot
.venv/bin/python -m unittest discover -s tests
```

The pilot exporter strips preview constraints and unrelated armatures
from its in-memory copy. Saved Blender contents are preserved. It exports
the skin, inverse binds, and four baked deform Actions to
`assets/models/characters/pilot.glb`. The byte comparison uses Blender 5.0.1.
An enabled Rigify preview is an export error. The validator checks rest
hierarchy, mirrored anatomical placement, bone axes, flexion direction,
arm and leg phase, and animation loops.

The Jeep pose is baked onto a separate skinning palette at runtime, with
the pelvis aligned to **DriverSeat**, forward thighs, feet directed toward
the footwell, and arms reaching forward. Its rotations use the anatomical
bind bases rather than assuming an axis-aligned rig. Exit still starts a
fresh locomotion session. The stylized mesh and fixed driver pose remain
simple. There is no procedural foot placement or hand-to-wheel IK in game.
