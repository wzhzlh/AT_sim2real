# ATDog robot descriptions

Local models for ATDog, ATDog2 and ATDog3. These packages are maintained in this project; the build does not fetch the upstream model zoo.

| Robot | URDF | MuJoCo scenes |
| --- | --- | --- |
| atdog | `urdf/dog.urdf` | `scene`, `scene_terrain` |
| atdog2 | `urdf/dog2.urdf` | `scene`, `scene_terrain`, `scene_tag` |
| atdog3 | `urdf/dog3.urdf` | `scene`, `scene_terrain`, `scene_tag` |

Meshes and dynamics are retained. Shared AprilTag scene assets live in `common/mjcf`.
ROS2 packages install the URDF, mesh and MJCF resources. See [workspace instructions](../../ATDOG_CLEANUP.md).
