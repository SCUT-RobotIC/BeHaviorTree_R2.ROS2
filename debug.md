# Debug Summary

| Error / Symptom | Root Cause | Fix |
|---|---|---|
| `PackageNotFoundError: package 'R2_Decision' not found` when launching `ros2 launch r2_decision ...` | Package name mismatch (actual name was uppercase or different) | Use correct package name or rename to lowercase; update launch to `get_package_share_directory("r2_decision")`. |
| `Failed to find ... foundationpose_interface/package.sh` while building `pose_est_bt` | `foundationpose_interface` not built/installed | Build `foundationpose_interface` first, then rebuild `pose_est_bt`. |
| `Can't find a tree with name: GetBlockPose` | BT executor parameters not loaded (node name mismatch in YAML) | Make YAML top key match node name (`bt_executor:` or `bt_action_server:`) and rebuild/install. |
| `filesystem error ... share/tf_listen_bt/lib` | Plugin libraries installed to `lib/` instead of `share/<pkg>/lib` | Install plugins to `share/${PROJECT_NAME}/lib` and rebuild. |
| `Node not recognized: TfListen` | Plugin registration name did not match XML node name | Register node as `"TfListen"` (or update XML to match). |
| `Can't find a tree with name: GetBlockPose` right after tree load failure | Tree XML failed to load because of unrecognized node | Fix node names/ports and rebuild so tree can register. |
| Action goal executed but tree still missing | Forgot to `source install/setup.bash` after rebuild | Re-source the workspace before launching and sending goals. |
