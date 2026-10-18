# Pinned application components

Root CMake builds the local submodule checkouts with Component Manager disabled.
The application Git tree pins each component to an exact commit; initialize with
`git -C /path/to/application submodule update --init --recursive` after cloning.
These revisions supersede the earlier dirty-local/base-revision inventory.

| Component | Selected commit |
| --- | --- |
| esp-mt6701 | `98d605d0d40e61f6dcb33bdab08ded92c512145c` |
| esp_2wd_robot | `ac6f4c4fd742ce7b691ccf6be0bd05bd27b798a5` |
| esp_angle_lut | `74ae7c1c70a409076d49361b4ded679935f5e456` |
| esp_drv8833 | `b62ac55131af160cf71e55f2ac4e71bcb1c46ceb` |
| esp_pid | `9ff1debce5d9ea6a2d9c73d1de2921d053e90a61` |
| esp_rt_diagnostics | `9d5ac3541a372f28879255ce716c6efbe8a51b37` |
| esp_rt_diagnostics_reporter | `994331a7905d6125489b5050d67e4ae0791ae089` |
| kalman | `5bbe0a624acbf06548bb963a8498dc7a19c92b3b` |

The robot's manifest pins LUT, PID, Kalman and reporter. The reporter pins the
diagnostics accumulator transitively. DRV8833 and MT6701 are physical application
adapters, so the application also tracks them directly. The local checkouts must
match the gitlinks for reproducible builds; manifests do not override this root
application's explicit offline/local build selection.

Publish diagnostics before reporter, and LUT plus reporter before the robot;
publish all component commits before sharing the application commit that points
to them. Public GitHub main heads were checked on 2026-10-18 and matched every revision
in the table; this verifies publication, not Component Manager-only resolution.
No generated managed_components content or local measurement MAT data is tracked.

`.gitmodules` and component manifests contain standard public HTTPS URLs.
Developer SSH aliases may be set in local `remote.origin.pushurl` only; they are
not installation dependencies and do not transfer automatically to another clone.
The root application has no configured remote until its own destination is chosen;
it is distinct from the reusable esp_2wd_robot component repository.
