# Controlled High-Speed Benchmarking of Event-Based Visual Place Recognition

A controlled rotary platform for benchmarking event-based visual place recognition, integrating servo control, coordinated event-camera acquisition, visualisation and SAD-based recognition analysis across rotational speeds.

## Start here

| Guide | Contents |
|---|---|
| [01 Repository guide](servo_controller/01_README.md) | Folder structure, workflow and testing instructions. |
| [02 GUI and recording](servo_controller/02_GUI_README.md) | Motor-PC interface, LattePanda recording service and experiment collection. |
| [03 VPR Media](servo_controller/03_VPR_MEDIA_README.md) | Recording preparation, alignment, images and videos. |
| [04 SAD VPR](servo_controller/04_SAD_VPR_README.md) | Recognition, representation comparisons and motion/event evaluation. |
| [Scenes](Scenes/SCENES_README.md) | Printable fixed-size pi targets and their specification. |
| [Experiments](Experiments/EXPERIMENTS_README.md) | Local acquisition layout and analysis-output descriptions. |

## Repository contents

- `servo_controller/`: application, camera service, processing scripts, tests and workflow guides.
- `Scenes/`: eight fixed-size printable pi target images. Other target variants are not bundled here.
- `Manuals/`: the project operating manual, currently labelled as a draft.
- `Experiments/`: the folder guide only. Recordings, caches and generated results are not included in Git.

Run application commands from `servo_controller/` using the appropriate Python environment. Local settings, credentials, virtual environments and logs are excluded by `.gitignore`.

The GUI controls physical machinery. Simulation is available for software development; physical operation requires the commissioned rig, its hardware protections and appropriate operating procedures. Software tests do not establish physical stop or containment performance.

## Attribution

The SAD implementation follows the event-count baseline and tutorial by Tobias Fischer and Michael Milford. See [third-party attribution and upstream licence](servo_controller/THIRD_PARTY_VPR.md) for details. The upstream licence applies to the identified third-party work; this README does not grant a separate licence for all project material.
