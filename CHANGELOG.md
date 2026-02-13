# Changelog

## Unreleased

- Added drone mode switching commands in VS Code:
  - `tensorfleet.setDroneMode` (`TensorFleet: Set Drone Mode`)
  - `tensorfleet.switchDroneModeReal` (`TensorFleet: Switch Drone Mode to Real`)
  - `tensorfleet.switchDroneModeSITL` (`TensorFleet: Switch Drone Mode to SITL`)
- Added VM Manager drone mode client contract for:
  - `GET /vms/{vmId}/drone-mode`
  - `POST /vms/{vmId}/drone-mode`
  - Error codes: `COMMAND_FAILED`, `COMMAND_START_FAILED`, `COMMAND_TIMEOUT`
- Added extension-side drone mode command handler tests for success and failure states.
