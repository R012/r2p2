# Changelog
All notable changes to this project will be documented in this file.

## [1.0.0b]
### Added
- Added path planning functionalities to the PDDL executor.
- Added CO2 origin display to the map display functionalities.
- Added an optional argument to control FPS display.
- Added a controllers/controllers.py file to centralize imports and registering of newly generated controllers.

### Changed
- Cleaned the API for ease of use.
- Upgraded the teleop controller to allow for joystick input.
- Recreated the grid functionalities for path planning.
- Made PID parameters optional, and added an efficient default configuration.
- Refactored all controllers into their own files, now living under controllers/.

### To do
- Implement optional parameters to control whether the grid will be shown for any given controller, config-side.

### Known issues
- Minor errors while displaying the grid if the axes have different sizes, namely some coordinates not displaying properly.

## [0.0.5]
### Changed
- Changed the graphical frontend to pygame. Kept Matplotlib as a dependency to ensure backwards compatibility.

## [0.0.2 - Unreleased]
### Added
- Changelog file.

### Changed
- Shifted the update-render loop to a parallel process to minimize the impact of frame rate on controller behavior.
- Integrated locks in the robots themselves to prevent threads from being locked for long.
