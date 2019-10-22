# Changelog
All notable changes to this project will be documented in this file.

## [0.0.5]
### Changed
- Changed the graphical frontend to pygame. Kept Matplotlib as a dependency to ensure backwards compatibility.

## [0.0.2 - Unreleased]
### Added
- Changelog file.

### Changed
- Shifted the update-render loop to a parallel process to minimize the impact of frame rate on controller behavior.
- Integrated locks in the robots themselves to prevent threads from being locked for long.