# Changelog
All notable changes to this project will be documented in this file.

## [Unreleased]
### Added
- Changelog file.

### Changed
- Shifted the update-render loop to a parallel process to minimize the impact of frame rate on controller behavior.
- Integrated locks in the robots themselves to prevent threads from being locked for long.