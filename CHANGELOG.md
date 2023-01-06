# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [Unreleased]

### Changed

-  Updated NCS version to `v2.2.0`. Resolved any issues connected with this and 
   updated readme.

### Removed

-   `east.yml` file, `east` tool does not need it any more to function.

## [1.0.0] - 2023-01-05

### Added

-   PCA9554 driver implementation which implements Zephyr's GPIO API. This driver
    does not support interrupt functionality. This driver was tested with
    `nrf-sdk v2.1.0.`.
-   Sample which shows basic use and configuration.

[Unreleased]: https://github.com/IRNAS/irnas-pca9554-driver/compare/v1.0.0...HEAD

[1.0.0]: https://github.com/IRNAS/irnas-pca9554-driver/compare/6f7a9b66a890d6ea1cd648d72d7a67404892e5f3...v1.0.0
