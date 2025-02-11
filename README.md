# ESP Rust Playground

This is a simple project to demonstrate how to use Rust on an ESP32 microcontroller. The project is based on the `esp-hal` crate and is configured to compile to `esp32s3` microcontroller.

The main part is a FOC implementation based on algorithm (currently velocity motion control and simple PI without D) from `SimpleFOC`. See `motor.rs` for more details.
