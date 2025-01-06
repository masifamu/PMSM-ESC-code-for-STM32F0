# PMSM ESC Firmware for STM32F0

This repository contains Electronic Speed Controller (ESC) firmware $ Hardware for Permanent Magnet Synchronous Motor (PMSM). This ESC is built with STM32F0 microcontroller and currently supports trapezoidal control method to control to the motor.

## Features
- **Trapezoidal Control method** for motor control.
- Optimized for **STM32F0** microcontrollers.
- Customizable parameters for various motors.
- Real-time diagnostics via UART.
- Modular and easy-to-extend codebase.

## Getting Started
### Prerequisites
- STM32F0-based hardware.
- [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html).
- Basic knowledge of motor control and STM32 development.

### Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/masifamu/PMSM-ESC-code-for-STM32F0.git
   cd pmsm-esc-stm32f0

### Getting Started
1. Open the project in STM32Cube IDE.
2. Build and flash the firmware to your STM32F0 microcontroller.

### Configuration

- Edit `config.h` to set motor and control parameters.
- Enable or disable diagnostics as needed.

### Repository Structure
/src - Source files for control algorithms and drivers. /inc - Header files and configuration. /docs - Documentation and guides. /examples - Sample applications.

### Contributing

Contributions are welcome! Please fork the repository, create a branch, and submit a pull request. See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

### License

Licensed under the [GNU General Public License v2.0](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html).

### Resources

- [STM32F0 Reference Manual](https://www.st.com/resource/en/reference_manual)
- [Field-Oriented Control Basics](https://www.ti.com/lit/an/sprabq8/sprabq8.pdf)
