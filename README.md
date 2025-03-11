# Beachbot

## Overview

Beachbot is a robotics project for creating autonomous beach robots. The system integrates various hardware components including cameras, GNSS (GPS), IMU sensors, and motors to create a fully functional robot that can navigate and operate in beach environments.

## Features

- Camera-based vision system
- GNSS/GPS positioning
- IMU-based orientation
- Motor control system
- Navigation and localization
- Remote control capabilities
- User interface

## System Requirements

- Python 3.x (Anaconda distribution recommended)
- Hardware components:
  - Camera modules
  - GNSS/GPS receiver
  - IMU sensor
  - Motor controllers (ODrive compatible)
  - Jetson development board (for GPIO)

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/beachbot.git
cd beachbot
```

### 2. Set up Python environment

The project uses Python and requires several dependencies. We recommend using a virtual environment:

```bash
# Create and activate virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install requirements
pip install -r env/python_requirements/requirements.txt
```

## Usage

Beachbot can be run in two modes:

- UI mode: For monitoring and controlling via a user interface
- RC mode: For remote control operation

### Running the application

```bash
python main.py ui  # For UI mode
python main.py rc  # For RC mode
```

The orchestrator will automatically launch the required nodes based on the selected profile.

## Project Structure

- `src/python/`: Core Python modules
  - `controls/`: Motor and movement control modules
  - `drivers/`: Hardware driver interfaces
  - `geometry/`: Geometric calculations and transformations
  - `ipc/`: Inter-process communication
  - `kinematics/`: Robot kinematics
  - `localization/`: Position tracking and localization
  - `mapping/`: Environmental mapping
  - `models/`: Data models
  - `node/`: Individual system nodes (camera, GPS, IMU, etc.)
  - `orchestration/`: System orchestration and node management
  - `planning/`: Path planning and navigation
  - `tools/`: Utility tools and helpers

## Development

### Adding a new node

1. Create a new Python file in `src/python/node/`
2. Implement the node functionality
3. Add the node configuration to `NodeConfig` in `src/python/orchestration/launch.py`
4. Include the node in the appropriate profile in the `_gen_profile` function

## License

See the [LICENSE](LICENSE) file for details.

## Troubleshooting

### Common Issues

- Hardware connection problems: Ensure all devices are properly connected and powered
- Missing dependencies: Verify all required Python packages are installed
- Permission issues: Some hardware interfaces may require elevated permissions

For more help, please open an issue on the project repository.
