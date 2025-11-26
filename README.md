# Altos V2 Radar Driver

This project is a driver for processing point cloud data from the Altos V2 radar system. It receives radar data packets via UDP, processes the point cloud information, and saves the data to files.

## Project Structure

```
.
├── altosRadarParse.cpp  # Main source code for processing radar data
├── pointCloud.h         # Data structure definitions
├── CMakeLists.txt       # CMake build configuration
├── data/                # Directory for RCS data and output files
└── README.md            # This file
```

## Features

- Receives radar point cloud data via UDP (both unicast and multicast supported)
- Calculates RCS (Radar Cross Section) values
- Processes point cloud data and converts from spherical to Cartesian coordinates
- Estimates relative velocities of detected objects
- Classifies points based on motion (approaching, stationary, moving away)
- Saves raw data to timestamped files

## Dependencies

- PCL (Point Cloud Library)
- Standard C++ libraries
- Linux socket programming libraries

## Building the Project

### Using CMake (Recommended)

```bash
# Create build directory
mkdir build
cd build

# Generate build files
cmake ..

# Compile the project
make
```

### Direct compilation with g++

```bash
g++ -o altosRadarParse altosRadarParse.cpp -lpcl_common -lpcl_io -lm -std=c++11
```

## Configuration

The application can be configured through several constants in [altosRadarParse.cpp](file:///home/xiaobo/work/code/altosV2Driver/altosRadarParse.cpp):

- Network settings:
  - `GROUPIP`: Multicast group IP address
  - `GROUPPORT`: Multicast port
  - `LOCALIP`: Local IP address
  - `UNIPORT`: Unicast port
  - `UNIFLAG`: Switch between unicast (1) and multicast (0)

- Processing parameters:
  - `vrMax`/`vrMin`: Velocity range limits
  - `vStep`: Velocity histogram step size
  - `errThr`: Error threshold for velocity estimation

## Usage

1. Prepare the data directory:
   ```bash
   mkdir -p data
   ```

2. Ensure `data/rcs.dat` exists (RCS calibration data):
   ```bash
   touch data/rcs.dat
   ```

3. Run the application:
   ```bash
   ./build/altosRadarParse
   ```

The application will:
- Listen for radar data on the configured network interface
- Process incoming point cloud packets
- Save raw data to timestamped files in the `data/` directory
- Output processing information to the console

## Data Format

The application processes point cloud data in a custom format with the following structure:

1. Packet Header ([PCKHEADER](file:///home/xiaobo/work/code/altosV2Driver/pointCloud.h#L22-L34))
2. System Information ([SYSINFO](file:///home/xiaobo/work/code/altosV2Driver/pointCloud.h#L40-L51))
3. Point Data ([DETECTION](file:///home/xiaobo/work/code/altosV2Driver/pointCloud.h#L7-L20) array)

Each point contains range, velocity, angle, and signal strength information.

## Output

Processed data is saved in two forms:
1. Raw data files in the `data/` directory with timestamps
2. Console output showing frame processing information

## Notes

- The application may require root privileges to access certain network ports
- The RCS calibration file (`data/rcs.dat`) should contain valid calibration data for accurate RCS calculations
- Network configuration should match the radar system settings