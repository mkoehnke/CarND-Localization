# Localization (Kidnapped Vehicle) Project
This project implements a 2 dimensional particle filter in C++. The particle filter is initialized with a given map and some initial localization information (analogous to what a GPS would provide). At each time step the filter retrieves observation and control data. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Installation

Install the uWebSocketIO using the following command:

```
sh install-mac.sh
```

Compile the source code by entering the following commands:

```
mkdir build
cd build
cmake ..
make
./particle_filter
```

![EKF Simulator](https://github.com/mkoehnke/CarND-Localization/raw/master/resources/kidnapped_vehicle.png)

