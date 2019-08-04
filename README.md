# Navimet: Navigation Unit

This project combines a BNO055 IMU, a NEO-6 GPS, a NRF24 radio and a series of
WS2812B LEDs into a stand-alone navigation unit built around the NUCLEO-L432KC
development board.

The on-board GPS computes the local position, while the IMU computes the local
(magnetic) heading. When the button is pressed, the local position is broadcast
to the other nodes via the radio. When a position is received via the radio the
heading and distance to this position is computed and shown interactively on a
360° WS2812 LED ring.

The behavior is such that the heading ring shows the distance through the LED
"width" and the closer the target position, the wider the LEDs show it.


## Customizing

The LED position ring is configured for 24 LEDs, this can be changed in
`src/tasks/ring.cpp`. In addition, only the red channel is currently changed to
display both distance and heading. This can be changed in the same file.


## Building

Before you build: there exists a `src/navimet.bin`, which you can simply
drag-and-drop onto the NUCLEO USB mass storage device to program it.

If you want to change application code, you only need to call `scons` not
`lbuild`, since the generated modm and ublox libraries have checked into this
repo.

Otherwise, here are the steps to generate the libraries and compile the
application from scratch:

1. Install modm: https://modm.io/guide/installation/#macos
2. Clone the repository recursively!
	```
	git clone --recurse-submodules --jobs 8 git@github.com:salkinium/navimet.git
	```
	Or if you didn't clone recusively:
    ```
    git submodule update --init --jobs 8
    ```
3. Generate the libraries inside `src` folder:
	```
	cd src
	lbuild build
	```
4. Build the application
    ```
	scons
	```
5. Program the dev board
    ```
	scons program
	```

