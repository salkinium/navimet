# Navimet: Navigation Helmet



## Building

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

