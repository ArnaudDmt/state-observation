# Installing

There are several ways of installing and using this software, based on your preferences:
- From ubuntu packages
- With Nix
- From source

## Nix

### Build

You can build this package with:

```sh
nix build .#state-observation
```

and its documentation with

```sh
nix build .#state-observation.doc
```

### Develop

This package provides a nix flake. To get started, simply use

```sh
nix develop .#state-observation
```

to enter a developement shell with all dependencies available, then

```sh
mkdir build
cmake -B build $cmakeFlags
cmake --build build -j8
```

## Ubuntu LTS (22.04, 24.04, 26.04)

You must first setup our package mirror:

```sh
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' \
  | sudo -E bash
```

You can also choose the head mirror which will have the latest version of this package:

```sh
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' \
  | sudo -E bash
```

You can then install the package:

```sh
sudo apt install libstate-observation-dev
# Install documentation
sudo apt install libstate-observation-doc
```

## Manually build from source

To compile you need the following tools:

 * [Git](https://git-scm.com/)
 * [CMake](https://cmake.org/) >= 3.22
 * [pkg-config](https://www.freedesktop.org/wiki/Software/pkg-config/)
 * [doxygen](https://www.doxygen.nl/)
 * [g++](https://gcc.gnu.org/) >= 4.7 (for C++11 support)
 * [Boost](http://www.boost.org/doc/libs/1_58_0/more/getting_started/unix-variants.html) >= 1.49
 * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2

### Building

```sh
git clone --recursive https://github.com/jrl-umi3218/state-observation
cd state-observation
mkdir build
cd build
cmake [options] ..
make && make intall
```
