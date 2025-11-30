# **BUILDING.md**

Build Instructions for Kalibrate-HydraSDR on Windows, Linux, and macOS

This document describes how to build **kalibrate-hydrasdr** on multiple platforms using CMake, MinGW, MSVC, and Linux toolchains.
It assumes you have the HydraSDR SDK (headers and libraries) available locally.

---

# **1. Directory Layout**

Your project directory should follow this structure:

```
project_root/
├── CMakeLists.txt
├── include/
│   └── hydrasdr.h
├── lib/
│   ├── libhydrasdr.dll.a   (MinGW import library)
│   ├── hydrasdr.lib        (MSVC import library)
│   └── other required libs
├── libhydrasdr.dll         (Windows runtime DLL, optional in root)
└── src/
    ├── kal.cc
    ├── arfcn_freq.cc
    ├── c0_detect.cc
    ├── circular_buffer.cc
    ├── fcch_detector.cc
    ├── hydrasdr_source.cc
    ├── offset.cc
    ├── util.cc
    └── ...
```

This layout ensures CMake can detect include paths and link dependencies correctly.

---

# **2. Building on Windows (MinGW-w64 / MSYS2)**

### **Prerequisites**

Install MSYS2/MinGW toolchain:

```
pacman -S --needed base-devel mingw-w64-x86_64-toolchain
pacman -S mingw-w64-x86_64-fftw
```

Ensure HydraSDR SDK files are present:

* `include/hydrasdr.h`
* `lib/libhydrasdr.dll.a`

### **A. Manual g++ build (no CMake)**

```
g++ -O3 -std=c++11 -Wall \
    -I./include -I./src \
    src/arfcn_freq.cc src/c0_detect.cc src/circular_buffer.cc src/dsp_resampler.cc \
    src/fcch_detector.cc src/hydrasdr_source.cc src/kal.cc src/offset.cc src/util.cc \
    -o kal \
    -L./lib -lhydrasdr -lfftw3 -lpthread
```

### **B. Building with CMake**

```
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make
```

The output executable will be located in the `build/` directory.

---

# **3. Building on Windows (Microsoft Visual Studio / MSVC)**

### **Prerequisites**

Visual Studio does not provide:

* `pthread.h`
* `getopt.h`

These must be installed via **vcpkg**.

### **A. Install vcpkg**

```
git clone https://github.com/microsoft/vcpkg
.\vcpkg\bootstrap-vcpkg.bat
```

### **B. Install dependencies**

```
.\vcpkg\vcpkg install fftw3:x64-windows
.\vcpkg\vcpkg install getopt:x64-windows
```

### **C. HydraSDR library for MSVC**

MSVC cannot use `libhydrasdr.dll.a`.

You need:

* `hydrasdr.lib` (MSVC import library for the HydraSDR DLL)

If you only have the DLL, you may generate the import library:

```
lib /def:hydrasdr.def /out:hydrasdr.lib /machine:x64
```

Place the `.lib` file in:

```
lib/hydrasdr.lib
```

### **D. Generate Visual Studio solution**

```
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

Open the generated `.sln` file in Visual Studio and compile.

---

# **4. Building on Linux**

### **Install dependencies**

Ubuntu/Debian:

```
sudo apt install build-essential cmake libfftw3-dev libusb-1.0-0-dev pkg-config git
```

```
git clone https://github.com/hydrasdr/rfone_host.git rfone_host
cd rfone_host
mkdir build
cd build
cmake ../ -DINSTALL_UDEV_RULES=ON
make
sudo make install
sudo ldconfig
```

Ensure HydraSDR SDK files are available in:

```
include/hydrasdr.h
lib/libhydrasdr.so
```

### **Build**

```
mkdir build
cd build
cmake ..
make
```

The resulting binary will be produced in `build/`.

---

# **5. Building on macOS (Clang)**

HydraSDR supports macOS when the SDK libraries are provided.

### **Install dependencies (Homebrew)**

```
brew install cmake fftw libusb
```

### **Build**

```
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

Ensure that:

* `libhydrasdr.dylib` is in `lib/`
* `hydrasdr.h` is in `include/`

---

# **6. Notes on Runtime Libraries**

For Windows:

Ensure the following DLLs are placed next to `kal.exe` or available in PATH:

* `libhydrasdr.dll` (HydraSDR runtime)
* `libusb-1.0.dll`
* `libwinpthread-1.dll` (MinGW only)
* `libfftw3-3.dll` (if not statically linked)

For Linux/macOS:

Ensure the dynamic linker can locate:

* `libhydrasdr.so`
* `libhydrasdr.dylib`

See https://github.com/hydrasdr/rfone_host to build libhydrasdr

Use `LD_LIBRARY_PATH` or install system-wide.

---

# **7. Summary Table**

| Platform              | Build System  | Notes                          |
| --------------------- | ------------- | ------------------------------ |
| Windows (MSYS2/MinGW) | CMake or g++  | Uses `libhydrasdr.dll.a`       |
| Windows (MSVC)        | CMake + vcpkg | Requires `hydrasdr.lib`        |
| Linux                 | CMake         | Standard GCC/Clang environment |
| macOS                 | CMake         | Requires Homebrew dependencies |
