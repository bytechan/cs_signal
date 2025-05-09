## CsSignal

### Introduction

CsSignal is a library for thread aware Signal/Slot delivery. This library does not depend upon CopperSpice or any
other libraries.

One of the major benefits of CsSignal is how the library delivers signals in a multithreaded application. Signals can
be delivered using a queued connection or a blocking queued connection.

In the CsSignal library signals and slots are both methods whereas in many other signal/slot libraries each signal is
a separate object.


### System Requirements

Building CsSignal requires a C++20 compiler and a C++20 standard library.

CMake build files are provided with the source distribution to build this library. The unit test binary executable is
an optional part of the build process.

This library has been tested with clang sanitizer and an extensive industry code review.


### Running the Catch Tests

To enable unit testing set the BUILD_TESTS in the root CMakeLists.txt file to ON. You can also configure the
setting by passing -DBUILD_TESTS=ON on the CMake command line.

If you do not have the Catch2 library installed the files can be download from our website. This is a header
only library.

https://download.copperspice.com/toolchain/catch/

Either add the path where the Catch2 files are located or pass the path using -DCMAKE_PREFIX_PATH on the CMake
command line. The following shows how to pass both arguments.

export CMAKE_FLAGS="-DBUILD_TESTS=on -DCMAKE_PREFIX_PATH=C:/Catch2/lib/cmake/Catch2"

Build this library and then run CTest in the root of your build directory.


### Documentation

Class level documentation for CsSignal is available on the CopperSpice website:

https://www.copperspice.com/docs/cs_signal/index.html


### Presentations

Our YouTube channel contains over 75 videos about C++, programming fundamentals, Unicode/Strings, multithreading,
graphics, CopperSpice, DoxyPress, and other software development topics.

https://www.youtube.com/copperspice

Links to additional videos can be found on our website.

https://www.copperspice.com/presentations.html


### Authors / Contributors

* **Ansel Sermersheim**
* **Barbara Geller**


### License

This library is released under the BSD 2-clause license. For more information refer to the LICENSE file provided with
this project.


### References

 * Website:  https://www.copperspice.com
 * Twitter:  https://twitter.com/copperspice_cpp
 * Email:    info@copperspice.com

<!-- -->
 * Github:   https://github.com/copperspice

<!-- -->
 * Forum:    https://forum.copperspice.com
 * Journal:  https://journal.copperspice.com

