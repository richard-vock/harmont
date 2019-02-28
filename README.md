harmont
=======

This branch is not for public use but instead used as a lightweight point cloud renderer for Wavefront obj files with added RGB values (as read by Meshlab).

### Dependencies ###

A C++17 compiler is expected (with filesystem support).

- Eigen 3 (http://eigen.tuxfamily.org). A recent version please.
- OpenGL (GL + GLU)
- GLEW (http://glew.sourceforge.net/)

Already included are versions of the nanoflann library as well as the RGBE file reader library.

### Compiling ###

In short:

mkdir build
cd build
cmake ..
make

### Running ###

The built cloud_renderer executable is used as follows:

    cloud_renderer <input_file> [hdr_name]

Where <input_file> is an obj file with RGB and radius information embedded in the vertex spec like in this example:

    v foo/bar

[hdr_name] is an optional name for a different HDR environment map. Available maps can be seen in the hdr folder where the "\_diffuse.hdr" part is to be ignored, so for example to use the "grace_diffuse.hdr" map use "grace" as the optional parameter.
