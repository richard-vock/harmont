harmont
=======

This branch is not for public use but instead used as a lightweight point cloud renderer for Wavefront obj files with added RGB values and radii.

### Dependencies ###

A C++11 compiler is expected (with filesystem support).

- Eigen 3 (http://eigen.tuxfamily.org). A recent version please.
- OpenGL (GL + GLU)
- GLEW (http://glew.sourceforge.net/)

### Compiling ###

In short:

```
mkdir build
cd build
cmake ..
make
```

### Running ###

The built cloud_renderer executable is used as follows:

    cloud_renderer <input_file>

Where <input_file> is an obj file with RGB and radius information embedded in the vertex spec like in this example:

    # pos x     pos y     pos z    red      green    blue     radius   normal_x  normal_y normal_z
    v -0.048350 -0.685820 1.379109 0.523725 0.626839 0.619690 0.006490 -0.020519 0.085549 0.996123

Color channel data is assumed to be in [0,1]; normals are assumed to be normalized.

You can use the '+' and '-' keys to change the size factor of rendered splats. Right mouse drag rotates the view, middle mouse drag pans and scrolling zooms in/out.
