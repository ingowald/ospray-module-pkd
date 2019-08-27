# OSPRay Particle-K-D Tree (PKD Tree) Module

This project is a module for the OSPRay ray tracer (http://www.ospray.org) that implements the Particle K-D Tree Technique as described in the paper [I. Wald, A. Knoll, G. P. Johnson, W. Usher, V. Pascucci, M. E. Papka. “CPU Ray Tracing Large Particle Data using Particle K-D Trees,” IEEE Visweek, 2015](http://www.sci.utah.edu/publications/Wal2015a/ospParticle.pdf).

This code was mostly authored by Ingo Wald and Aaron Knoll (with help by several others); this code is experimental, and is hereby release without any warranty (implied or expressed) as to correctness, performance, etcpp.

# Building the PKD Tree module

To build the PKD module, you first have to have a version of OSPRay in source. Let us assume that you have already checked that out (into ~/Projects/ospray), that you already have created a cmake build directory (say, in ~/Projects/ospray/build), and that you have already properly configured and built ospray.

Now, first go to ospray's module subdirectory, and check out the pkd module into this directory:

    cd ~/Projects/ospray/modules
    git clone <this module>

Now, go into your ospray build directory, do a 'ccmake', enable the pkd module, and rebuild

    cd ~/Projects/ospray/build
    ccmake .
    # enable PKD module in cmake dialog, then 'c'onfigure and 'g'enerate
    make

You should now have a properly built _libospray_module_pkd.so_, as well as a _ospPartiKD_ binary.

## Building for LiDAR Data

To build the PKD module for LiDAR data you'll need [LAStools](http://www.cs.unc.edu/~isenburg/lastools/) to read
LAS and LAZ files, then can enable `OPTION_MODULE_PARTIKD_LIDAR` in CMake. If LAStools is installed in some
non-standard location you can pass `-DLASTOOLS=<path to LAStools root>`. **Warning:** This will break support
for any non-LiDAR data due to how the attributes are handled.

# Using the PKD Module

Once built, using the PKD module consists of two steps:

1) converting a particle model to pkd format 

2) rendering the resulting pkd file with ospray's sample qt viewer

## 1) Converting a particle model to pkd format

To convert a particle model to our internal format (which includes building the tree), you use the _ospPartiKD_ tool. PartiKD supports several file format (TODO: more doc needed here!), and is run as follows:

    ./PartiKD <inputFile> --radius <particleRadius> -o outFileName.pkd

**Example:** To convert the cosmic web particle data set I use

    ./PartiKD ~/cosmic_web/6k-cubed/0.000/particles/0.000xv00*dat --radius 1 -o ~/scratch/cosmic_web.pkd

This step should create two files: a cosmic_web.pkd, and a cosmic_web.pkdbin

## 2) Rendering a pkd file

Given a ".pkd" file (assuming ~/scratch/cosmic_web.pkd) you can render this with the OSPRay Example Viewer:

    ./ospExampleViewer --module pkd --import:pkd:<path to pkd file>

More Information:

- OSPRay: http://www.ospray.org

- A sample video of this technique has been uploaded by Aaron Knoll to youtube: https://www.youtube.com/watch?v=BxO5pjmUPwk

- Intel Parallel Computing Center for Data Analysis and Visualization at the SCI Institute, University of Utah http://cedmav.org/research/grantproj/grant/29.html

- The project was mentioned in a HPC Wire article by James Jeffers: http://www.hpcwire.com/2015/12/01/23288/

