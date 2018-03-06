# Description

MoBDec (pronounced as Moby Dick) - for Moving Objects Detection - is the code that I use for my research experimentations.
My current configuration on Linux:
- GCC 7.2
- OpenCV 3.3 with Opencv Contrib and CUDA 8.0.
- libldof_gpu.so from https://lmb.informatik.uni-freiburg.de/resources/binaries/, "Large Displacement Optical Flow" part. Need CUDA 7.5.

# Build - Linux
```shell
cd build
cmake ..
make
```

# Publication

"Coupled 2D and 3D Analysis for Moving Objects Detection with a Moving Camera"

Take a look at configuration_file/coupled_2D_and_3D_analysis_for_moving_objects_detection_with_a_moving_camera.yml
