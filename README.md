The Open Motion Planning Library (OMPL)
=======================================

Linux / macOS [![Build Status](https://travis-ci.org/ompl/ompl.svg?branch=main)](https://travis-ci.org/ompl/ompl)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/valuv9sabye1y35n/branch/main?svg=true)](https://ci.appveyor.com/project/mamoll/ompl/branch/main)

Visit the [OMPL installation page](https://ompl.kavrakilab.org/core/installation.html) for the detailed installation instructions.

OMPL has the following required dependencies:

* [Boost](https://www.boost.org) (version 1.58 or higher)
* [CMake](https://www.cmake.org) (version 3.5 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)

The following dependencies are optional:

* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Py++](https://github.com/ompl/ompl/blob/main/doc/markdown/installPyPlusPlus.md) (needed to generate Python bindings)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  https://ompl.kavrakilab.org/core)

Once dependencies are installed, you can build OMPL on Linux, macOS,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    # next step is optional
    make -j 4 update_bindings # if you want Python bindings
    make -j 4 # replace "4" with the number of cores on your machine

If you found this research useful for your own work, please use the following citation:

    @INPROCEEDINGS{fit_2024,
      author={Zhang, Liding and Bing, Zhenshan and Chen, Kejia and Chen, Lingyun and Cai, Kuanqi and Zhang, Yu and Wu, Fan and Krumbholz, Peter and Yuan, Zhilin and Haddadin, Sami and Knoll, Alois},
      booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
      title={Flexible Informed Trees (FIT*): Adaptive Batch-Size Approach in Informed Sampling-Based Path Planning}, 
      year={2024},
      volume={},
      number={},
      pages={3146-3152},
      doi={10.1109/IROS58592.2024.10802466}}
