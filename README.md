# calico

CALICO: a method for calibrating asynchronous camera networks and/or multicamera systems, version 1.1. April 2020 (Original release November 2019).

Changelog: 
- Docker image added March 2020.
- Added incremental method as default, and another overhaul, April 2020.

Roadmap
- [Contact](#contact)
- [References](#underlying-ideas-how-and-when-to-cite-this-work)
- [Docker release](#docker-release)
- [Dependences](#dependencies)
- [Building](#building)
- [Running](#running)
	- [Running from a Docker container](#running-from-a-docker-container)
- [Input format](#input-format)
- [Output format](#output-format)
	
# Contact 

Comments/Bugs/Problems: amy.tabb@usda.gov, or open an issue on Github.

# Underlying ideas; how and when to cite this work

This README file is produced by Amy Tabb as a companion to a paper:
	Calibration of Asynchronous Camera Networks: CALICO

````latex
@article{tabb_calibration_2019,
	title = {Calibration of Asynchronous Camera Networks: CALICO},
	url = {http://arxiv.org/abs/1903.06811},
	abstract = {},
	urldate = {2019-11-14},
	journal = {arXiv:1903.06811 [cs]},
	author = {Tabb, Amy and Medeiros, Henry and Feldmann, Mitchell J. and Santos, Thiago T.},
	month = nov,
	year = {2019},
	note = {arXiv: 1903.06811},
	keywords = {Computer Science - Computer Vision and Pattern Recognition}
}
````

Dataset and/or code:

Tabb, Amy, & Feldmann, Mitchell J. (2019). Data and Code from: Calibration of Asynchronous Camera Networks: CALICO (Version 1.0) [Data set]. Zenodo. [http://doi.org/10.5281/zenodo.3520866](http://doi.org/10.5281/zenodo.3520866)

````latex
@dataset{tabb_amy_2019_3520866,
  author       = {Tabb, Amy and Feldmann, Mitchell J.},
  title        = {Data and Code from: Calibration of Asynchronous 
                   Camera Networks: CALICO},
  month        = nov,
  year         = 2019,
  publisher    = {Zenodo},
  version      = {1.0},
  doi          = {10.5281/zenodo.3520866},
  url          = {https://doi.org/10.5281/zenodo.3520866}
}
````

If you use this code in project that results in a publication, please cite at a minimum the paper above, and best practice would be to cite the paper and the dataset.  Otherwise, there are no restrictions in your use of this code.  However, no guarantees are expressed or implied.

## Changes from published version

The code has been updated to use an interleaved solve of the multi-camera calibration problem.  Whereas before, steps 4 and 5 of the arXiv paper were:

4: Find the initial solution set V0 by iteratively solving first individual, and then pairwise uninitialized variables found at prior iterations (using a closed-form method), section 4.4 of the paper.

5: Find  the  final  solution  set V by  refining  the  estimated HTMs through reprojection error minimization (i.e. Levenberg-Marquardt algorithm), section 4.5 of the paper.

This version of calico is still available, using the `--non-incremental` flag.  Now, however, the default setting is to solve the problem incrementally for variables: a closed-form solution is used as an initial solution for non-linear minimization of reprojection error through Levenberg-Marquardt.  For some datasets, the result is very similar to the prior result.  For others, the new approach is better and for other datasets we are working on, the new approach works much better.  

## Docker release

To avoid building the code yourself, a Docker image of this project is available, and the Dockerfile used to generate it is part of this repository.

I suggest using the Docker release to evaluate this code and as a fast way to get started with it, as the code itself runs quickly.  If you want to extend or look at the details of the code, you can build it yourself using the instructions and code in this repository.

### Install Docker

[Install Docker](https://docs.docker.com/install/), if you haven't already.  I endorse uninstalling old versions if you have them floating around.

### Pull the image

The image for CALICO is : [amytabb/calico](https://hub.docker.com/r/amytabb/calico).

```bash
docker pull amytabb/calico
```

### Run the image

CALICO needs to read and write results to disk; to do so with Docker means that we need to mount a portion of your hard drive to a volume in the Docker image.

I used a bind mount below; the Docker image's volume is `docker_dir` and will not change no matter which machine or dataset you run it on.  `/full/file/path/on/your/machine` is the directory that you want the reading and writing to occur.  

Example:

```bash
sudo docker run -v /full/file/path/on/your/machine:/docker_dir -it amytabb/calico:latest bash
```

The bind mount is potentially confusing, so here is an example.  Say I have a directory `/home/amy/Data/March/` and within `March` is a directory of images that I want to process with CALICO.  I also want to write to a directory within `/home/amy/Data/March/`.  So, 

```bash
sudo docker run -v /home/amy/Data/March:/docker_dir -it amytabb/calico:latest bash
```

Creates a container with all of the libraries and a Ubuntu 18.04 operating system, and bash shell (command line), and may look something like:

```bash
root@f6feb7ce8c31:/docker_dir# 
```

but if you take a look at the contents of `/host_dir`, with `ls`, they are `/home/amy/Data/March/`.  That's the bind mount magic.

First, suppose we forgot to create the write directory.  No problem.

```bash
root@f6feb7ce8c31:/host_dir# mkdir write-dir
```

creates our write directory `write-dir`.

And from here on out, we issue commands from this Docker container, which is writing to our filesystem.  Skip to [Running](#running) to get details on how to run the code.  The only difference is that `./` is not needed before commands when using the Docker version, and the `--src-dir=[STRING]` has been set up within the Docker image, and does not need to be specified. 


## Dependencies

This code uses the Ceres, OpenCV 4.0, OpenMP, and Eigen libraries.  Ceres *can* be used without cmake, but is best used with cmake.  I've included instructions for building with cmake, and the specific OpenCV libraries needed, as well as what OpenCV 3.x versions have worked without alteration.

[Ceres](http://ceres-solver.org/)

We are not responsible for whatever it takes to get Ceres to build; but advise that having a recent version of Eigen and Google's glog are helpful to the process. 

[OpenCV](http://opencv.org/) As of this writing, the version installed from [Github](https://github.com/opencv/opencv) is returning OpenCV 4.xx.  The code will also compile if the user has OpenCV 3.xx installed, but we encountered runtime errors with OpenCV 3.3.0  OpenCV 3.4.8 was tested and worked fine.  In both cases, the [OpenCV contributed module](https://github.com/opencv/opencv_contrib) is needed to detect aruco patterns.  These libraries need to be installed: 

- `opencv_core`
- `opencv_imgproc`
- `opencv_imgcodecs` 
- `opencv_aruco` 
- `opencv_calib3d`

[OpenMP](https://www.openmp.org/) OpenMP is used to parallelize some sections of the individual camera calibration section.  On Ubuntu, to install the library, run `sudo apt-get install libgomp1` at a terminal.   **Now OpenMP is required.**  Evidently, OpenMP support on MacOS is ... difficult to accomplish.  If you have notes on getting this to work on MacOs, let me know so that I can add them to this document.  If you cannot get OpenMP working and you have a Mac (or Windows), I suggest using the Docker container to run calico.

[tex-live](https://tug.org/texlive/) If you enable the `--verbose` option for CALICO, some of the output will be written to latex files, and then .pdfs will be generated.  To get this all to work, the `pdflatex` needs to be available.  On Ubuntu again, run `sudo apt-get install texlive-latex-base` to get started.

[exiftool](https://owl.phy.queensu.ca/~phil/exiftool/) This is only needed for the `--rotate` case, which is specific to our laboratory.  I doubt that other users will need it, but if you are running the `rot1` and `rot2` datasets, you will need this.  On Ubuntu, install with `sudo apt-get install exiftool`. 

This code has been tested on Ubuntu 16.04 and Ubuntu 18.04.  You are welcome to convert it to Windows, but I have not.  While OpenCV is available from distribution repositories, my long experience with it is has always been to build from the source to get the best results.


## Building 

These instructions will walk you through cloning to configuring with cmake, and importing to the Eclipse Integrated Development Environment (IDE) (if desired).

1. Clone! Go to the location where you want the project, and from a terminal `git clone https://github.com/amy-tabb/calico.git`

2. `cd calico`

3. The code is in `src`.  Create a build directory, `mkdir build` , `cd build`.

4. Here's the cmake fun.  The way I have the cmake file configured (`calico/src/CmakeLists.txt`), the last installed version of OpenCV will be used.  I don't specify a version.  However, you can specify which version of OpenCV is used by altering the the `find_package()` line:
 
- `find_package( OpenCV 3.4.8 REQUIRED )`
- `find_package( OpenCV 4 REQUIRED )`

Both work and those versions are compatible with CALICO. 

5. Configure with cmake.  Don't have cmake? (`sudo apt-get install cmake`). Then from the build folder, you can use any of the following four options below: 

- `cmake ../src`  (basic)
- `cmake  -DCMAKE_BUILD_TYPE=Release ../src` (Release configuration)
- `cmake  -DCMAKE_BUILD_TYPE=Debug ../src` (Debug configuration)
- `cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE ../src/` (Create an Eclipse project to import -- it will be in the build folder)

In case you have installed OpenCV and cmake can't find it, you need to specify the location of `OpenCVConfig.cmake`.  Don't know where it is?   Find out with `locate OpenCVConfig.cmake`.  Then append

`-DCMAKE_PREFIX_PATH=dir`

in my case on one machine, this was:

`-DCMAKE_PREFIX_PATH=/usr/local/opencv41/lib/cmake/opencv4/`

where `/usr/local/opencv41/lib/cmake/opencv4/` is the directory containing `OpenCVConfig.cmake`.  Of course, you will substitute whatever the appropriate directory returned from  `locate OpenCVConfig.cmake` was.


6. Then, you can either import the project to Eclipse (if you used the last option), or build from there, or type `make`.   If the everything compiled and linked, and you have an executable named `calico`, you are ready to go. 

7. I highly suggest that you download at least one test dataset from [http://doi.org/10.5281/zenodo.3520866](http://doi.org/10.5281/zenodo.3520866).  These datasets are in the format needed for CALICO, and you can ensure that everything is correctly configured on your system.

## Running

To see the available options, run `./calico` with no arguments or `--help`.:

````
Printing help for calico
ESSENTIAL FUNCTIONALITY -------------------
--verbose                     No arguments.  Writes additional information during the run.
--create-patterns             No arguments, write aruco image patterns from a specification file.
--network                     No arguments, indicates that this dataset is a camera network or multicamera system.
--rotating                    No arguments, indicates that this dataset is the rotating type with background.
--synch-rotate                No arguments, indicates the where there are multiple cameras in the rotating case, that they are synchronized.  Default is false. (not synchronized).
--ground-truth                No arguments. The ground truth calibration information is available.
--num-threads                 Number of threads to use.  Default is # returned by omp_get_max_threads();, currently = 24
--non-incremental             No arguments.  Set the solving method to use the linear the non-linear match minimization, instead of the default incremental linear/non-linear reprojection error algo.

DIRECTORIES AND PATHS ----------------------- 
--input=[STRING]              Mandatory, has to be a directory.
--output=[STRING]             Mandatory, has to be a directory.
--src-dir=[STRING]            Directory where the source code resides relative to where the executable is being run. 
 Specifically, the location of 'detector_params.yml'  Default is ../src/ . 

CAMERA CALIBRATION OPTIONS ---------------------------
--only-camera-cali            No arguments, only perform individual camera calibration, but not network calibration.
--read-camera-cali            No arguments, read-previously-computed camera calibration from file.  It should be in the output directory.
--non-zero-tangent            No arguments. In the camera calibration part, sets the tangential components of radial distortion (p1, p2) to non-zero.
--non-zero-k3                 No arguments. In the camera calibration part, sets the 3rd radial distortion k value to non-zero.
--fix-pp                      No arguments. In the camera calibration part, sets the principal point to the image center. 
--focal-px=[float]            Initial focal length in pixels for the camera.  Default is max dimension * 1.2 

OPTIONS ON NUMBER OF IMAGES READ/USED; NUMBER OF POINTS USED FOR NETWORK -----------
--max-internal-read=[int]     Integer argument. Sets the number of internal camera calibration images to read. Default is the number of images in the directory 'internal'.
--max-internal-use=[int]      Integer argument. Sets the number of images to use where the pattern is detected in the calibration, from the 'internal' directory. The default is the maximum number of patterns found in . 
--max-external=[int]          Integer argument. Sets the number of images /time instants to read from  the 'external' directory for each camera.
--k=[int]                     Specifies the number of points to use for the minimization of reprojection error, relevant only  for the network case.  The default is 8.
--num-pattern=[int]           Integer argument. Sets the number of points required to estimate the   pose for a pattern.  Default is >=10 for network, >= 4 for rotating.

DISPLAY -----------
--camera-size=[float]         Float argument.  Specifies the size of the cameras written, default is 40.
--track-size=[float]          Float argument.  Specifies the size of the track size written, default is 0.5 .
````

Assuming you've downloaded some of the datasets from Zenodo [http://doi.org/10.5281/zenodo.3520866](http://doi.org/10.5281/zenodo.3520866), the arguments used to run a sampling is here:

sim1:  `./calico --network --k=8 --input=/home/username/data-calico/sim1/base/ --output=/home/username/data-calico/sim1/result/ --camera-size=80 --track-size=8 --ground-truth --verbose`

mult1: `./calico --network --k=8 --input=/home/username/data-calico/mult1/base/ --output=/home/username/data-calico/mult1/result/ --camera-size=40 --track-size=5 --verbose`   
  
rot1: `./calico --rotating --input=/home/username/data-calico/rot1/base/ --output=/home/username/data-calico/rot1/result/ --camera-size=40 --track-size=5 --verbose`  

The arguments used to generate the results are written in a file in the output folder, titled  `arguments-calico.txt` . 

It is suggested that you generate aruco patterns to use in your own calibrations using the `--create-patterns` argument.  

1. To do so, create an input and output directory.  

2. Select whether you will be creating a network (most likely), or rotating case and create the corresponding file `network_specification_file.txt` or `rotate_specification_file.txt` detailed in [Calibration object specifications](#calibration-object-specifications).  

3. call CALICO using the `--create-patterns`, `--network` or `--rotating` flags and specify the input and output directories.  And example is: 

````bash
./calico --input /home/username/data-calico/sim1/base --output /home/username/data-calico/created-pattern --network --create-patterns
````

4. The code create calibration patterns  -- in the `patterns` subdirectory within the output directory.  Print and measure the squares.  Edit the `squareLength_mm` parameter in the `pattern_square_mmNUMBER.txt` files.  

### Running from a Docker container

As mentioned in [Run the image](#run-the-image), you start the Docker container from the command line, and then issue commands similar to above.  So, start the container running, 

````bash
sudo docker run -v /home/amy/Data/March/:/docker_dir -it amytabb/calico:latest bash
````

And this will start the container.  Suppose we forgot to create a directory for the results. 
 
````bash
root@d208fe4482b5:/docker_dir# 
````

Supposing we forgot to create a directory for the results, we can do so from the terminal here:

````bash
root@d208fe4482b5:/docker_dir# mkdir results/
````
Then, we can call calico.  Note that the `--src-dir=[STRING]` flag is not needed; the paths have already been set to the defaults within the Docker image. For example using the `sim-1-base` dataset:

````
root@d208fe4482b5:/docker_dir# calico --input sim-1-base/ --output results/ --network
````


# Input format

For most datasets, the input consists of the [Image Data](#image-data) and [Calibration object specifications](#calibration-object-specifications).

### Image data

directory: `data` 
Within `data` are directories, one for each camera.  There is no specification on the names, so long at the names are different from each other.

Within the individual camera directories, there are the images comprising the dataset OR folders, `external` and `internal`.

Images in the `internal` folder are only used to refine the intrinsic camera calibration parameters, but not to estimate relative transformations between cameras.  In general, I have not found that I needed additional images, but difficult acquisitions contexts may benefit from more images.  The images in the `external` directory, or without folders, in the individual camera directory, are ordered by the time variable referenced in the paper.

By this I mean that the 1st image in the `camera0` directory was acquired at a corresponding time as the 1st image in the `camera1` directory.

### Calibration object specifications

Currently, the specifications of the calibration object is different for the network versus the rotating camera case.   The network case is the most likely case for those using CALICO, and consists of a two of more cameras that may or may not be asynchronous, and a number of charuco calibration patterns rigidly attached together to create a calibration rig.  The transformations between patterns is not needed -- CALICO computes those transformations.

The rotating camera case is specific to a data acquisition environment we have in one of our labs.  There is an aruco backstop, charuco patterns rigidly attached together to create a calibration rig, and it is assumed that the images have exif information to use for initializing internal camera calibration. 


#### Network case

For the network case, you'll need to include a file called `network_specification_file.txt` in the input directory.  This file will specify the number of patterns and their individual characteristics; the boards may have different characteristics.  Because of this, the items in the file must be in order.  Here is an example with four charulco boards (from `sim1`, `sim2` datasets:

````
aruco_dict 10
number_boards 4
squaresX 7
squaresY 9
squareLength 45
markerLength 22
squaresX 7
squaresY 9
squareLength 45
markerLength 22
squaresX 7
squaresY 9
squareLength 45
markerLength 22
squaresX 7
squaresY 9
squareLength 45
markerLength 22
````

The [aruco dictionary code](aruco-code-numbers) `aruco_dict` parameter provides the dictionary constant used to generate the aruco markers.  `number_boards` is the number of calibration patterns used to construct the calibration rig.  

After those two parameters, for every calibration pattern `squaresX`, `squaresY`, `squareLength`, and `markerLength` are needed. The X and Y parameters are the number of squares in the chessboard grid, and `squarelength` is the size of the chessboard squares in pixels.  `markerLength` is the size of the aruco marker in pixels,  `squarelength < markerLength`. 

For each calibration pattern, there will also need to be a corresponding `pattern_square_mmNUMBER.txt`, where `NUMBER` is the index of that calibration pattern.  (Taking a look at the dataset TODO will likely be helpful here.) Each file consists of one parameter, such as:

````
squareLength_mm 45
````

where the parameter is the length of the *chessboard* squares in the charuco pattern, in millimeters.

#### Rotating case

In this case, you will need to specify the parameters for the aruco backstop as well as the charuco patterns used to calibrate for the positions of the cameras as the calibration rig moves.  It is assumed that the charuco patterns all have the same parameters, per pattern, while the individual tags per pattern are different.  The file is named `rotate_specification_file.txt` and an example is:

````
internal_squaresX 14
internal_squaresY 14
internal_squareLength 200
internal_markerLength 90
internal_margin 50
number_boards 8
squaresX 3
squaresY 3
squareLength 60
markerLength 46
aruco_dict 10
sensorWidth 23.5
````

now, the section below gives the parameters for the aruco backstop.  `_squaresX` and `_squaresY` and the number of aruco IDs in the X and Y axis.  Then `internal_markerLength` is the length in pixels allocated for each aruco ID in the image, `internal_squareLength` is the length in pixels of the greater grid, into which aruco markers are placed.  Given the dimensions below, each aruco ID will have a border of 90 pixels.  There is a border of 50 pixels at the edge of the image.  

 
````
internal_squaresX 14
internal_squaresY 14
internal_squareLength 200
internal_markerLength 90
internal_margin 50
````

The following parameters are for the charuco patterns that are used for the calibration object. `number_boards` is the number of calibration patterns, `squares_X`, `_Y` the dimension of the charuco grid, `squareLength` and `markerLength` is similar to above for the aruco pattern.   

````
number_boards 8
squaresX 3
squaresY 3
squareLength 60
markerLength 46
````

Finally, the [aruco dictionary code](aruco-code-numbers) `aruco_dict` is needed, as well as the CCD or CMOS sensor width in millimeters. This information is usually available from the manufacturer's website for the camera.   

````
aruco_dict 10
sensorWidth 23.5
````

Then two more text files are required, `pattern_square_mm_internal.txt` and `pattern_square_mm_external.txt`, corresponding to the physical dimensions of the squares for the aruco backstop and charuco patterns, respectively.  There is one entry in each file, 

````
squareLength_mm 20
````
which indicates the length of one edge in millimeters.

** VERY IMPORTANT ** The code does not check for this (at the moment, TODO); each dictionary has an upper limit on how many aruco makrers it can generate (last number in the code `DICT_4X4_100` means 100 markers). Ensure that the number of patterns in the X-Y grid does not exceed the number in the aruco dictionary you have selected. 

#### Aruco code numbers

````c++
enum PREDEFINED_DICTIONARY_NAME {
    DICT_4X4_50 = 0,
    DICT_4X4_100,
    DICT_4X4_250,
    DICT_4X4_1000,
    DICT_5X5_50,
    DICT_5X5_100,
    DICT_5X5_250,
    DICT_5X5_1000,
    DICT_6X6_50,
    DICT_6X6_100,
    DICT_6X6_250,           //10
    DICT_6X6_1000,          //11  
    DICT_7X7_50,
    DICT_7X7_100,
    DICT_7X7_250,
    DICT_7X7_1000,
    DICT_ARUCO_ORIGINAL,
    DICT_APRILTAG_16h5,     ///< 4x4 bits, minimum hamming distance between any two codes = 5, 30 codes
    DICT_APRILTAG_25h9,     ///< 5x5 bits, minimum hamming distance between any two codes = 9, 35 codes
    DICT_APRILTAG_36h10,    ///< 6x6 bits, minimum hamming distance between any two codes = 10, 2320 codes
    DICT_APRILTAG_36h11     ///< 6x6 bits, minimum hamming distance between any two codes = 11, 587 codes
};
````

# Output format

## Directory structure

The output is stored in the directory you specified with the `--output` flag.  The following subdirectory structure is created:

````
cameras-initial
cameras-minimization1
data
patterns
reconstructed-patterns
verbose_output [if you used the --verbose flag TODO check]
````

The arguments used to call the program are in file `arguments-calico.txt`. 

## Quantified results
To view the results, take a look at `total_results.txt`, which is within the output directory.  The quantified results are reported according to three metrics: algebraic error (equation 18 in the paper), reprojection error (equation 19 in the paper), and finally reconstruction accuracy error (rae, equations 18 and 19).  RAE is reported by mean, median, and standard deviation.  Note that not all  calibration pattern points may be reconstructed. 

## Network calibration file
If you're running this code, you probably just want a file showing the calibration of the cameras relative to each other.  

The final result is in `camera_cali_minimization.txt` and has a file format that is similar to, but not exactly like, the [Middlebury Multi-View Stereo format](http://vision.middlebury.edu/mview/data/).  An example of the first three lines of the `sim-1` result is:

````
8
camera_images0 1401.9 0 601.382 0 1402.08 449.69 0 0 1 -0.999983 0.000221258 -0.00585503 -0.000221361 -1 1.69823e-05 -0.00585503 1.82781e-05 0.999983 -336.214 167.961 1503.06 0.00847675 -0.0396309 0 0 0 
camera_images1 1398.49 0 598.732 0 1397.42 448.67 0 0 1 -0.999998 -0.000333189 -0.00195105 0.000332287 -1 0.000462825 -0.0019512 0.000462176 0.999998 666.101 168.9 1496.45 -0.00395854 0.00205396 0 0 0 
````

The first number in the file is the number of cameras.

Then, on each line, is:
- the directory name for the camera
- k11 k12 k13 k21 k22 k23 k31 k32 k33 (where k11 is the top left entry of the intrinsic camera calibration matrix **K**, k12 is the top row, second column of **K**, etc.)
- r11 r12 r13 r21 r22 r23 r31 r32 r33  (where r11 is the top left entry of the rotation matrix **R**, r12 is the top row, second column of **R**, etc.)
- t1 t2 t3 (elements of the translation vector **t**)
- d1 d2 d3 d4 d5 (elements of the distortion vector, property of intrinsic camera calibration) 


## Individual camera calibration results

The camera calibration results are stored in the `data` subdirectory, and within `data`, there are directories for each camera.  `cali_results.txt` contains the intrinsic parameters for the camera, and then `extNUMBER.img_ext` are the original images used for computing the network calibration, with the calibration points identified and undistorted using the intrinisic camera calibration parameters.  These images can be checked in case of very poor calibrations.   

## Visualization of reprojection 

`cameras-initial` holds the results from step 4.  Within that directory, `variables.txt` gives the values for individual variables.  

`EquationNUMBER.png` are image files for each foundational relationship.  The difference between the image point and reprojected point is shown as a line.  Ideally, the difference a small, so you may see a small point.  In the `--network` version, there are rings around the k points that will be used for the reprojection error minimization in step 5.

The same structure is available for the output of step 5, in directory `cameras-minimization1`.

## Visualization of cameras

Camera results are recorded by storing them as .ply files, which can be opened in any 3D model viewer.  I use [Meshlab](http://www.meshlab.net/), which is free.

It is possible to view the output of step 4, the initial solution in `cameras-initial`, and the refined solution in step 5 `cameras-minimization1`.  The camera files are within those directories as `single_cameras`.  You can load all of the cameras with `all.ply`, or load them individually by number.  

Alternatively, if you are interested in seeing the movement of the cameras with respect to the calibration objects, camera models are in individual files within the `cameras-initial` and `cameras-minimization1`.

The names follow the pattern:

````
c0_time0.ply
c0_time1.ply
c0_time2.ply
.
.
.
track0.ply
````

where `c0_time0.ply` corresponds to "camera 0 at time 0".  File `track0.ply` consists of (generally) line segments connecting the camera locations over time.  

Camera and track sizes can be adjusted to increase visability through the `--camera-size` and `--track-size` flags.

## Visualization of reconstructed calibration patterns

Reconstructed patterns are written in the `reconstructed-patterns` directory.

RAE, reconstruction accuracy error, is computed assuming that the pattern has been transformed back to the world coordinate system.  In the camera calibration literature, and in the conventions I used in creating CALICO, the world coordinate system is at (0,0,0) and the pattern's points are distributed on the X-Y plane.  

To assess the difference between the ideal world coordinate system (as defined by a pattern) and the computed transformation, there are 3D model files that again one can load in their 3D model viewer of choice.   
- `world-ideal_patternNUMBER.ply` is the coordinate system as defined by the calibration pattern, which  
- `world-initial-patternNUMBER.ply` is the estimate produced by step 4 of CALICO.  
- `world-minimization1-patternNUMBER.ply` is the estimate produced by step 5 of CALICO.  Throughout, `NUMBER` is the pattern number.  If the pattern transformation is not estimated, which does happen, the transformation will be the identity.

(Here's a listing of files for the sim1 dataset.)

````bash
at-P-initial-pattern0.ply
at-P-initial-pattern1.ply
at-P-initial-pattern2.ply
at-P-initial-pattern3.ply
at-P-minimization1-pattern0.ply
at-P-minimization1-pattern1.ply
at-P-minimization1-pattern2.ply
at-P-minimization1-pattern3.ply
world-ideal_pattern0.ply
world-ideal_pattern1.ply
world-ideal_pattern2.ply
world-ideal_pattern3.ply
world-initial-pattern0.ply
world-initial-pattern1.ply
world-initial-pattern2.ply
world-initial-pattern3.ply
world-minimization1-pattern0.ply
world-minimization1-pattern1.ply
world-minimization1-pattern2.ply
world-minimization1-pattern3.ply
````

Then, to view the calibration rig, where the patterns are arranged relative to each other, see the `at-P-initial-patternNUMBER.ply` and `at-P-minimization1-patternNUMBER.ply` files, for the step 4 and step 5 results, respectively. 

If the `--incremental` flag was used, you will see `at-P-incremental-patternNUMBER.ply` instead. 

## Listing of foundational relationships

If the `--verbose` flag is used, the `verbose_output` directory is created.  The most important documents in there are 

````
doc-first-unreduced-singles.pdf
doc-after-stage3.pdf    
````

`doc-first-unreduced-singles.pdf` is a listing of the foundational relationships created by CALICO, with the selected p^* and t^*.  `doc-after-stage3.pdf` shows the same listing of foundational relationships after p^* and t^* are substituted, and what variables are intialized as a result (go to the bottom for a list).  Note that the equation numbers start from 1, while in the code the numbering starts from 0 if you are trying to line up items with the `EquationNUMBER.png` images in  [Visualization of cameras](#visualization-of-cameras). 

#### TODO
Check for bad input : the number of markers specified in the files is greater than the size of the aruco dictionary.
Check for bad input : alert if markerLength is > squareLength
Verbose and write: Add another flag for writing output, as lots of images are currently written.
