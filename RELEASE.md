ADHamamatsu Releases
======================

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============

R1-0 (November XXX, 2024)
===================
* Initial release
  - This release is based on a prototype driver written by Hamamatsu Photonics, which was based on ADSimDetector.
  - Mark Rivers made many changes including the following:
    - That driver was based on my ADDimDetector driver, and had residual code from the simDetector that was unused.  I removed that.
    - Changed all tabs to spaces, and reformatted all the text to have consistent indenting.
    - The driver used EPICS PVs to initialize the SDK and connect to the camera.
      I moved the initialization and connection into the constructor, as is done for all other areaDetector cameras.
    - Fixed crashes due to uninitialized variables.
    - Improved the medm screen, using the standard .adl file components where possible (ADShutter.adl, ADAttrFile.adl, ADBuffers.adl).
    - Added example CONFIG_SITE.$(EPICS_HOST_ARCH).Common files to define the path to the Hamamatsu DCAM SDK files.
    - Use standard ADDriver parameters where possible.

