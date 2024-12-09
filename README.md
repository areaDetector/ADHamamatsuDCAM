ADHamamatsuDCAM
===============
An 
[EPICS](http://www.aps.anl.gov/epics/) 
[areaDetector](https://cars.uchicago.edu/software/epics/areaDetector.html) 
driver for cameras from [Hamamatsu](https://www.hamamatsu.com/us/en/product/cameras.html) 
using their DCAM library.

The driver should work with any Hamamatsu camera OS environment that are supported by DCAM as listed in their Compatibility Notes.
It has been tested with the [Orca Fire](https://www.hamamatsu.com/us/en/product/cameras/cmos-cameras/C16240-20UP.html)
camera with an Active Silicon AS-FBD-4XCXP6-2PE8 frame grabber on Windows and Ubuntu 22.

The [Orca Fire](https://www.hamamatsu.com/us/en/product/cameras/cmos-cameras/C16240-20UP.html)
is a [CoaXPress](https://en.wikipedia.org/wiki/CoaXPress) interface camera, using CXP-6 X4.
It is 4432x2369 pixels, and can run at 115 frames/s in 16-bit mode at full resolution.

The Github issues in this repository should only be used for issues specific to the EPICS
driver, not for issues with the cameras or the DCAM library.
Those issues should be reported to  Hamamatsu through their normal customer support channels
or send an email with subject prefix [EPICS] as in this mailto link: HCRDG@hamamatsu.com.

Additional information:
* [Documentation](https://areadetector.github.io/areaDetector/ADHamamatsuDCAM/ADHamamatsuDCAM.html).
* [Release notes](RELEASE.md).