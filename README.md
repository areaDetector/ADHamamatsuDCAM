ADHamamatsuDCAM
===============
An 
[EPICS](http://www.aps.anl.gov/epics/) 
[areaDetector](https://cars.uchicago.edu/software/epics/areaDetector.html) 
driver for cameras from [Hamamatsu](https://www.hamamatsu.com/us/en/product/cameras.html) 
using their DCAM library.

The driver should work with any Hamamatsu camera that is supported by the DCAM library.
It has been tested with the [Orca Fire](https://www.hamamatsu.com/us/en/product/cameras/cmos-cameras/C16240-20UP.html)
camera on Windows and Ubuntu 22.

The Orca Fire](https://www.hamamatsu.com/us/en/product/cameras/cmos-cameras/C16240-20UP.html)
is a [CoaXPress](https://en.wikipedia.org/wiki/CoaXPress) camera, using CXP-6 X4.
It is 4432x2369 pixels, and can run at115 frames/s in 16-bit mode.
While this camera uses CoaXPress, it is not fully GenICam compliant.
Only a few GenICam features are exposed in its XML file.
This means that it must be used with the Active Silicon frame grabber that is
supported by the DCAM library.  It cannot be used with other CoaXPress frame grabbers, for
example those supported by ADEuresys.

The Github issues in this repository should only be used for issues specific to the EPICS
driver, not for issues with the cameras or the DCAM library.
Those issues should be reported to  Hamamatsu through their normal customer support mechanisms.

Additional information:
* [Documentation](https://areadetector.github.io/areaDetector/ADHamamatsuDCAM/ADHamamatsuDCAM.html).
* [Release notes](RELEASE.md).