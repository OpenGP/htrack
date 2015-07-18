# Robust Articulated-ICP for Real-Time Hand Tracking

- **YouTube Video**: https://youtu.be/rm3YnClSmIQ
- **Paper PDF**: http://infoscience.epfl.ch/record/206951/files/htrack.pdf

## Disclaimer
To obtain the results shown in the video (or as seen in the live demo sessions at SGP'15) proper hardware is necessary:
- **Ubuntu 14.04** (on windows/osx the OS interferes with GPU on Compute/Graphics context switches) 
- Intel Core **i7 @4GhZ**
- CUDA Graphic card (**NVIDIA GTX980** used in our demo)
- **Senz3D** depth camera (SENSOR_DEPTHSENSEGRABBER, use SoftKinetic or Intel/Creative)

Other notes:
- note the software must be compiled in **64bits**!
- **Wristband** color calibration (make sure the wristband is detected robustly otherwise the tracking might not perform as effectively, you can check this by enabling "show wband" in the htrack_atb application)

## BibTex
    @article{htrack_sgp15,
    author = {Andrea Tagliasacchi   and   Matthias Schr{\"o}der   and   Anastasia Tkach and 
              Sofien Bouaziz        and   Mario Botsch            and   Mark Pauly},
    title = {Robust Articulated-ICP for Real-Time Hand Tracking},
    journal = {Symposium on Geometry Processing (Computer Graphics Forum)},
    month = July,
    year = 2015}

## News
- July 8th 2015: we won the **Best Paper Award** at the Symposium on Geometry Processing (SGP) 2015!
- June 12th 2015: official code release (in sync with CVPR'15 HANDS workshop)

## Acknowledgements
- Thanks to "Tu-Hoa Pham" for his pull request that improved the SoftKinetic (Senz3D) Sensor! 
