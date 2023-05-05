# 2024-Vision
The repository for team 8230's 2024 Vision System

To Do:

- [ ] Buy the vision stuff
- [ ] Get PhotonVision running on Beelink
- [ ] Get multiple cameras running with Photon
- [ ] See if it's accurate/otherwise able to be used (networking, etc.)
    - Don't spend too long on this if it isn't working

If Photon is good:

    - [ ] Fine tune it for all the cameras
    - [ ] Put Beelink in read only mode

If Photon is innacurate or otherwise bad:

    - [ ] Figure out what framework we want to use for a custom solution (OpenCV?)
    - [ ] Create said custom solution (KoiVision?)
        - [ ] Get it working on a laptop camera
            - Refer to [This](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/index.html) for some basic instructions
            - [ ] Filtering
            - [ ] Calibration
            - [ ] 2d Detection
            - [ ] 3d Detection
        - [ ] Get custom solution running on Beelink
            - [ ] Multi threading support
            - [ ] Get it working with USB cameras
                - [ ] USB camera detection/use
                - [ ] Work out multi camera support
            - [ ] Figure out data transfer
                - [ ] Java interface
                - [ ] Reciving commands
                - [ ] Outputting data
    - [ ] Fine tune for all cameras
    - [ ] Put Beelink in read only mode
