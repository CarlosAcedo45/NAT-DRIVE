# Model Calibration

CF model calibration and string stability analysis for a specific recorded CF event.  

The main code is Stability_Sim.m using the matching Sync and Radar .csv files in NAT-DRIVE/Model Calibration/Data as inputs to calibrate the IDM, OVRV, and AOVRV CF models.
The rest of the .m files are MATLAB function files that get called through the calibration and stability simulation process. 

- **Stability_Sim.m**: main code for calibration and stability analysis. Returns Calibrated parameters for each CF-model, and string stability results.
- **All_Models_Calibration.m**: Initializes the vehicle states according to the recorded data which is then passed to each corresponding CF-model_Calibration.m function.  
- **CF-model_Calibration.m**: This refers to each IDM, OVRV, and AOVRV Calibration functions. we guide interested readers in the calibration methodology to the source paper of this work.  
- **CF-model_Dynamics_Loop.m**: Computes the CF-behavior of a vehicle given the entirety of the preceding vehicle trajectory.
- **Lambda2.m**: Calculates the Lambda_2 condition for string stability of the IDM and OVRV CF models.
