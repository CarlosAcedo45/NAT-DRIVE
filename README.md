# NAT-DRIVE

Naturalistc data collection and processing framework designed to facilitated the acquisition of real-world car-following (CF) data through field experiments.


This repository contains the Python and MATLAB implementation for the following paper:


**NAT-DRIVE: A Novel Approach for Collecting and  Validating Naturalistic Car-Following Data**  
Jose Carlos Acedo Aguilar, Sabbir Ahmned, Davi V. Q. Rodrigues, Mingfeng Shang, Shian Wang

## Overview

The code aims to support researchers processed naturalistic CF based on the recordings from a [Racebox mini GPS](https://www.racebox.pro/products/racebox-mini?srsltid=AfmBOorvv0Dqr7NldHoyiihFM__L8jbzjT3ZvE4ZTFppwYhhORAzQPXs) and a [AWR2944EVM from TI](https://www.ti.com/tool/AWR2944EVM).  
Data procesing is then broken into three key parts defined by each folder of this respository:  
1. Data Synchronization.
2. Radar.
3. CF Model Calibration

Furthere documentation is given in a suplementary README file inside of each folder.

## Requirements
- Python
- numpy, matplotlib, math, gmplot, pandas, os
- MATLBAL 2025b
