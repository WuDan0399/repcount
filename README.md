# Reps Counter

Rep counter is a smart gym application to record the number of reps an user have done with dumbbells.

## Recommended System
* Windows, MacOS, Ubuntu 18.04

## Hardware Requirements
* Manuca Air stack, consists of programming\debugging board, manuca air, sensor board from bottom to the top.
* [STLINK V3SET](https://www.st.com/en/development-tools/stlink-v3set.html).
* USB to TTL Serial cable.

# Getting Started

## Hardware Set Up
1. Connect the Manuca Air stack with PC using TTL-usb cable and STLINK-V3. (reference: https://www.siot.gov.sg/starter-kit/set-up-your-hardware/)

## Software Set Up

### Mbed Studio

1. Download and install the latest Mbed Studio [here](https://os.mbed.com/studio/).
3. Locate and open the workspace `RepCount`.
2. Set Target as **NUCLEO-L476RG** inside Mbed Studio.

### Arduino IDE
1. To visualising the output from the sensor, download arduino IDE [here](https://www.arduino.cc/en/software).
2. Set board as **Arduino UNO**.
3. Choose the correct port that the Manuca Air is connected.
3. Open Serial monitor or Serial plotter to observe the output.

## Build and Flash application

1. Download the base code with the mbed-os library [here](https://drive.google.com/drive/folders/1or_4bsE6CyxNxOh8ZdXjUYpasFRZvgPO?usp=sharing). **Note**: The mbed-os library cannot be uploaded to github due to file size limitation. We highly recommend to download the code from Google Drive and update the code from github.
3. Unzip the code. 
      * `unzip repcount_23062021.zip`
5. `cd repcount`
6. Pull the latest update code from this repo.
      * `git pull origin main`
7. (opt) A decription file for current application can be found in `docs\Rep_count_algoirthm.md`. To modify the application, locate and modify the file `main.cpp` in `RepCount` workspace. 
8. Build the program inside Mbed Studio.
9. Locate the file `RepCount.bin` in `RepCount/BUILD/NUCLEO_L476RG/ARMC6`.
10. Copy the binary file into `STLINK_V3S` folder.

