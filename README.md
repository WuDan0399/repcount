# Reps Counter

Rep counter is a smart gym application to record the number of reps an user have done with dumbbells.

## Recommended System
* Windows, MacOS, Ubuntu 18.04

## Hardware Requirements
* Manuca Air and sensor boards.
* [STLINK V3SET](https://www.st.com/en/development-tools/stlink-v3set.html).
* USB to TTL Serial cable.

# Getting Started
## Code and Libraries

1. Download the base code with the mbed-os library [here](https://drive.google.com/drive/folders/1or_4bsE6CyxNxOh8ZdXjUYpasFRZvgPO?usp=sharing).
2. Pull the latest update code from this repo.
    * `git clone https://github.com/WuDan0399/repcount.git`


## Mbed Studio

1. Download and install the latest Mbed Studio [here](https://os.mbed.com/studio/).
3. Locate and open the workspace `RepCount`.
2. Set Target as **NUCLEO-L476RG** inside Mbed Studio.

## Arduino IDE
1. To visualising the output from the sensor, download arduino IDE [here](https://www.arduino.cc/en/software).
2. Set board as **Arduino UNO**.
3. Choose the correct port that the Manuca Air is connected.
3. Open Serial monitor or Serial plotter to observe the output.

## Modifying the Code
1. Locate and modify the file `main.cpp` in `RepCount` workspace.

## Programming the Board
1. Build the program inside Mbed Studio.
2. Locate the file `RepCount.bin` in `RepCount/BUILD/NUCLEO_L476RG/ARMC6`.
3. Copy the file into `STLINK_V3S` folder.

