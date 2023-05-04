# SnapmakerController-IDEX

SnapmakerController-IDEX is the firmware for Snapmaker IDEX Printers. It's based on the popular [Marlin firmware](http://marlinfw.org/) with optimized FreeRTOS support.

## Introduction

SnapmakerController-IDEX builds on the codebase of Marlin 2.0.9.1, we also added some new features:

- Vibration compensation with input shapper, support many types of shappers:
  - ei, ei2, ei3;
  - mzv, zv, zvd, zvdd, zvddd.

- Smoothed look-ahead (refer to [klipper](https://github.com/Klipper3d/klipper.git)).
- Corner velocity settings (refer to [klipper](https://github.com/Klipper3d/klipper.git)).
- Snapmaker specific functions like calibrations for Z offset and XY offset.
- Quickly switching toolhead.
- HMI (Touch Screen) communication over SACP.

## Feedback & Contribution

- To submit a bug or feature request, [file an issue](https://github.com/Snapmaker/SnapmakerController-IDEX/issues/new) in github issues.
- To contribute some code, make sure you have read and followed our guidelines for [contributing](https://github.com/Snapmaker/SnapmakerController-IDEX/blob/master/CONTRIBUTING.md).

## Development

### Setup Development Environment

As of recommended in Marlin's development settings, we use **Visual Studio Code** and **PlatformIO IDE** to develop Snapmaker2-Controller. 

- Follow [Setting up Visual Studio Code](https://code.visualstudio.com/docs/setup/setup-overview) to install and setup **VSCode**.

- Follow the [guide](https://platformio.org/install/ide?install=vscode) to install PlatformIO extension in **VSCode**.
- Clone [SnapmakerController-IDEX repo](https://github.com/Snapmaker/SnapmakerController-IDEX) using Git to your local folder.

```shell
> git clone git@https://github.com/Snapmaker/SnapmakerController-IDEX.git
```

- Open downloaded repo in **VSCode**
  - Use the **Open Folder…** command in the **VSCode** **File** menu
  - Then choose top folder of **SnapmakerController-IDEX** in your location
- After opening the source code in **VSCode**, you will see these icons at the bottom status bar，it also indicates PlatformIO has been installed successfully.

![VSCode with PlatformIO](https://user-images.githubusercontent.com/3749551/98325327-814d3200-2029-11eb-9dd8-df9bee2dcbad.png)

### Ensure your changes are loaded

- The printer will not load new firmware if the version string remains the same
- Please update [Marlin/src/inc/Version.h](https://github.com/Snapmaker/SnapmakerController-IDEX/blob/main/Marlin/src/inc/Version.h) to change the `J1_BUILD_VERSION` or your changes will not be loaded when flashing the firmware.

### Compile the code

- To compile the code, you have two ways:
  - click the **build** icon in status bar
  - click the **terminal** icon to open terminal, then type command ***pio run***

NOTE: if you build the source for first time, PlatformIO will download the relative libraries and toochains. It may take a few minutes.

- After PlatformIO finishing the build, you will get two images:
  - `(PROJECT FOLDER)/.pioenvs/GD32F105/firmware.bin`: image to be programmed into main controller
  - `(PROJECT FOLDER)/.pioenvs/GD32F105/firmware.elf`: image used to debug the firmware (in online debug tools like Ozone) 

- To clean previous build, just click the **clean** icon, or type command ***pio run -t clean*** in the terminal.

### Program compiled firmware to main controller

#### With PlatformIO CLI

After building, type below command in VSCode terminal

```
> pio run -t pack
```

Then you will get below firmwares in the folder `(PROJECT FOLDER)/release`:

- `J1_MC_APP_{xxx such as V2.2.2}_{xxx such as 20201222}.bin`: minor image of controller, can be used to generate major image
- `J1_{xxx: version such as V2.2.2}_{xxx: date such as 20201222}.bin`: major image which can be used to upgrade modules with USB stick

Finally, copy the major image to your USB stick and upgrade your machine follow the instructions in [How to update Firmware](https://wiki.snapmaker.com/en/snapmaker_j1/manual/User_Manual/Maintenance) section.

## License

SnapmakerController-IDEX is released under terms of the GPL-3.0 License.

Terms of the license can be found in the LICENSE file or at https://www.gnu.org/licenses/gpl-3.0.en.html.
