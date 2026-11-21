# Firmware: arm-side motion driver

`system_task_handle.cpp` runs on the NexArm's ESP32. It is a **drop-in replacement** for the file
of the same name in Hiwonder's official NexArm Arduino example. The other files of that example
(inverse kinematics link to the AT32, display, buttons, buzzer) are Hiwonder's and are **not
included** in this repository; get them from Hiwonder's NexArm documentation package.

## What it does

* **PC link** (used by the Python FSM and HMI): text lines at 115200 baud
  * laptop → arm: `@ping`, `@tel N`, `@mv x y z pitch roll claw ms`, `@estop`, `@reset`
  * arm → laptop: `TEL,ms,x,y,z,pitch,roll,claw,claw_raw,estop,age`, `EVT,KEY1`, `EVT,ESTOP,KEY2`, `PONG`, `ERR,...`
* **KEY2 = hardware E-stop**, latched in the firmware: motion commands are refused until `@reset`.
* **Stand-alone teaching mode** (serial console, type `help`): jog, `save A` / `save B`, `run N`,
  `stats` (ISO 9283 accuracy and repeatability). This mode produced the measurements in `data/`.

## Build and upload

1. Arduino IDE 2 (or `arduino-cli`) with the **ESP32 board package by Espressif, version 2.0.11**
   (Hiwonder's example needs 2.0.x, not 3.x).
2. Libraries: `U8g2`, `MultiButton`, `PS3 Controller Host`, and the DFRobot **HuskyLens** library
   (shipped with Hiwonder's example).
3. Open Hiwonder's `Nex_Arm` example and replace its `system_task_handle.cpp` with the one in this folder.
4. Board settings: **ESP32 Dev Module**, Flash 4 MB, **Partition scheme: Huge APP**, PSRAM disabled, CPU 240 MHz.
5. Upload, then open the serial monitor at 115200 baud and type `help`.

`arduino-cli` equivalent:

```
arduino-cli compile --upload -p COM12 \
  --fqbn esp32:esp32:esp32:PSRAM=disabled,PartitionScheme=huge_app,CPUFreq=240 Nex_Arm
```
