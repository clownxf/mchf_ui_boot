# mchf_ui_boot

-- mcHF rev 0.8 UI board bootloader --

Note: This code is still somehow experimental. For older mcHF board revisions, please use the better supported 
fork of the code here: https://github.com/df8oe/UHSDR

---------------------------------------------------------------------------------------------------------------

This bootloader has rudimentary LCD support. Some HW testing(absolutely needed!) on each boot up is to be
implemented. As it is can pass control to main firmware or be controlled by the DSP bootloader and reflash
the main image.

---------------------------------------------------------------------------------------------------------------
How to compile:

- Download SW4STM32 studio (http://www.openstm32.org) and install

- On first run set your Workspace directory to something in MyDocuments,
not the mcHF project directory! In Eclipse, unlike other IDEs, the Workspace
is settings holder, not projects grouping method!

- Import the mcHF project from local dir or GitHub directly(point the path to
firmware\mchf_ui_boot\project\mchf-pro-boot and make sure mchf-pro-boot is checked)

All mcHF downloads here: http://www.m0nka.co.uk/?page_id=5269

--------------------------------------------------------------
Krassi Atanassov, M0NKA
mcHF project, 2012 - 2019