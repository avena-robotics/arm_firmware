@echo off

set datetimef=%date:~-4%%date:~3,2%%date:~0,2%_%time:~0,2%%time:~3,2%%time:~6,2%
C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe --bin universal_joint_3.2_MC5Y3_RI70\universal_joint_3_2_MC5Y3_RI70.axf --output "universal_joint_3_2_MC3Y3_RI70_%datetimef%.bin"