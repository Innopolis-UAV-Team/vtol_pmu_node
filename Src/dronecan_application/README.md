The node has the following registers:

| №  | Register name           | Description |
| -- | ----------------------- | ----------- |
|  1 | uavcan.node.id          | Defines a node-ID. Allowed values [0,127]. |
|  2 | system.name             | Defines custom node name. If empty, the node will use 'arl.pmu' name. |
|  3 | battery.soc_pct         | Percent of the full charge [-1, 127]: </br> - Value of -1 means auto estimation (doesn't work at the moment), </br> - [0, 100] means constant percent, </br> - [101, 127] means constants as well, but the values are incorrect for an autopilot. |
|  4 | buzzer.error_melody     | Melody that is played during error: </br> - 0 means anoying beep melody (500 ms enabled, 500 disabled),</br>- 1 means tolerable melody (500 ms enabled, 2500 ms disabled) </br>- 2 means bimmer melody </br>- [3, 126] - reserved, no melody </br>- 127 constant sound defined by buzzer.frequency and buzzer.duration. |
|  5 | buzzer.arm_melody       | Melody that is played when the vehicle is armed. Not implemented at the momement. |
|  6 | buzzer.frequency        | This parameter is applied only if buzzer.melody=127. It means PWM Frequency, Hz. |
|  7 | buzzer.duration         | This parameter is applied only if buzzer.melody=127. It means PWM duration, us. |
|  8 | gate.threshold          | If gate ADC less than this paramter, so it is broken.</br>If the node detects a broken gate once, it publishes LogMessage every 10 seconds until reboot.</br>The node ignore. |

> This docs was automatically generated. Do not edit it manually.

