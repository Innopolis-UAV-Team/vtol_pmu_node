The node has the following registers:

| №  | Register name           | Description |
| -- | ----------------------- | ----------- |
|  1 | uavcan.node.id          | Defines a node-ID. Allowed values [0,127]. |
|  2 | system.name             | Defines custom node name. If empty, the node will use the default name. |
|  3 | pmu.soc_pct             | Percent of the full charge [-1, 127]: </br> - Value of -1 means auto estimation (doesn't work at the moment), </br> - [0, 100] means constant, </br> - [101, 127] are incorrect values. |
|  4 | buzzer.frequency        | Frequency, Hz |
|  5 | buzzer.duration         | Duration, us |

> This docs was automatically generated. Do not edit it manually.

