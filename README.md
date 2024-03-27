# VTOL PMU DroneCAN application

> Developers docs: [DroneCAN template application](https://github.com/RaccoonlabDev/mini_v2_node/wiki/DroneCAN-application).

### 1. Brief hardware info

| View | STM32CubeMX | Pinout |
| ---- | ----------- | ------ |
| <img src="Assets/view.jpg" alt="drawing" width="200"> | <img src="https://github.com/Innopolis-UAV-Team/vtol-pmu-ioc/raw/6530f3319c28916c0268c4aedbf7e330f0dfdd90/Assets/stm32cubemx.png" alt="drawing" width="200"> | pinout |

### 2. Application description and software design

The PMU node do 3 things:
1. Battery. This module measures ADC voltage and current and estimates the battery state.
2. Gate monitor. This module measures gates voltages and estimates the corectness of the PMU and publishes the LogMessage if error is detected.
3. Buzzer. This module notifies with sound about such events as dead gates or arming state of the vehicle

From the software side, the high level node design can be illustrated with the following UML diagramm: 

<img src="Assets/uml.png" alt="drawing">

More details about each module:

1. Battery sends [BatteryInfo](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#batteryinfo) message with 1 Hz rate.

<img src="Assets/battery_info.png" alt="drawing">

2. Gate monitor checks the gates

| DMA Rank | Periphery   | Pin  | Meaning        | Note |
| -------- | ----------- | ---- | -------------- | ---- |
| 1 | ADC_IN2 | PA2 | ADC_GATE_2 | If voltage < 1.4 V, so it is broken. |
| 3 | ADC_IN4 | PA4 | ADC_GATE_3 | If voltage < 1.4 V, so it is broken. |
| 4 | ADC_IN6 | PA6 | ADC_GATE_4 | If voltage < 1.4 V, so it is broken. |

If at least one gate is broken, it starts to notify with LogMessage each 10 seconds until the node is rebooted. The LogMessage has a number of broken gates.

The node ignores all broken states first 5 seconds to prevent false positive failures.

3. Buzzer notifies with sound about dead gates or arming state

> arming state will appear later...

### 3. Test Cases

1. TestBatteryInfo
    - TC1. Publish rate must be 1 Hz (plus minus 50 ms is acceptable)
    - TC2. Votlage should be within [4.5, 60.0] Volts
    - TC3. Set a random value between [0, 100] `battery.soc_pct`, so `msg.state_of_charge_pct` should be the same
    - TC4. The same for `battery.battery_id`
    - TC5. The same for `battery.model_instance_id`
    - TC6. Similar for pair `battery.capacity_mah` and `battery.full_voltage`, but check `msg.full_charge_capacity_wh` value according to a formula
    - TC7. We should also check `msg.remaining_capacity_wh`. Well, it seems to be we should check it manually.
2. TestBuzzer
    - I don't know how to automatically test it...
4. TestGateMonitor
    - TC1. Reboot the node.
      Set `gate.threshold` to 0 (that means gates are always ok).
      Wait for 15 seconds.
      Debug message should not appear.
      The NodeStatus should have good health.
    - TC2. Reboot the node.
      Set `gate.threshold` to 4095 (that means gates are always bad).
      Verify, that the node publish LogMessage with 0.1 Hz rate (burst publishing are not allowed, first 5 second node publish nothing)
      The NodeStatus should have CRITICAL health.

### 4. How to upload a firmware?

<img src="Assets/swd.png" alt="drawing" width="400">

### 5. Notes

**Current ADC**

1. Accuracy

```
ADC_RESOLUTION = 12 bit
ADC_RAW_MAX = 4095
MAX_CURRENT = 600 A
CURRENT_SENSETIVITY = 146.5 mA
```

2. Frequency

```
MCU_FREQUENCY = 72 000 000
ADC_FREQUENCY = 12 000 000
NUMBER_OF_CONVERSION = 7
SAMPLING_TIME = 55.5 Cycles
THEORY_GROUP_FREQUENCY = 30 888 Measurements/sec
ACTUATL_GROUP_FREQUENCY = 22 175 Measurements/sec
```
