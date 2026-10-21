# Alias Definition

## Gateways

1. Can Box: 51
2. D1616BP IO BOX: 52
    - logic
      - input: a proximity sensor, state stable = high voltage
      - output: an actuator, state active(unlock) = high voltage
    - io
      - sensor1 at input0:1, sensor2 at input0:2
      - actuator1 at output0:1, actuator2 at output0:2
    - script
      - get_tool
        - actuator lock(suppose tool empty), delay, actuator unlock(geting tool attached), sensor stable for 3s, actuator lock
      - switch_tool
        - actuator lock(suppose tool attached), delay, actuator unlock(switching tool attached), sensor stable for 5s, actuator lock
3. RS485 BOX: 53

