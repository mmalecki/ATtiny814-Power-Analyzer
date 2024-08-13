# v2r0

## Hardware
* Migrated to KiCad
* Migrate from USB B to USB C
* Migrate from CH330N to FT232 due to part availability
* Migrate to direct current control loop as opposed to shunt-based method, saving board space and heat concerns
* Drastically reduce DAC output capacitor, change its purpose from seemingly sampling to filtering, removing seesaw effect (max allowed capacitance is 30 pF)
* Add a low-pass RC filter on both input and output of the opamp
* Move OpAmp closer to DAC, reducing noise pick up
* Add sampling capacitor to NTC

## Software
* Add direct NTC probe temperature calculations
* Refactor to control-loop-based current control
* Implement overload notification
* Start using the LED for error and status notifications (various blinks)
* Implement reporting and adjusting of programs at different intervals
* General clean-up
* Bundle multimeter (`m`) and transmit sensors (`t`) into one, with optional timing info
* Add some debug commands
* Change `SEPARATOR` to character
