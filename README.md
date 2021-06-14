# Weather Station 2021 Redesign

This repository holds the migration of my earlier Arduino weather station implementation https://github.com/kbarrell/Arduino-WeatherStation.  The redevelopment was prompted by several factors:

- Desire to adopt a more current version of **MCCI LMIC library** (v3.3.0) as the original implementation had followed an available, but early, snapshot of that library and its invocation.
- Desire to move form Arduino IDE to a more functional development environment (**VSC**) and associated multi-target build platform **PlatformIO**.
- Need to migrate the radio networking environment from **TTN**, The Things Network v2, to its replacement, **The Things Stack - Community Edition**, prior to the retirement of v2 operation in 2H21.
- Desire to move to the TTN-preferred Over-The-Air Activation **OTAA** of the weather station node, in place of the static Activation By Personalisation **ABP** that the first implementation used.
- Need to replace the humidity sensor (BME280) in the station node with a new model (SHT31-D) as it had become susceptible to condensation and thus produced erroneous readings of 100% RH for extended periods.  The BME280 is a multi-sensor breakout - it is retained in the implementation for barometric pressure measurement - its temperature and humidity readings are no longer used.

To ease the challenge of the LMIC library migration and to adopt a more structured approach to its process environment setup and calling, I have adopted the comprehensive example **LMIC-node** provided by Leonel Parente's https://github.com/lnlp/LMIC-node.  LMIC-node proveds a very complete, multi-board, multi-region (i.e. radio regulation) environment using PlatformIO. It illustrates a robust approach to developing a node that communicates via TTN.  A couple of notes on the LMIC-node code inclusion in WeatherStn-2021:

- A change in the looping structure from that used by LMIC-node was necessary. The WeatherStn multi-level looping structure and reliance on interrupts with matching ISRs, was incompatible with the simpler design used in LMIC-node.
- The majority of LMIC-node code was left in place, even if not invoked in the new design, as a hedge against future changes in the environment.
- A new Board Support File for an atmMega2560 Arduino processor + Dragino Lora Shield was produced for the LMIC-node environment, to support the hardware platform retained from the original Weather Station implementation.
- While the multi-board environment of LMIC-node offered the opportunity to re-build WeatherStn-2021 for more capable hardware, e.g. Adafruit Feather M0 Lora, the original WeatherStn design relies on the TimerOne library as a fundamental software interrupt scheduler.  The lack of a readily available TimerOne library for the atmelSAM environment of the Feather M0 became a roadblock, and that development path was suspended.

Note:  Please refer to the above-referenced repositories of the two major code components, for details of their design, capabilities and use.