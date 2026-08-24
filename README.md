# AHB-APB Bridge Protocol Design

## Overview
A structural hardware implementation of an AMBA (Advanced Microcontroller Bus Architecture) AHB to APB Bridge. This module serves as a critical System-on-Chip (SoC) integration component, allowing high-performance, pipelined AHB masters to successfully communicate with slower, unpipelined APB peripherals.

The design utilizes a dedicated Finite State Machine (FSM) to translate AHB transfers into the strict Setup and Enable phases required by the APB protocol, ensuring zero data loss during cross-bus transactions.

## Architecture Highlights
* **AHB Slave Interface:** Handles incoming pipeline requests from the master, latching addresses (`haddr`) and control signals (`hwrite`, `htrans`) to account for the AHB's address-phase to data-phase delay.
* **APB FSM Controller:** A comprehensive state machine that dictates protocol translation. It manages Idle, Read, Write, and Pipelined Burst Write states, accurately generating the `psel`, `penable`, and `pwrite` signals.
* **Burst Transfer Support:** Specifically designed to handle continuous `SEQ` and `NONSEQ` AHB burst transfers without dropping peripheral data.
* **Bus Functional Models (BFMs):** The verification environment includes custom BFMs to simulate the behavior of a genuine AHB Master and APB Slave peripheral.

## Verification
The repository includes a comprehensive testbench (`ahb_apb_tb.v`) that tests the FSM against standard AMBA transaction profiles:
1. **Single Write:** Verifies a standard isolated write transaction to a specific peripheral address.
2. **Single Read:** Asserts read control signals and verifies data retrieval from the APB slave.
3. **Incrementing Burst Write:** Drives a continuous back-to-back write sequence, forcing the bridge to pipeline the APB enable cycles.
