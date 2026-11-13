# ECU Black-Box Verification

I used CANoe and CAPL to automate requirements-based verification for a training ECU. The ECU reported fault status through its CAN interface.

## From requirement to test

- Defined the preconditions, expected state transitions, and timing windows for fault detection and recovery.
- Used a CAN database to represent observable inputs and outputs. Automated scenario execution and pass/fail checks with CAPL.
- Interpreted external CAN timestamps in the context of the ECU's 10 ms task cycle. The timestamps describe when the status was observable at the interface.

| Example check | Evidence |
| --- | --- |
| Fault detection | Observed state and elapsed time after the specified condition |
| Fault recovery | Observed transition after the recovery conditions were met |
| Status reporting | Returned state compared with the expected result |

The verification scope was functional fault handling on a training ECU.
