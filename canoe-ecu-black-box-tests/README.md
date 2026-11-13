# CANoe ECU 블랙박스 검증

CANoe와 CAPL로 교육용 ECU의 고장 검출·회복 시험을 자동화했습니다. 요구사항별 사전 조건, 기대 상태 전이, 허용 시간을 정리하고 CAN 응답을 기준으로 합격 여부를 판단했습니다. ECU 내부 태스크의 실행 주기는 10 ms였으며, 외부에서 상태를 관측한 시점을 해석할 때 이 주기를 고려했습니다.

CAPL 원본과 실행 리포트는 현재 확보한 자료에서 찾지 못했습니다.

---

## English

## ECU Black-Box Verification

I used CANoe and CAPL to automate requirements-based verification for a training ECU. The ECU reported fault status through its CAN interface.

### From requirement to test

- Defined the preconditions, expected state transitions, and timing windows for fault detection and recovery.
- Used a CAN database to represent observable inputs and outputs. Automated scenario execution and pass/fail checks with CAPL.
- Interpreted external CAN timestamps in the context of the ECU's 10 ms task cycle. The timestamps describe when the status was observable at the interface.

| Example check | Evidence |
| --- | --- |
| Fault detection | Observed state and elapsed time after the specified condition |
| Fault recovery | Observed transition after the recovery conditions were met |
| Status reporting | Returned state compared with the expected result |

The verification scope was functional fault handling on a training ECU.

The original CAPL scripts and execution reports were unavailable in the source material.
