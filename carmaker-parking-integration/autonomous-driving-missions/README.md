# autonomous-driving-missions

### How to Start

```
git clone https://github.com/IVS-2/autonomous-driving-missions.git
cd autonomous-driving-missions
```


### 이슈 및 개인 브랜치 생성
1. 이슈 등록 - 개인 업무 요약 및 기능 구현 목록 요약
2. 브랜치 생성 및 이동 `git checkout -b feat/{미션명}-{이름}`

   💡 브랜치 규칙: `feat/{미션명}-{이름}`

### Branches

```
main
│
├── dev                        # 통합 개발 브랜치
└── feat/{미션명}-{이름}        # 각자 기능 개발할 브랜치
```

### Commit Message Convention

```
feat:     새 기능 추가
fix:      버그 수정
refactor: 코드 리팩토링
test:     테스트 추가/수정
docs:     문서 수정 (README 등)
chore:    빌드, 설정 변경
```

#### 예시

```
feat: Mission1 추월 판단 로직 구현
fix: Pure Pursuit look-ahead 거리 계산 오류 수정
refactor: FSM 상태 전환 조건 정리
docs: README 브랜치 전략 추가
```

### 참고
- **개인 브랜치에서 작업 및 정상 동작 확인 후 dev 브랜치로 Pull Request 넣어주세요. (바로 dev 브랜치로 push X)**
- 브랜치 이동 : `git checkout {이동하고 싶은 브랜치명}`
    ```
    ex) git checkout dev
    ```

- 푸시 방법
    ```
    1. git add .
    2. git commit -m "커밋 메세지"
    3. git push origin "본인 브랜치"
    ```

