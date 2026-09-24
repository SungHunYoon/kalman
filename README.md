# ft_kalman

센서 스트림에서 받은 IMU 가속도와 GPS 위치로 3차원 위치를 추정하는 C++17 클라이언트다. `ft_kalman.pdf`의 기본 과제를 위한 `KalmanFilter`는 처음 구현한 동적 행렬 버전으로 복귀했다. 별도 UDP 텔레메트리와 raylib 3D 뷰어는 유지한다.

## 현재 범위

- 상태는 위치 3축과 속도 3축이다. 최초 위치·속력·방향으로 초기화한다.
- 이후 각 패킷에서 가속도로 예측하고, `POSITION` GPS가 있으면 항상 보정한다. 이후 `DIRECTION`은 필터 갱신에 사용하지 않는다.
- 원래 필터의 고정 분산은 가속도 `1e-4`, GPS `1.0`이다. `--accel-sigma`, `--gps-sigma`, `--gate-threshold` 클라이언트 옵션은 없다.
- 추가 정확도 보너스(고잡음 `--noise 10` 통과), 방향 관측, GPS gating, 적응형 잡음은 범위에서 제외했다. 필수 과제의 정확도 요구사항 자체를 없앤 것은 아니다. 필터 복귀 이후 장기 정확도는 재검증하지 않았으므로 통과를 주장하지 않는다.
- 기존 평균 `0.05 ms` 보너스 성능 목표는 현재 합격 기준이 아니다. 원래 필터의 응답 시간은 실측값 그대로 기록한다.

## 빌드와 테스트

```sh
make
make test
make sanitize
make visualizer  # raylib와 pkg-config가 설치된 macOS
```

`make visualizer`에는 raylib가 필요하다. macOS에서는 `brew install raylib pkg-config`로 설치할 수 있다. 일반 `make`와 테스트에는 raylib가 필요 없다.

## 센서 스트림 실행

서버와 클라이언트를 별도 터미널에서 순서대로 실행한다. Apple Silicon에서는 제공된 x86_64 센서 스트림을 Rosetta로 실행한다.

```sh
arch -x86_64 ./imu-sensor-stream-macos -s 42 -d 1 -p 4242 --filterspeed
./kalman --no-telemetry
```

클라이언트는 `127.0.0.1:4242`로 `READY\n`을 보낸다. `MSG_START`/`MSG_END` 사이의 필드들을 조립하고, 초기 `TRUE POSITION`, `SPEED`, `DIRECTION`을 받은 뒤 위치 응답 `X Y Z\n`을 전송한다. 가속도 `ACCELERATION`은 매번, GPS `POSITION`은 있을 때만 처리한다. `GOODBYE.`를 받으면 정상 종료한다. 디버그 모드의 반복 `TRUE POSITION`과 `SPEED`는 필터의 추가 관측으로 사용하지 않는다.

클라이언트 옵션은 다음과 같다.

```text
--telemetry-host <IPv4 address>   기본값 127.0.0.1
--telemetry-port <1..65535>       기본값 4243
--no-telemetry                    별도 텔레메트리 비활성화
--help
```

서버 `--noise`는 센서 입력만 바꾸며 클라이언트의 고정 필터 분산은 바꾸지 않는다. 고잡음 정확도 합격 행렬은 현재 보너스 목표가 아니다.

## 텔레메트리와 뷰어

센서 스트림에 위치를 응답한 다음, 클라이언트는 기본적으로 `127.0.0.1:4243`에 non-blocking UDP 텔레메트리를 발행한다. 별도 수신기나 뷰어가 없어도 필터는 실행된다. 버전 1 패킷(216바이트) 형식은 이전 뷰어와의 호환을 위해 유지했다. 위치·속도·가속도, GPS와 보정 전 innovation, 위치 공분산, 필터 처리 시간 등이 담긴다. 원래 필터는 GPS를 거부하지 않으므로 수신된 GPS는 모두 `accepted`로 표시되고 rejected 수는 0이다. 기존 `adaptive_gps_variance` 필드에는 고정 GPS 분산 `1.0`을 넣는다.

세 터미널에서 뷰어를 함께 실행할 수 있다.

```sh
./kalman_visualizer --port 4243
arch -x86_64 ./imu-sensor-stream-macos -s 42 -d 1 -p 4242 --filterspeed
./kalman --telemetry-port 4243
```

뷰어는 위치 궤적, GPS 점, 속도 방향, 위치 공분산과 처리 시간 통계를 표시한다. `F`는 위치 추적, `Space`는 궤적 기록 일시정지, `R`은 궤적 삭제, `G`/`C`는 GPS 점/공분산 표시 전환, `Esc`는 뷰어 종료다. 뷰어 종료는 센서 스트림 응답을 중단시키지 않아야 한다.

## 검증 상태

2026-09-24 Apple M5 macOS에서 원래 필터로 복귀한 뒤 `make`, `make test`, `make sanitize`, `make visualizer`가 통과했다. raylib 링크 시 라이브러리가 더 새 macOS 버전용으로 빌드됐다는 경고는 출력됐으나 실행 파일은 만들어지고 창이 열렸다.

| 실행 | 서버 `--filterspeed` 원시 출력 | 내부 필터 평균 | 결과 |
|---|---:|---:|---|
| seed 42, 기본 잡음, 1분, 텔레메트리 끔 | `0.000191381863454484` | `7.24732 µs` | 서버·클라이언트 종료 코드 0, `GOODBYE.` 수신 |
| seed 42, 기본 잡음, 1분, 뷰어 연결 | `0.0001729217872624213` | `6.69932 µs` | 서버·클라이언트 종료 코드 0, 뷰어에서 최종 seq 5997 및 GPS 적용 19회 확인 |
| seed 42, 기본 잡음, 10분, 뷰어 종료 이후 | `0.00016918986026200926` | `6.73277 µs` | 서버·클라이언트 종료 코드 0, `GOODBYE.` 수신, 텔레메트리 dropped 111 |

서버 출력은 숫자에 단위가 붙지 않는다. 기존 원래 필터의 약 `0.166 ms` 기록 및 내부 필터 처리 시간과 비교하면 서버 수치를 초 단위로 해석하는 것이 타당하다. 이 해석이라면 첫 실행의 평균 응답은 약 `0.191 ms`로, 이전 `0.05 ms` 보너스 목표에 미달한다. 원래 필터 그대로 복귀하기로 한 선택에 따라 이 목표는 합격 기준으로 사용하지 않는다. 이전 고정 크기 필터나 방향 관측의 벤치마크는 현재 구현의 성능 수치가 아니다. 복귀 후 90분·다중 seed 정확도는 다시 검사하지 않았다.

