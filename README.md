# ft_kalman

UDP 센서 스트림의 IMU 가속도와 GPS 위치를 이용해 3차원 위치를 추정하는 C++17 프로젝트입니다. 클라이언트 `kalman`은 위치·속도 6개 값을 상태로 갖는 칼만 필터를 실행하고, 매 메시지에 대한 추정 위치를 센서 스트림 서버에 응답합니다. 선택적으로 별도 UDP 텔레메트리를 보내 raylib 뷰어에서 이동 궤적을 볼 수 있습니다.

## 동작 개요

1. 클라이언트가 로컬 센서 스트림 서버(`127.0.0.1:4242`)에 `READY`를 보냅니다.
2. 최초 `TRUE POSITION`, `SPEED`, `DIRECTION`으로 위치와 속도를 초기화합니다.
3. 이후 메시지마다 `ACCELERATION`으로 상태를 예측하고, `POSITION` GPS 값이 있으면 위치를 보정합니다.
4. 추정 위치를 `X Y Z` 형식으로 서버에 응답합니다. 서버가 `GOODBYE.`를 보내면 종료합니다.

초기화 이후의 `DIRECTION`과 디버그용 실제 위치·속도는 필터 관측값으로 사용하지 않습니다. 필터의 가속도 및 GPS 잡음 분산은 고정값이며 클라이언트에서 조정하는 옵션은 없습니다. 과제의 상세 요구사항은 [ft_kalman.pdf](ft_kalman.pdf)를 참고하세요.

## 준비와 빌드

- C++17 컴파일러와 `make`
- 과제에서 제공하는 `imu-sensor-stream-macos` 실행 파일 (센서 스트림 서버)
- 뷰어를 사용할 경우에만 `raylib`과 `pkg-config` (`macOS`: `brew install raylib pkg-config`)

```sh
make                 # kalman 빌드
make visualizer      # 선택 사항: kalman_visualizer 빌드
```

`make`와 테스트에는 raylib가 필요하지 않습니다. 센서 스트림 실행 파일은 Git에 포함되지 않으므로 별도로 준비해 프로젝트 루트에 놓으세요.

## 실행

서버와 클라이언트를 각각 다른 터미널에서 실행합니다. 다음 예시는 제공된 macOS 센서 스트림으로 1분간 실행합니다. Apple Silicon에서는 x86_64 실행 파일을 Rosetta로 실행할 수 있어야 합니다.

```sh
# 터미널 1: 센서 스트림 서버
arch -x86_64 ./imu-sensor-stream-macos -s 42 -d 1 -p 4242

# 터미널 2: 위치 추정 클라이언트
./kalman --no-telemetry
```

클라이언트는 센서 스트림 서버의 `127.0.0.1:4242`에 접속하도록 고정되어 있으므로 서버도 포트 `4242`로 실행해야 합니다. 서버 옵션은 `./imu-sensor-stream-macos --help`, 클라이언트 옵션은 `./kalman --help`에서 확인할 수 있습니다.

| 클라이언트 옵션 | 설명 |
| --- | --- |
| `--telemetry-host <IPv4 주소>` | 텔레메트리 수신 주소 (기본값 `127.0.0.1`) |
| `--telemetry-port <1..65535>` | 텔레메트리 포트 (기본값 `4243`) |
| `--no-telemetry` | 텔레메트리 전송 비활성화 |
| `--help` | 사용법 출력 |

텔레메트리는 기본으로 활성화되지만 뷰어가 없어도 위치 추정은 계속됩니다.

### 3D 궤적 뷰어

`make visualizer`로 빌드한 뒤 세 터미널에서 뷰어, 서버, 클라이언트를 실행합니다. 뷰어와 클라이언트의 텔레메트리 포트가 같아야 합니다.

```sh
# 터미널 1
./kalman_visualizer --port 4243

# 터미널 2
arch -x86_64 ./imu-sensor-stream-macos -s 42 -d 1 -p 4242

# 터미널 3
./kalman --telemetry-port 4243
```

뷰어는 추정 궤적 전체를 기본 화면에 맞춰 표시합니다. X/Y는 수평, Z는 높이입니다. 스트림이 끝난 뒤에도 마지막 궤적이 남습니다.

| 조작 | 동작 |
| --- | --- |
| `H` / `F` | 전체 궤적 맞춤 / 최신 위치 따라가기 |
| `1` / `2` / `3` | 위 / 정면 / 측면 시점 |
| 왼쪽 드래그 | 화면 회전 |
| 오른쪽 드래그 / 휠 | 화면 이동 / 확대·축소 |
| `WASD`, `Q`/`E`, `Shift` | 수평 이동, 높이 이동, 빠른 이동 |
| `Space` / `R` | 궤적 기록 일시정지 / 기록 삭제 |
| `G` / `C` / `Tab` | GPS 점 / 공분산 / 상세 정보 표시 전환 |
| `Esc` | 뷰어 종료 |

## 테스트

```sh
make test       # 단위 및 통합 테스트
make sanitize   # AddressSanitizer·UndefinedBehaviorSanitizer로 테스트
```

빌드 산출물은 `make clean`(오브젝트 파일) 또는 `make fclean`(실행 파일 포함)으로 삭제할 수 있습니다.
