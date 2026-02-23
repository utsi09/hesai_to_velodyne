# PTP 시간 동기화 가이드 (Hesai AT128 + Jetson Orin)

## 목차

1. [PTP란?](#1-ptp란)
2. [전체 구성도](#2-전체-구성도)
3. [네트워크 환경 확인](#3-네트워크-환경-확인)
4. [Orin에 linuxptp 설치](#4-orin에-linuxptp-설치)
5. [하드웨어 타임스탬프 지원 확인](#5-하드웨어-타임스탬프-지원-확인)
6. [ptp4l 설정 (PTP Master)](#6-ptp4l-설정-ptp-master)
7. [phc2sys 설정 (시스템 클럭 동기화)](#7-phc2sys-설정-시스템-클럭-동기화)
8. [Hesai AT128 PTP 설정](#8-hesai-at128-ptp-설정)
9. [Hesai ROS2 드라이버 PTP 설정](#9-hesai-ros2-드라이버-ptp-설정)
10. [동기화 확인](#10-동기화-확인)
11. [서비스로 등록 (부팅 시 자동 실행)](#11-서비스로-등록-부팅-시-자동-실행)
12. [IMU 시간 동기화](#12-imu-시간-동기화)
13. [트러블슈팅](#13-트러블슈팅)
14. [참고 자료](#14-참고-자료)

---

## 1. PTP란?

PTP(Precision Time Protocol, IEEE 1588v2)는 네트워크를 통해 장치 간 시간을 **마이크로초(us) 수준**으로 동기화하는 프로토콜입니다.

**왜 필요한가?**

LiDAR와 IMU의 타임스탬프가 다른 시간 기준을 사용하면, LIO-SAM 같은 센서 퓨전 알고리즘이 데이터를 매칭할 수 없습니다.

| 센서 | 동기화 전 | 동기화 후 |
|------|----------|----------|
| Hesai AT128 | 내부 클럭 (2020년...) | Orin 시스템 시간 |
| IMU (Microstrain) | Orin 시스템 시간 | Orin 시스템 시간 |

PTP를 사용하면 **모든 센서가 같은 시간 기준**을 사용하게 됩니다.

**역할 정리:**

| 역할 | 장치 | 설명 |
|------|------|------|
| PTP Master (Grandmaster) | Jetson Orin | 시간의 기준. 시스템 클럭을 PTP로 방송 |
| PTP Slave | Hesai AT128 | Master의 시간에 자신의 클럭을 맞춤 |

---

## 2. 전체 구성도

```
┌─────────────────────┐          이더넷          ┌──────────────────┐
│   Jetson Orin        │ ◄──────────────────────► │  Hesai AT128     │
│                      │    192.168.1.x 대역       │                  │
│  [ptp4l]  PTP Master │ ────── PTP Sync ───────► │  PTP Slave       │
│  [phc2sys] 클럭 동기  │                          │  (자동 설정)      │
│  [ROS2 드라이버]      │ ◄── /lidar_points ────── │  포인트클라우드    │
│  [IMU 드라이버]       │ ◄── /imu/data            │                  │
└─────────────────────┘                           └──────────────────┘
```

---

## 3. 네트워크 환경 확인

### 3.1 Orin과 LiDAR가 같은 서브넷인지 확인

```bash
# Orin의 IP 확인
ip addr show

# 예: Orin이 192.168.1.100 이면
# LiDAR 기본 IP는 192.168.1.201
# 같은 서브넷 (192.168.1.x) → OK
```

### 3.2 LiDAR 통신 확인

```bash
ping 192.168.1.201
```

응답이 오면 네트워크 연결 OK.

### 3.3 필요한 포트 (방화벽 확인)

| 포트 | 프로토콜 | 용도 |
|------|---------|------|
| 319 | UDP | PTP Event 메시지 |
| 320 | UDP | PTP General 메시지 |
| 2368 | UDP | LiDAR 데이터 |
| 9347 | TCP | PTC 제어 프로토콜 |
| 80 | TCP | LiDAR 웹 인터페이스 |

```bash
# 방화벽 열기 (필요 시)
sudo ufw allow 319/udp
sudo ufw allow 320/udp
sudo ufw allow 2368/udp
sudo ufw allow 9347/tcp
```

---

## 4. Orin에 linuxptp 설치

```bash
sudo apt update
sudo apt install linuxptp ethtool
```

설치 확인:

```bash
ptp4l -v
phc2sys -v
```

---

## 5. 하드웨어 타임스탬프 지원 확인

```bash
# <인터페이스>를 실제 네트워크 인터페이스명으로 교체 (예: eth0, eno1, enp1s0)
# 인터페이스명 확인:
ip link show

# 하드웨어 타임스탬프 확인:
sudo ethtool -T <인터페이스>
```

**결과 확인:**

```
Time stamping parameters for eth0:
Capabilities:
        hardware-transmit     (SOF_TIMESTAMPING_TX_HARDWARE)    ← 있으면 HW 지원
        hardware-receive      (SOF_TIMESTAMPING_RX_HARDWARE)    ← 있으면 HW 지원
        hardware-raw-clock    (SOF_TIMESTAMPING_RAW_HARDWARE)   ← 있으면 HW 지원
        software-transmit     (SOF_TIMESTAMPING_TX_SOFTWARE)
        software-receive      (SOF_TIMESTAMPING_RX_SOFTWARE)
```

- **hardware-transmit/receive 있음** → 하드웨어 PTP 지원 (최고 정밀도, us 수준)
- **software만 있음** → 소프트웨어 PTP만 가능 (정밀도 낮음, ms 수준이지만 충분함)

> **참고:** Jetson Orin의 내장 이더넷은 HW 타임스탬프를 지원하지 않을 수 있습니다.
> 소프트웨어 모드(`-S` 옵션)로도 센서 퓨전에 충분한 정밀도를 얻을 수 있습니다.

---

## 6. ptp4l 설정 (PTP Master)

ptp4l은 Orin을 **PTP Master(Grandmaster)**로 만들어, 네트워크에 시간을 방송합니다.

### 6.1 설정 파일 생성

```bash
sudo tee /etc/linuxptp/hesai_ptp.conf << 'EOF'
[global]
# PTP 프로파일
priority1               128
priority2               128
clockClass              128
# clockClass 128 = free-running clock (외부 기준 없이 자체 시스템 클럭 사용)

# Master 전용 모드
masterOnly              1

# 로깅
logging_level           6
verbose                 1
summary_interval        1

# 타임스탬프 모드 (하드웨어 지원 시 자동 감지)
time_stamping           auto

# PTP 도메인 (LiDAR와 같은 도메인이어야 함, 기본값 0)
domainNumber            0

# 네트워크 전송 모드 (L2 또는 UDPv4)
network_transport       UDPv4
EOF
```

### 6.2 실행 (테스트)

```bash
# 인터페이스를 실제 이름으로 교체

# --- 하드웨어 타임스탬프 지원 시 ---
sudo ptp4l -i <인터페이스> -f /etc/linuxptp/hesai_ptp.conf -m

# --- 소프트웨어 타임스탬프만 지원 시 (-S 추가) ---
sudo ptp4l -i <인터페이스> -f /etc/linuxptp/hesai_ptp.conf -m -S
```

**정상 출력 예시:**

```
ptp4l[12345.678]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[12345.678]: selected local clock xxxxxx.fffe.xxxxxx as best master
ptp4l[12345.678]: assuming the grand master role
```

> `assuming the grand master role`이 나오면 성공!

### 6.3 옵션 설명

| 옵션 | 설명 |
|------|------|
| `-i <인터페이스>` | LiDAR가 연결된 네트워크 인터페이스 |
| `-f <설정파일>` | 설정 파일 경로 |
| `-m` | 로그를 stdout에 출력 |
| `-S` | 소프트웨어 타임스탬프 강제 사용 |
| `--masterOnly=1` | Master 전용 (설정 파일 대신 CLI로도 가능) |

---

## 7. phc2sys 설정 (시스템 클럭 동기화)

phc2sys는 **시스템 클럭(CLOCK_REALTIME)**과 **PTP 하드웨어 클럭(PHC)**을 동기화합니다.

Orin이 Master이므로, 시스템 클럭 → PHC 방향으로 동기화합니다.

### 7.1 하드웨어 타임스탬프 지원 시

```bash
# 시스템 클럭(CLOCK_REALTIME)을 PTP 하드웨어 클럭에 동기화
sudo phc2sys -s CLOCK_REALTIME -c <인터페이스> -m -O 0 -w
```

### 7.2 소프트웨어 타임스탬프만 지원 시

소프트웨어 모드에서는 phc2sys가 **필요 없습니다**.
ptp4l이 `-S` 옵션으로 직접 시스템 클럭을 사용하기 때문입니다.

### 7.3 옵션 설명

| 옵션 | 설명 |
|------|------|
| `-s CLOCK_REALTIME` | 소스: 시스템 클럭 |
| `-c <인터페이스>` | 목적지: 해당 인터페이스의 PHC |
| `-m` | 로그를 stdout에 출력 |
| `-O 0` | UTC-TAI 오프셋을 0으로 (중요!) |
| `-w` | ptp4l이 준비될 때까지 대기 |

> **중요: `-O 0` 옵션**
>
> PTP는 내부적으로 TAI(국제원자시)를 사용하는데, 시스템 클럭은 UTC를 사용합니다.
> TAI와 UTC 사이에는 약 37초의 차이(leap seconds)가 있습니다.
> `-O 0`을 지정하면 이 오프셋을 무시하고, 시스템 클럭 시간을 그대로 사용합니다.
> Hesai LiDAR는 UTC 기준으로 동작하므로 `-O 0`이 올바릅니다.

---

## 8. Hesai AT128 PTP 설정

Hesai AT128은 **PTP Slave가 기본 설정**이며, Plug & Play로 동작합니다.
PTP Master가 네트워크에 있으면 자동으로 동기화됩니다.

### 8.1 웹 인터페이스로 확인/설정

1. 브라우저에서 **http://192.168.1.201** 접속
2. 왼쪽 메뉴에서 **설정(Settings)** 또는 **Time Sync** 메뉴 선택

### 8.2 확인할 설정값

| 항목 | 값 | 설명 |
|------|-----|------|
| PTP Profile | **IEEE 1588v2** | PTPv2 프로토콜 |
| PTP Transport | **UDP/IPv4** | 네트워크 전송 방식 (ptp4l 설정과 일치해야 함) |
| PTP Domain | **0** | ptp4l의 domainNumber와 일치해야 함 |

### 8.3 동기화 상태 확인

웹 인터페이스 **Home** 페이지에서:

| 항목 | 의미 |
|------|------|
| PTP Status: **Locked** | 동기화 성공! |
| PTP Status: **Free Run** | 동기화 안 됨 (Master를 못 찾음) |
| PTP Status: **Tracking** | 동기화 진행 중 (잠시 기다리면 Locked로 전환) |

> **Locked**가 되면 LiDAR의 포인트클라우드 타임스탬프가 Orin의 시스템 시간과 일치합니다.

---

## 9. Hesai ROS2 드라이버 PTP 설정

### 9.1 config.yaml 설정

```yaml
lidar:
  - driver:
      source_type: 1                          # 1 = 라이브 UDP (실시간 센서)
      lidar_udp_type:
        device_ip_address: 192.168.1.201
        udp_port: 2368
        ptc_port: 9347
        use_ptc_connected: true               # PTC 연결 활성화 (PTP 상태 조회용)
      use_timestamp_type: 0                   # 0 = 포인트클라우드 타임스탬프 사용 (PTP 동기화된 시간)
                                              # 1 = SDK 수신 시간 사용 (PTP 없을 때)
    ros:
      ros_frame_id: hesai_lidar
      ros_send_point_cloud_topic: /lidar_points
      ros_send_ptp_topic: /lidar_ptp          # PTP 상태 토픽 활성화 (주석 해제)
      send_point_cloud_ros: true
```

### 9.2 핵심 파라미터

| 파라미터 | PTP 동기화됨 | PTP 없음 |
|---------|------------|---------|
| `use_timestamp_type` | **0** (포인트클라우드 시간) | 1 (SDK 수신 시간) |
| `ros_send_ptp_topic` | `/lidar_ptp` (상태 모니터링) | 주석 처리 |

### 9.3 PTP 상태 모니터링

```bash
# PTP 상태 토픽 확인
ros2 topic echo /lidar_ptp
```

출력:
```yaml
ptp_lock_offset: 0    # 0이면 정상 (Locked)
ptp_status: [...]     # 16바이트 상태 배열
```

---

## 10. 동기화 확인

### 10.1 타임스탬프 비교 (가장 중요!)

```bash
# 터미널 1: LiDAR 타임스탬프
ros2 topic echo /lidar_points --no-arr --field header.stamp --once

# 터미널 2: IMU 타임스탬프
ros2 topic echo /imu/data --field header.stamp --once
```

**성공 예시:**
```
# LiDAR
sec: 1770880187
nanosec: 613502026

# IMU
sec: 1770880187
nanosec: 758985519
```

두 `sec` 값이 같거나 1초 이내 차이 → 동기화 성공!

**실패 예시:**
```
# LiDAR
sec: 1593495378    ← 2020년 (PTP 미동기)

# IMU
sec: 1770880187    ← 2026년
```

### 10.2 ptp4l 동기화 상태

```bash
# ptp4l 로그에서 slave 연결 확인
journalctl -f -u ptp4l
```

### 10.3 Hesai 웹 인터페이스

브라우저에서 http://192.168.1.201 → Home → PTP Status가 **Locked**인지 확인

---

## 11. 서비스로 등록 (부팅 시 자동 실행)

매번 수동으로 실행하지 않도록 systemd 서비스로 등록합니다.

### 11.1 ptp4l 서비스

```bash
sudo tee /etc/systemd/system/hesai-ptp4l.service << 'EOF'
[Unit]
Description=PTP Master for Hesai LiDAR
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
# 인터페이스명을 실제 이름으로 교체하세요
# 소프트웨어 모드는 -S 추가
ExecStart=/usr/sbin/ptp4l -i <인터페이스> -f /etc/linuxptp/hesai_ptp.conf -S
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
```

### 11.2 phc2sys 서비스 (하드웨어 타임스탬프 지원 시만)

```bash
sudo tee /etc/systemd/system/hesai-phc2sys.service << 'EOF'
[Unit]
Description=PHC to System Clock Sync for Hesai PTP
After=hesai-ptp4l.service
Requires=hesai-ptp4l.service

[Service]
Type=simple
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c <인터페이스> -O 0 -w
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
```

### 11.3 서비스 활성화

```bash
sudo systemctl daemon-reload

# ptp4l 활성화
sudo systemctl enable hesai-ptp4l
sudo systemctl start hesai-ptp4l
sudo systemctl status hesai-ptp4l

# phc2sys 활성화 (HW 타임스탬프 지원 시만)
sudo systemctl enable hesai-phc2sys
sudo systemctl start hesai-phc2sys
sudo systemctl status hesai-phc2sys
```

---

## 12. IMU 시간 동기화

### Microstrain IMU

Microstrain ROS2 드라이버는 기본적으로 **호스트(Orin)의 시스템 클럭**을 `header.stamp`에 사용합니다.
따라서 PTP로 LiDAR를 Orin 시간에 동기화하면, IMU와 자동으로 시간이 맞습니다.

확인사항:
- Microstrain 드라이버의 `use_device_timestamp` 파라미터가 **false** (기본값)인지 확인
- true로 설정하면 IMU 내부 클럭을 사용하므로 동기화가 깨질 수 있음

```bash
# 확인
ros2 param get /microstrain_inertial_driver use_device_timestamp
```

---

## 13. 트러블슈팅

### PTP Status가 "Free Run"에서 변하지 않음

| 확인사항 | 해결 |
|---------|------|
| ptp4l이 실행 중인가? | `systemctl status hesai-ptp4l` |
| 같은 서브넷인가? | `ping 192.168.1.201` |
| PTP 도메인이 같은가? | ptp4l: `domainNumber 0`, LiDAR 웹: Domain 0 |
| 전송 방식이 같은가? | ptp4l: `UDPv4`, LiDAR 웹: UDP/IPv4 |
| 방화벽이 포트 319,320을 막고 있는가? | `sudo ufw allow 319/udp && sudo ufw allow 320/udp` |
| 멀티캐스트가 활성화되어 있는가? | `sudo ip link set <인터페이스> multicast on` |

### 타임스탬프가 약 37초 차이남

TAI-UTC 오프셋 문제입니다. phc2sys에 `-O 0` 옵션을 추가하세요.

### 타임스탬프가 약 20년 차이남

LiDAR의 PTP가 동기화되지 않았습니다. (Free Run 상태)
웹 인터페이스에서 PTP Status를 확인하세요.

### "Large velocity, reset IMU-preintegration!" (LIO-SAM)

타임스탬프 동기화 후에도 발생하면 extrinsic 캘리브레이션 문제입니다.
`extrinsicRot`, `extrinsicRPY`, `extrinsicTrans` 값을 확인하세요.

---

## 14. 참고 자료

- [Hesai AT128P User Manual (PTP 섹션)](https://www.hesaitech.com/wp-content/uploads/2025/04/AT128P_User_Manual_A02-en-250410.pdf)
- [Hesai 소프트웨어 다운로드](https://www.hesaitech.com/downloads/)
- [Ouster PTP Quickstart Guide (linuxptp 참고)](https://static.ouster.dev/sensor-docs/image_route1/image_route2/appendix/ptp-quickstart.html)
- [linuxptp 공식 문서](https://linuxptp.nwtime.org/documentation/ptp4l/)
- [phc2sys 공식 문서](https://linuxptp.nwtime.org/documentation/phc2sys/)
- [SBG Systems Hesai LiDAR 연동 가이드](https://support.sbg-systems.com/sc/hp/latest/how-to-articles/lidar-integration)
- [PTP 시간 동기화 상세 설명 (Ruixiang's Notes)](https://notes.rdu.im/system/network/time-sync-using-ptp/)

---

## 빠른 시작 요약 (Quick Start)

```bash
# 1. 설치
sudo apt install linuxptp ethtool

# 2. 하드웨어 타임스탬프 확인
sudo ethtool -T <인터페이스>

# 3. ptp4l 실행 (소프트웨어 모드)
sudo ptp4l -i <인터페이스> -m --masterOnly=1 -S

# 4. (HW 지원 시만) phc2sys 실행
sudo phc2sys -s CLOCK_REALTIME -c <인터페이스> -m -O 0 -w

# 5. Hesai 웹 인터페이스에서 PTP Status: Locked 확인
#    http://192.168.1.201

# 6. 타임스탬프 비교
ros2 topic echo /lidar_points --no-arr --field header.stamp --once
ros2 topic echo /imu/data --field header.stamp --once
# → 두 sec 값이 같으면 성공!
```
