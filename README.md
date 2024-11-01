## 프로젝트 개요
  - 중소공장의 협소한 실내 공간 내에서 빈번하게 변화하는 레이아웃에 능동적으로 반응할 수 있는 대차형 AMR 프로젝트
  - [시연 영상](https://youtu.be/TZNeZHwiGjE?feature=shared)

### 목차
- [프로젝트 개요](#프로젝트-개요)
  - [목차](#목차)
- [프로젝트 정보](#프로젝트-정보)
- [아이디어 기획](#아이디어-기획)
- [하드웨어](#하드웨어)
- [맡은 업무](#맡은-업무)

---
## 완성 사진

![AMR](https://github.com/user-attachments/assets/77dc6605-6a2b-4b24-8758-3f15451ee303)
---

## 프로젝트 정보

|프로젝트| 설명 | 기타 |
|:---:|:---:|:---:|
| 프로젝트 기간| 2023.05.01 ~ 2024.09.28| |
| 프로젝트 성격 | 캡스톤디자인(졸업작품) | |
| 개발 인원 | 팀 / 4명 | [장영도](https://github.com/jangyoungdo), [박지호](https://github.com/JJo-0), [이건원](https://github.com/leegunwon), [조성우](https://github.com/) |
| 하드웨어 | raspberry pi 5, mdrobot 등.. [링크](##하드웨어) | |


   
쓰는 cpu 버전에 따라서 달라진다.  
사용 및 검증 버전   
**arm64(aarch64)**
- raspberry pi 4
- raspberry pi 5
- AGX Xavier
   
**x86_64**
- MINIX (n100 미니 pc)
- Intel Cpu 데스크탑
   
[cpu_아키텍처_종류_설명링크](https://velog.io/@480/%EC%9D%B4%EC%A0%9C%EB%8A%94-%EA%B0%9C%EB%B0%9C%EC%9E%90%EB%8F%84-CPU-%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98%EB%A5%BC-%EA%B5%AC%EB%B6%84%ED%95%B4%EC%95%BC-%ED%95%A9%EB%8B%88%EB%8B%A4)   

|아키텍쳐| ros image | 기타 |
|:---|:---:| ---:|
| arm64 | [ros_foxy image](https://hub.docker.com/r/arm64v8/ros/) |ros 공식 지원 image, (arm64v8)|
| x86_64 | [ros_foxy image](https://hub.docker.com/_/ros) |ros 공식 지원 image, (ros)|
   
   
---



## 하드웨어
이 프로젝트에서 사용된 주요 하드웨어 부품 목록입니다.


**1. 구동 모터 및 제어 시스템**

|부품 | 품명 | 설명 |
|:--- |:---:|:---|
| BLDC 모터 | MDR200 Series | 로봇의 주요 구동 장치로, 각 바퀴에 독립적으로 장착하여 좁은 공간에서도 자유로운 이동이 가능 |
| 모터 드라이버 | MD500T | BLDC 모터의 속도 및 방향 제어를 담당하며, 고성능의 구동력을 제공 |
| 구동부 제어보드 | NUCLEO-F412ZG | Micro-ROS 구동 및 모터 제어 역할 |
| 센서부 제어보드 | Arduino DUE | RTOS 구동 및 광전 센서 스위치와 범퍼 스위치 신호 입력과 리프트 제어 |
| 리프트 | - | 구동 전압 24V, 대차의 작업자 편의를 위한 리프트, H-Bridge 활용 |

**2. 센서 시스템**

|부품 | 품명 | 설명 |
|:---|:---:|:---|
| 2D LiDAR | YDLIDAR G2 | 로봇의 주변 장애물 인식 및 경로 계획을 위한 핵심 센서로, 빠른 반응 시간과 비용 효율성을 갖춤 |
| 광전 센서 | E3F-R2N1 | 근거리 장애물 감지 및 회피를 위해 사용. <br> 초음파 센서는 리프트 높이 조절에 활용되며 적외선 센서는 경로 수정 및 근거리 장애물 감지를 지원 |
| IMU 센서 | MW-AHRS.v2 | 로봇의 자세 및 가속도를 실시간으로 측정하여 자율 주행에 필요한 데이터 피드백을 제공 |
| 범퍼스위치 | NKB-NUB720 | 범퍼 형상으로 외부에서 충격이 가해지면 토글되는 원리의 스위치 <br> 하단 장애물에 대응 |
| 초음파센서 | HC-SR04 | 거리를 피드백 받아 리프트 제어 |

**3. 전원 및 배터리 시스템**

|부품 | 품명 | 설명 |
|:---|:---:|:---|
| 배터리 | TABOS DH-375 <br> 24V 50Ah | 로봇이 약 6시간 연속 작동할 수 있도록 하는 고용량 배터리. <br> 과전류 및 과열 보호 기능을 갖추어 시스템의 안전성을 보장 |

---

## 맡은 업무
이 프로젝트에서 제가 맡은 업무입니다.

0. Project Leader
  - 프로젝트 일정 관리
  - 프로젝트 주제 선정
  - 공문서 제작 및 자재 주문 관리
  - 팀원 메탈 케어

1. H/W 설계 및 URDF 제작
  - 시뮬레이션 내에서 적용할 물리 변수를 포함한 3D 모델 yaml파일 제작
  - 인벤터, 블랜더 활용

![image](https://github.com/user-attachments/assets/ad747644-300e-44e9-8470-79676c6da30d)

2. 전장 작업
  - 릴레이 스위치를 활용한 안전한 배터리 활용 기능 설계
  - 전력 계산 및 회로 구성

![image](https://github.com/user-attachments/assets/f1cae6b3-2fe2-438e-980d-20ea26116f92)

3. 센서 인터페이스 및 리프트 제어
  - 장애물에 대한 응답이기에 실시간성 중요하다 판단
  - RTOS를 활용한 Task로 최대한 실시간 반응 구현
  - H - Bridge 회로와 초음파 센서를 통한 거리 피드백으로 작업 편의성 증대

![image](https://github.com/user-attachments/assets/997ebd93-cd6b-4629-b7fd-1d3295c5a6c1)
