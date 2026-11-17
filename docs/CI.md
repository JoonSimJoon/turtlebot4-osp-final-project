# GitHub Actions 검증

`.github/workflows/ci.yml`은 `main` 푸시, Pull Request, 수동 실행 시 Ubuntu 24.04에서 다음을 수행합니다.

1. 실제 배포용 `./start.sh setup`으로 Jazzy·TurtleBot4·Nav2·SLAM·OpenCV 의존성을 설치하고 패키지를 빌드합니다.
2. 설치된 프로젝트 실행 파일과 프로젝트/공식 TurtleBot4 launch 인수를 확인합니다.
3. 순수 로직 및 실제 ROS 통신을 사용하는 테스트를 모두 실행합니다. ROS 통합 테스트는 `TB4_RUN_ROS_TESTS=1`로 활성화하며, 생략된 테스트가 있으면 CI도 실패합니다.
4. 테스트 JUnit과 launch 인수 기록을 `ubuntu-jazzy-verification` 아티팩트에 14일 동안 보관합니다.

테스트는 localhost DDS와 전용 도메인으로 실행합니다. 센서 입력과 Nav2 액션 서버는 테스트용 대역이며 실제 로봇에 연결하지 않습니다. 이 검증이 성공해도 실제 SLAM 지도 품질·Nav2 주행 성능·현장 카메라 보정이 검증된 것은 아닙니다.

워크플로에는 저장소 읽기 권한만 부여하며 별도 secret은 필요하지 않습니다. 공식 [checkout](https://github.com/actions/checkout) 및 [upload-artifact](https://github.com/actions/upload-artifact)의 확인된 v7 커밋에 고정했습니다. apt 패키지는 실행 시 공식 Jazzy 저장소의 배포 버전을 사용하므로 패키지 변경으로 CI 결과가 달라질 수 있습니다.

첫 실제 GitHub 실행 결과는 저장소의 **Actions → ROS Jazzy CI**에서 확인합니다. 이 문서는 CI 구성 설명이며 실행 성공을 미리 의미하지 않습니다.
