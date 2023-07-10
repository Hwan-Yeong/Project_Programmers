# teamProject_programmers

## 1. 차선 인식 프로젝트
 - xycar (monocular usb) camera 센서만을 사용하여 차선인식 주행
 - OpenCV (Gaussian_Blur, Canny_Edge, Hough_Transform ...)활용
 - PID 제어 (Stanley 적용시켜 봤으나 실패 - 카메라만 사용해서 구현하기 힘듦)
 - 노드 통신 개략도
   
   <img src="https://github.com/Hwan-Yeong/Project_Programmers/assets/130347326/c4086b5a-d234-4dcb-91ff-e55a90102a5d" width="500" height="300">


## 2. 장애물 회피 프로젝트 (미로탈출)
 - xycar Lidar, Ultrasonic, Camera 센서를 사용하여 장애물 회피 주행
 - hector_slam 시도했으나 실패 (planning 못함 - 안배운 걸 4일만에 구현하기 힘듦)
 - 장애물 회피하는 규칙기반 알고리즘 설계
 - Steering Angle 구하는 식
   
   <img src="https://github.com/Hwan-Yeong/Project_Programmers/assets/130347326/1458f103-32be-4f5a-8447-bdd7a2f72652" width="500" height="300">
 - Camera&OpenCV 사용하여 주차구역 인식하는 미션 실패 (프로젝트 시간부족)

## 3. 시뮬레이션 기초편 활용 (Mid_Project)
 - Mint20 / gazebo11 / ros noetic
 - 로봇 모델 생성 (urdf, mesh, xacro, gazebo)
 - ROS plugin (robot differential driver, robot laser sensor, joint control)
 - World 만들기 (gazebo models, DEM(digital elevation models, population)
 - Plugin 개발 (world plugin, model plugin)

<img src="https://github.com/Hwan-Yeong/Project_Programmers/assets/130347326/86effe5e-07d7-4165-b216-89fbe5d8b3b7" width="650" height="500">

## 4. 서빙로봇 시뮬레이션 프로젝트 (Final_Project)
 - Mint20 / gazebo11 / ros noetic / turtlebot3
 - Mapping -> Localization -> Path Planning -> Control
 - gmapping / amcl / Navfn (global), DWA (local) Planner / move_base, state machine 등의 오픈소스 라이브러리 및 패키지 사용
 - waypoint를 따라 주행하는 python 코드 작성 (smach)
