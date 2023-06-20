# teamProject_programmers

## 1. 차선 인식 프로젝트
 - xycar (monocular usb) camera 센서만을 사용하여 차선인식 주행
 - OpenCV (Gaussian_Blur, Canny_Edge, Hough_Transform ...)활용
 - PID 제어 (Stanley 적용시켜 봤으나 실패 - 카메라만 사용해서 구현하기 힘듦)
 - 노드 통신 개략도
   
   <img src="https://github.com/Hwan-Yeong/teamProject_programmers/assets/130347326/4357f35e-ee6e-4765-84cb-9e0d390a9df0" width="500" height="300">


## 2. 장애물 회피 프로젝트 (미로탈출)
 - xycar Lidar, Ultrasonic, Camera 센서를 사용하여 장애물 회피 주행
 - hector_slam 시도했으나 실패 (planning 못함 - 안배운 걸 4일만에 구현하기 힘듦)
 - 장애물 회피하는 규칙기반 알고리즘 설계
 - Steering Angle 구하는 식
   
   <img src="https://github.com/Hwan-Yeong/teamProject_programmers/assets/130347326/ea60a6cf-e6b5-4ae3-b8d8-9f60520db9c2" width="500" height="300">
 - Camera&OpenCV 사용하여 주차구역 인식하는 미션 실패 (프로젝트 시간부족)

## 3. 
