# capstone-design

#### 팀명 : ncslab

#### 기간 : 2022. 03 ~ 2022. 11

#### 팀원 : 고신엽, 이승헌, 장석천

# Harware  
- Board  
  - Jetson nano, Rasberry Pi  
 
- Wheel  
  - Dynamixel xc430  

- Lidar  
  - RP Lidar A1  
![image](https://user-images.githubusercontent.com/94602114/200448379-fb0d22c4-9538-4bf0-a420-cf2caf58e01e.png)

# Service robot Block Diagram  
![image](https://user-images.githubusercontent.com/94602114/200448002-ce8384bd-69d4-4e37-a437-80a4e45a1acf.png)

- SLAM Node(Simultaneous localization and mapping) 
  - 자율주행 차량에 사용되어 동시에 위치를 추정하고 주변 환경에 대한 지도를 작성하는 기술입니다.  
  

https://user-images.githubusercontent.com/94602114/200451320-b7a866bc-86a5-4843-85f3-7d02b073f2a1.mp4



- AMCL Node(Adaptive Monte Carlo Localization)  
  - lidar data를 가지고 특징점을 찾아 자신의 위치를 추정하는 알고리즘  
  - 초기  
![스크린샷, 2022-06-23 23-19-08](https://user-images.githubusercontent.com/94602114/175321758-4bb87b78-14c9-4028-9af3-bc452ef81cc8.png)
  - amcl 알고리즘 후  
![스크린샷, 2022-06-23 23-27-06](https://user-images.githubusercontent.com/94602114/175323423-8b2a0eca-ac7f-465e-9346-e68d694dab4e.png)

- MoveBase Node
  - 목표값(goal)을 주면 로봇이 갈 길을 찾고 모터제어명령을 내리는 알고리즘
  - Global path plan & local plan  
![image](https://user-images.githubusercontent.com/94602114/175328295-d821cbb6-3c05-4afa-8392-278c6792a0a8.png)

 - 구동 영상  
 https://youtu.be/1NEKSqXpm4c
 
 - 디지털 정보관 1층 slam 및 navigation 
 ![1658298974067](https://user-images.githubusercontent.com/94602114/188551885-1a6e7ad6-e63c-4af4-a3b3-dfa204b54982.jpg)

 https://youtu.be/Y2KB4LlKs5s
 
# Multi-robot-simulation  
## transform tree  
 ![b9303efd99f4e86728f09dc9e490023db08b5875](https://user-images.githubusercontent.com/94602114/188367818-a5e267f5-42c4-4006-93b9-6f6a95562042.png)

## rqt_graph
![rosgraph](https://user-images.githubusercontent.com/94602114/188405626-ae9ff68e-28f5-4226-83a6-23afe1c6b5c7.png)

## 구동영상  
https://youtu.be/6zpc89aQP3U
