# capstone-design  
### Service robot Block Diagram  
![image](https://user-images.githubusercontent.com/94602114/175318217-ec607ce4-3d4f-4d13-819b-c79d845eed28.png)

### navigation node Block Diagram  
![캡처](https://user-images.githubusercontent.com/94602114/175317866-de436de3-97ef-48f4-bdc5-c7f0723f87fb.PNG)
- AMCL Node  
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
 
 # 문제점  
 deadreckoning을 이용하여 x좌표 y좌표, theta를 구하는데  
 직진만 하였을 때는 오차가 적지만  
 회전을 하였을 때 오차가 누적되는 현상을 발견하였음  
 
 그리하여 IMU의 raw data를 filtering하여 theta값(YAW)을 추정할 예정
 

https://user-images.githubusercontent.com/94602114/175331810-7e71c63e-b27f-4f9e-95c0-57893c6c037d.mp4

그러나 imu데이터에 drift가 살짝 있음 -> 우선 실험을 해볼 예정
