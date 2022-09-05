# capstone-design  
### Service robot Block Diagram  
![image](https://user-images.githubusercontent.com/94602114/175318217-ec607ce4-3d4f-4d13-819b-c79d845eed28.png)

### navigation node Block Diagram  
![image](https://user-images.githubusercontent.com/94602114/175333138-0412e1da-08a6-404e-8b5b-cf6c49e9353e.png)  
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
 
 - 디지털 정보관 1층 slam 및 navigation  
 https://youtu.be/Y2KB4LlKs5s
 
 # Multi-robot-simulation  
 ## transform tree  
 ![b9303efd99f4e86728f09dc9e490023db08b5875](https://user-images.githubusercontent.com/94602114/188367818-a5e267f5-42c4-4006-93b9-6f6a95562042.png)

## rqt_graph
![rosgraph](https://user-images.githubusercontent.com/94602114/188405626-ae9ff68e-28f5-4226-83a6-23afe1c6b5c7.png)
