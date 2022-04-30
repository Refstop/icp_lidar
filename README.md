# icp_lidar
point to point icp 알고리즘을 구현한 C++ 패키지입니다. [링크](https://github.com/richardos/icp)의 파이썬 예제를 참고하여 작성했습니다.

## Dependencies
- [Eigen Library](https://eigen.tuxfamily.org/index.php?title=Main_Page)  
- [Matplotlib C++](https://github.com/lava/matplotlib-cpp)  
- [knncpp kd tree Library](https://github.com/Rookfighter/knn-cpp)  
위의 3가지 라이브러리를 `make install` 한 후에 컴파일 및 실행이 가능합니다.

## 예제 실행 방법
```
cd icp_lidar
mkdir build
cd build && cmake ..
make
./icp_lidar ../example/reference_points.txt ../example/points_to_be_aligned.txt
```
build 폴더를 생성하여 make 한 후 실행 파일을 실행합니다. 실행 파일의 파라미터는 각각 기준 포인트들의 파일명, icp 알고리즘에 의해 정렬될 포인트들(input points)의 파일명입니다. 두 포인트 예제 파일은 [링크](https://github.com/richardos/icp)의 파이썬 예제에 의해 생성된 랜덤 포인트를 저장한 것입니다.

## 예제 실행 결과
파란색 점: 기준 points  
파란색 *: input points  
빨간색 *: icp 알고리즘에 의해 정렬된 points  
- reference_points.txt와 points_to_be_aligned.txt의 정렬 결과
<p align="center"><img src="/figs/result_1.png">scan1 data</p>

- true_data.txt와 moved_data.txt의 정렬 결과
<p align="center"><img src="/figs/result_2.png">scan1 data</p>