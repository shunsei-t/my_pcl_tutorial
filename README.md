# Intelisense setting
c_cpp_properties.json
```
{
    "configurations": [
        {
            "browse": {
                "databaseFilename": "",
                "limitSymbolsToIncludedHeaders": true
            },
            "includePath": [
                "/mnt/d/catkin_ws/devel/include/**",
                "/opt/ros/noetic/include/**",
                "/usr/include/**"
            ],
            "name": "ROS",
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17"
        }
    ],
    "version": 4
}

```
# build

```
catkin build -DCMAKE_BUILD_TYPE=Release my_pcl_tutorial
```

```
catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -flto -fomit-frame-pointer -funroll-loops" my_pcl_tutorial
```

# What is this?
Point Cloud LibraryのチュートリアルをROSで実装していったもの。
# Filters
## Passthrough filter
```
rosrun my_pcl_tutorial passthrough_filter
roslaunch my_pcl_tutorial passthrough_filter.launch
```
### Subscribed Topics
- output(sensor_msgs/PointCloud2)
### Publish Topics
- output(sensor_msgs/PointCloud2)
### parameters
- field_name
フィルタリングする軸（x, y, z）
- upper_limit
上限 [m]
- lower_limit
下限 [m]