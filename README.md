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