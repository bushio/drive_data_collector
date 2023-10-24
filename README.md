# drive_data_collector
本リポジトリはAWSIMと連携してデータを取得するためのツールです。

# Overview
本ツールではROSからセンサデータを受取り、保存・変換・再送することが目的です。

# Extract raw data from rosbag
velodyne からのデータを取得して保存する方法を記載します。
## Install
下記のコマンドで velodyne ツールをダウンロードしてください。

```
git clone https://github.com/ros-drivers/velodyne
cd velodyne/
git checkout ros2
``` 
Learn more about the velodyne tool [here](https://github.com/ros-drivers/velodyne).
Learn more about ros2_numpy tool [here](https://github.com/Box-Robotics/ros2_numpy).

## overview
"lidar_data_collector.py"　では下記のステップで velodyne パケットを受取り、.npy 形式で保存します。。

1. velodyne パケットを受信する。
2. velodyne ツールに受信したパケットを再送し、point_cloud2 データに変換する。
3. velodyne ツールが変換した point_cloud2 データを受信する。
4. 受信した point_cloud2 データ .npy 形式で保存する。

 velodyne tolls, 
## Run
実行方法を記載します。
### Terminal 1
下記コマンドで Velodyneのパケットが格納された rosbagをリプレイします。
```
ros2 bag play {/path/to/your/rosbag} -r 0.2 -s sqlite3
```

### Terminal 2 
下記コマンドで "lidar_data_collector.py" を起動します。

```
ros2 launch drive_data_collector lidar_data_collector.launch.xml
```

## Result
実行後、下記の手順で rviz を立ち上げることで、変換された point_cloud2　データを確認できます。
1. 下記コマンドで rviz を起動する。
```
rviz2
```
2. rviz 画面左の "Global Options" の Fixed Frame を "base_link" に設定する。
3. 画面左下の "Add" から "PointCloud2" を選択し、追加する。
4. "PointCloud2" タグの"Topic" に　"/velodyne_points" を設定する。



## Convert .npy into point_cloud2 and save them as .png .
### Run
下記コマンドで保存した .npy ファイルをロードし、送信することができます。
```
ros2 run drive_data_collector point_cloud_publisher_node --ros-args -p dirname:={/path/to/your/npy/dir}
```

