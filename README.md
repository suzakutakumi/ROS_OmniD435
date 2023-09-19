# ROS_OmniD435

## 実行方法

現状、カメラのIDやPCDファイルの保存する場所は`pub.cpp`や`sub.cpp`内の変数に直接書いてあります。  
ご自身の環境で実行する場合は、それらを編集してください。  

変数を置換後、以下のコマンドを実行すればビルドと実行ができます。
```bash
cd ROS_OmniD435
catkin build
source ./devel/setup.bash
roslaunch omni_d435 omni-d435.launch
```
