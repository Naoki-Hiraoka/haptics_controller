## 環境構築
### rosinstall
```bash
catkin_ws/src$ emacs -nw .rosinstall # auto_stabilizerのディレクトリにある.rosinstall と auto_stabilizer_choreonoid_sampleのディレクトリにある.rosinstallの内容を記入する. このリポジトリが入っていないので追加する.
catkin_ws/src$ wstool update
```
### ビルド
```bash
catkin_ws/src$ cd ..
# 依存ライブラリをインストールする.
catkin_ws$ rosdep install -r --from-paths src --ignore-src -y
# 参考 https://github.com/start-jsk/rtmros_hironx/issues/539
catkin_ws$ echo "export ORBgiopMaxMsgSize=2097152000" >> ~/.bashrc
catkin_ws$ source ~/.bashrc
# for choreonoid. 参考 https://github.com/start-jsk/rtmros_choreonoid
# choreonoidはrosdepに対応しておらず，代わりに依存ライブラリをaptでインストールするスクリプトがあるのでそれを実行
catkin_ws$ ./src/choreonoid/misc/script/install-requisites-ubuntu-18.04.sh # if ubuntu 18.04
# choreonoidのコンパイルオプションを設定
catkin_ws$ patch -p1 -d src/choreonoid < src/rtm-ros-robotics/rtmros_choreonoid/choreonoid.patch
```
* ビルド
```bash
catkin_ws$ source /opt/ros/melodic/setup.bash
catkin_ws$ catkin build haptics_controller_choreonoid_sample # 失敗してもそのままもう一回同じビルドを行うと通ることがある
catkin_ws$ source devel/setup.bash
```

## 実行

```shell
rtmlaunch haptics_controller_choreonoid_sample tablis_choreonoid.launch
```
