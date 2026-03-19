# Installation

本ページの手順は[Autoware Universe source installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)に従ってAutowareがインストールされていることを前提とします。

1. 以下の内容を`repositories/autoware.repos`に追記します。

   ```txt
     universe/external/tier4_ad_api_adaptor:
       type: git
       url: https://github.com/tier4/tier4_ad_api_adaptor.git
       version: tier4/universe
   ```

2. [How to update a workspace](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#how-to-update-a-workspace)に従ってリポジトリを更新します。

   ```bash
   vcs import src < repositories/autoware.repos
   vcs pull src

   sudo apt update && sudo apt upgrade
   source /opt/ros/humble/setup.bash
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. Autowareを起動時、別の端末から以下のコマンドを実行して追加のAPIを起動します。

   ```bash
   source install/setup.bash
   ros2 launch tier4_autoware_api_extension_launch tier4_autoware_api_extension.launch.xml api_mode:=1
   ```
