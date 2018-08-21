# Создание пакета

Любой написанный и запущенный код в рамках ROS принадлежить конкретному пакету.

Команды ниже создадут тестовый пакет `test_package` При этом мы укажем что пакет зависит от других пакетов \(std\_msgs rospy roscpp actionlib\_msgs message\_generation\). И наш пакет уже будет настроен для работы с этими зависимостями. Далее для добавления зависимостей, нам будет необходимо редактировать файл `package.xml`

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg test_package std_msgs rospy roscpp actionlib_msgs message_generation message_runtime
```

После создания пакета, мы увидим директорию `test_package` в которой будет находиться файлы пакета.

```bash
$ cd test_package 
$ ls

include        → Header File Folder
src            → Source Code Folder
CMakeLists.txt → Build Configuration File
package.xml    → Package Configuration File
```

Файл `package.xml` содержит информацию о пакете: название, автора, лицензию, список зависимостей от других пакетов ROS.

```markup
<?xml version="1.0"?>
<package format="2">
  <name>test_package</name>
  <version>0.0.0</version>
  <description>The test_package package</description>

  <maintainer email="cola@todo.todo">cola</maintainer>
  <license>TODO</license>
  <!-- <url type="website">http://wiki.ros.org/test_package</url> -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

Второй обязательный файл для пакета это файл `CMakeLists.txt` Этот файл содержит инструкции для `Catkin` \(системы сборки пакетов, которая использует Сmake\). В этом файле содержатся инструкции на создание исполнительных файлов, очередность сборки проекта, создание символьних ссылок и тп. Откройте и посмотрите на содержание файла.

### 

