# Создание и запуск первой программы

Напишем простую программу, публикующую топик, и запустим ее.

#### Для python

Перейдем в директорию проекта и создадим файл `./src/topic_publisher.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('hello', String, queue_size=10)
rospy.init_node('hello_topic_publisher')
rospy.loginfo("I`am first step PUB node")
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    pub.publish("Hello World")
    r.sleep()
```

При использовании `python` у нас есть возможность запустить выполнение программы несколькими способами.

Непосредственно вызов `python` скрипта

```bash
$ python src/topic_publisher.py

[INFO] [1534941383.313599]: I`am PUB node
```

Изменение атрибутов файла и запуск исполняемого файлы

```bash
$ chmod a+x src/topic_publisher.py
$ ./src/topic_publisher.py

[INFO] [1534941755.372466]: I`am PUB node
```

Использование консольной утилиты `rosrun`

```bash
$ chmod a+x src/topic_publisher.py
$ rosrun test_package topic_publisher.py

[INFO] [1534941755.372466]: I`am PUB node
```

При использовании  `rosrun` утилита самостоятельно найдет директорию проекта и исполняемый файл, указывать абсолютный путь к файлу не обязательно.

Для остановки программы \(выхода из программы\) необходимо нажать Ctrl-C \(Shift-Ctrl-C\)

#### Для Си

Сначала, в директории `src` создадим файл с исходным кодом клиента для подписки на топик `./src/topic_subscriber.cpp` .

```cpp
// ROS Default Header File
#include "ros/ros.h"
// Message File Header
#include "std_msgs/String.h"

void msgCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO("recieve msg = %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_subscriber_node");
  ROS_INFO("I`am SUB node");
  ros::NodeHandle nh;
  ros::Subscriber ros_tutorial_sub = nh.subscribe("hello", 100, msgCallback);
  ros::spin();
  return 0;
}

```

Для того, чтобы исходный код на `C` превратился в испольняемую программу, его необходимо скомпилировать.

Для этого необходимо внести изменения в файл `CMakeLists.txt`, для того чтобы сборщик `catkin_make` правильно его скомпилировал.

```text
## Has dependency on message_generation, std_msgs, roscpp.
## An error occurs during the build if these packages do not exist.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)

## A Catkin package option that describes the library, the Catkin build dependencies,
## and the system dependent packages.
catkin_package(
   LIBRARIES test_package
   CATKIN_DEPENDS std_msgs roscpp
)
## Include directory configuration.
include_directories(${catkin_INCLUDE_DIRS})

## Build option for the "topic_subscriber" node.
add_executable(topic_subscriber src/topic_subscriber.cpp)
add_dependencies(topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_subscriber ${catkin_LIBRARIES})
```

После изменения файла `CMakeLists.txt` необходимо произвести сборку проекта командой `catkin_make`

```bash
$ cd ~/catkin_ws
$ catkin_make

.....
[100%] Built target topic_subscriber
```

Для запуска программы, необходомо воспользоваться утилитой `rosrun`

```bash
$ rosrun test_package topic_subscriber

[ INFO] [1534958887.906387605]: I`am SUB node
```

Для остановки программы \(выхода из программы\) необходимо нажать Ctrl-C \(Shift-Ctrl-C\)

Если одновременно запустить Издателя \(Publisher\) и Подписчика \(Subscriber\) то в терминале Подписчика, мы увидим вывод

```bash
[ INFO] [1534958909.319854126]: recieve msg = Hello World
[ INFO] [1534958909.420175931]: recieve msg = Hello World
[ INFO] [1534958909.520166483]: recieve msg = Hello World
[ INFO] [1534958909.619940238]: recieve msg = Hello World
[ INFO] [1534958909.720158742]: recieve msg = Hello World
[ INFO] [1534958909.819872433]: recieve msg = Hello World
[ INFO] [1534958909.920228940]: recieve msg = Hello World
[ INFO] [1534958910.020099319]: recieve msg = Hello World
[ INFO] [1534958910.120193706]: recieve msg = Hello World
[ INFO] [1534958910.220381848]: recieve msg = Hello World
[ INFO] [1534958910.320247543]: recieve msg = Hello World
[ INFO] [1534958910.420066932]: recieve msg = Hello World
```

В этой главе мы написали и запустили простые программы на разных языках, которые обмениваються данными.

