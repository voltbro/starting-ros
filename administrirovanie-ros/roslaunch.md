# Roslaunch, управление запуском

Раньше мы использовали для запуска утилиту `rosrun`, которая запускает конкретный исполняемый файл.

В реальных системах, одновременно должны работать множество программ. Для их запуска и конфигурации служит утилита `roslaunch`

Используя `roslaunch` возможно дополнительно настраивать исполняемые файлы в момент их запуска \(передавать параметры, изменять имена и тп\)

`roslaunch` использует файлы с расширением .launch, которые представляет собой обычный XML файл.

Давайте создадим файл `./launch/demo.launch`

```markup
<launch>
  <node pkg="test_package" type="topic_publisher.py" name="topic_publisher1"/>
  <node pkg="test_package" type="topic_subscriber" name="topic_subscriber1"/>
  <node pkg="test_package" type="topic_publisher.py" name="topic_publisher2"/>
  <node pkg="test_package" type="topic_subscriber" name="topic_subscriber2"/>
</launch>
```

Теги, необходимые для запуска узла с помощью команды `roslaunch`, описаны в теге `launch`. Тег `node` описывает ноды, который которые необходимо запускать с помощью `roslaunch`. Параметры включают «pkg», «type» и «name».

| Параметр | Описание |
| :--- | :--- |
| pkg | Имя пакета |
| type | Название ноды, которая будет выполняться |
| name | Имя \(исполняемое имя\), используемое при выполнении ноды, соответствующего выше параметру type. Обычно это имя совпадает, но при его запуске можно использовать другое имя. |

После сохранения файла, мы можем его запустить, выполнив

```text
$ roslaunch test_package demo.launch --screen

SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.13

NODES
  /
    topic_publisher1 (test_package/topic_publisher.py)
    topic_publisher2 (test_package/topic_publisher.py)
    topic_subscriber1 (test_package/topic_subscriber)
    topic_subscriber2 (test_package/topic_subscriber)

ROS_MASTER_URI=http://localhost:11311

process[topic_publisher1-1]: started with pid [24963]
process[topic_subscriber1-2]: started with pid [24964]
process[topic_publisher2-3]: started with pid [24965]
process[topic_subscriber2-4]: started with pid [24978]
```

Если добавить параметр `--screen`, информация о работе запущенных программ будет отображенна на экране текущего терминала.

Или указав полный путь к .launch файлу

```text
$ roslaunch /home/user/catkin_ws/src/test_package/launch/demo.launch --screen
```

Как мы видим, запущенно 4 процесса с разными `pid.` Также мы можем увидеть список запущенных нод

```bash
$ rosnode list

/rosout
/topic_publisher1
/topic_publisher2
/topic_subscriber1
/topic_subscriber2
```

## Обьединение процессов в группы

Если мы посмотрим на список топиков

```bash
$ rostopic list

/hello

/rosout
/rosout_agg
```

То поймем, что оба Издателя \(Publisher\) публикуют данные в один топик `/hello` \(других топиков не создана\) и каждый из подписчиков получает сообщения сразу от двух Издателей. Скорее всего, такой режим работы нам не интересен.

Если мы хотим чтобы один конкретный Издатель и Подписчик были изолированны от аналогичных процессов, мы можем обьеденить их в группы.

Создадим новый файл запуска `./launch/demo1.launch`

```markup
<launch>
  <group ns="ns1">
    <node pkg="test_package" type="topic_publisher.py" name="topic_publisher"/>
    <node pkg="test_package" type="topic_subscriber" name="topic_subscriber"/>
  </group>
  <group ns="ns2">
    <node pkg="test_package" type="topic_publisher.py" name="topic_publisher"/>
    <node pkg="test_package" type="topic_subscriber" name="topic_subscriber"/>
  </group>
</launch>
```

Запустим новый файл

```text
$ roslaunch test_package demo1.launch
```

```text
$ rostopic list
/ns1/hello

/ns2/hello

/rosout

/rosout_agg
```

Мы видим, что создано два отдельных топика, с которым работает один Издатель и один Подписчик.

```bash
$ rostopic info /ns1/hello


Type: std_msgs/String

Publishers:
 * /ns1/topic_publisher (http://cola:44225/)


Subscribers:
 * /ns1/topic_subscriber (http://cola:44043/)



$ rostopic info /ns2/hello

Type: std_msgs/String


Publishers:
 * /ns2/topic_publisher (http://cola:45259/)


Subscribers:
 * /ns2/topic_subscriber (http://cola:35017/)
```

## Установка переменных окружения при запуске

Очень часто, возникает необходимость конфигурировать исполняемые файлы в момент их запуска. Например нужно иметь возможность изменять скорость `Serial` порта и его адрес. Каждый раз менять эти переменные в коде не удобно и очень долго.

Чтобы решить данную проблему, мы можем использовать возможность передавать конфигурационные переменные в запускаемые программы через `.launch` файлы

Рассмотрим пример

```markup
<launch>
    <node pkg="test_package" type="test_params.py" name="test_params" output="log" respawn="true">
        <param name="port" value="/dev/ttyS0"/>
        <param name="boud" value="57600"/>
    </node>
</launch>
```

Мы видим что в ветке `node` добавились элементы `param` с настройками. Открывая такой файл, сразу видно, какие параметры возможно конфигурировать и их значения по умолчанию.

Для того, чтобы исполняемый файлы смогли обрабатывать эти параметры, необходимо добавить для них специальный код. Это не сложно, и для `python` может выглядеть так

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

rospy.init_node('test_params')
r = rospy.Rate(10) # 10hz
port = rospy.get_param('~port','/dev/ttyS0')

while not rospy.is_shutdown():
    print(port)
    r.sleep()
```

Вторым параметром в функции `get_param` указывается значение по умолчанию, если параметр не определен в `.launch` файле.

Параметры возможно передавать при запуске через `rosrun`

```text
rosrun test_package test_params.py _port:=/dev/ttyS1
```

Еще одна удобная практика, для работы с параметрами, это перенос настроек в начала файла в перечислив их как аргументы. Это можно сделать использую элемент `arg` а далее обращение к этим аргумента в формате `$(arg env_name)`

```markup
<launch>
    <arg name="device" default="/dev/ttyS0"/>
    <arg name="boud" default="57600"/>

    <node pkg="test_package" type="test_params.py" name="test_package" output="log" respawn="true">
        <param name="port" value="$(arg device)"/>
        <param name="boud" value="$(arg boud)"/>
    </node>
</launch>
```

Эта практика позволяет не листать большие файлы и всегда иметь перед глазами самые важные настройки.

## Подключение других .launch файлов &lt;include&gt;

В реальных проектах, запускаются десятки нод. Конфигурировать каждую из них в одном файле, не всегда удобно. К тому-же обычно сторонние пакеты уже содержат подходящие `.launch` файлы. Поэтому существуют механизм `include`, который позволяет подключать другие файлы запуска.

Приведем пример

```markup
<launch>
 <include file="$(find test_package)/launch/test_params.launch">
     <arg name="device" value="/dev/ttyS1"/>
  </include>
  <include file="$(find navibro)/camera/camerav1_640x480.launch"/>
  <include file="$(find navibro)/launch/aruco_detect.launch"/>
  <include file="$(find navibro)/launch/fiducial_slam.launch"/>
</launch>
```

В этом примере, мы подключаем файл `test_params.launch` , который находиться в нашем пакете и настраиваем его на работу через устройство `/dev/ttyS1.`. А также подключаем три других `.launch` файла из другого пакета.

## Использование условий if и unless

При написании сложный .launch файлов, очень помагают атрибуты `if` и `unless`, которые позволяют формировать простые алгоритмы ветвления при работе с `roslaunch`

Приведем несколько примеров

```markup
<launch>
   <arg name="have_serial" value="true"/>

   <group if="$(arg have_serial)">
     <!-- Блок выполниться только если have_serial установлено в true -->
     <node pkg="test_package" type="test_params.py" name="test_package" output="log" respawn="true">
  </group>
  <!-- Также if можно использовать для одного тега-->
  <include if="$(arg have_serial)" file="$(find test_package)/launch/test_params.launch">
    <arg name="device" value="/dev/ttyS1"/>
  </include>
</launch>
```

Атрибут `unless` работает противоположно атрибуту `if`. Если значение `0` то блок выполняется.

Значение атрибутов для `if` и `unless` должно быть булевым \(`0,1,true,false`\)

