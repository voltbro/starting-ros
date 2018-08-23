# Roslaunch, управление запуском

Раньше мы использовали для запуска утилиту `rosrun`, которая запускает конкретный исполняемый файл. 

В реальных системах, одновременно должны работать множество программ. Для их запуска и конфигурации служит утилита `roslaunch`

Используя `roslaunch` возможно дополнительно настраивать исполняемые файлы в момент их запуска \(передавать параметры, изменять имена и тп\)

`roslaunch` использует файлы с расширением .launch, которые представляет собой XML файл.

Давайте создадим простой файл `./launch/demo.launch`

```markup
<launch>
  <node pkg="test_package" type="topic_publisher.py" name="topic_publisher1"/>
  <node pkg="test_package" type="topic_subscriber" name="topic_subscriber1"/>
  <node pkg="test_package" type="topic_publisher.py" name="topic_publisher2"/>
  <node pkg="test_package" type="topic_subscriber" name="topic_subscriber2"/>
</launch>
```

Теги, необходимые для запуска узла с помощью команды `roslaunch`, описаны в теге `launch`. Тег `node` описывает ноды, который которые необходимо запускать с помощью 'roslaunch'. Параметры включают «pkg», «type» и «name».

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

Если добавить параметр «--screen», информация о работе запущенных программ будет отображенна на экране текущего терминала.

Как мы видим, запущенно 4 процесса с разными `pid.` Также мы можем увидеть список запущенных нод

```bash
$ rosnode list

/rosout
/topic_publisher1
/topic_publisher2
/topic_subscriber1
/topic_subscriber2
```

### Обьединение процессов в группы

Если мы посмотрим на список топиков

```bash
$ rostopic list

/hello
/rosout
/rosout_agg
```

То поймем, что оба Издателя \(Publisher\) публикуют данные в один топик `/hello` \(других топиков не созданно\) и каждый из подписчиков получает сообщения сразу от двух Издателей. Скорее всего, такой режим работы нам не интересен.

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

Мы видим, что созданно два отдельных топика, с которым работает один Издатель и один Подписчик.

```bash
$ rostopic info /ns1/hello

Type: std_msgs/String
Publishers:  * /ns1/topic_publisher (http://cola:44225/)
Subscribers:  * /ns1/topic_subscriber (http://cola:44043/)

$ rostopic info /ns2/hello

Type: std_msgs/String
Publishers:  * /ns2/topic_publisher (http://cola:45259/)
Subscribers:  * /ns2/topic_subscriber (http://cola:35017/)
```

### Установка переменных окружения



