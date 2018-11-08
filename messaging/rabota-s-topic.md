# Работа с Topic

Модель работы в режиме Topic, подразумевает использование одного типа сообщения как для Издателя \(Publisher\) и подписчика \(Subscriber\).

Модель Topic являются однонаправленной и подразумевает непрерывную отправку или получение сообщений. Такой способ коммуникации подходит для датчиков, которым требуются периодическая передача данных. Несколько подписчиков могут получать сообщения от одного издателя и наоборот \(возможна работа несколько издателей\).

На изображении ниже показана модель работы датчика температуры, когда его данные получают различные ноды.

![](../.gitbook/assets/ros_topic.png)

## Примеры работы на Python

Для работы с топиками мы будем использовать библиотеку **rospy.Publisher Code API** [http://docs.ros.org/api/rospy/html/rospy.topics.Publisher-class.html](http://docs.ros.org/api/rospy/html/rospy.topics.Publisher-class.html)

Вы можете создать обработчик для публикации сообщения в Топик с помощью класса `rospy.Publisher`. После инициализации, вы можете публиковать сообщения:

### Публикация сообщений

```python
pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
pub.publish(std_msgs.msg.String("foo"))
```

В данном примере:

* `topic_name`Название топика для публикации сообщений
* `std_msgs.msg.String` Тип передаваемого сообщения
* `"foo"` Переданное сообщение
* `"queue_size"` Длина очереди

Полный пример кода Издателя \(publisher\) `src/hello_topic_publisher.py` :

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('hello', String, queue_size=10)
rospy.init_node('hello_topic_publisher')
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    pub.publish("Hello World")
    r.sleep()
```

### Прием сообщений

Для приема сообщениями необходимо воспользоваться **rospy.Subscriber Code API** [http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html](http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html)

Пример реализации подписчика \(subscriber\) `src/hello_topic_subscriber.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s",data.data)

def subscriber():
    rospy.init_node('hello_topic_subscriber')
    rospy.Subscriber("hello", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

Более подробную информацию по работе с топиками на Python можно посмотреть на Wiki странице ROS [http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)

## Консольная утилита rostopic

`rostopic` это специальная консольная утилита, предназначенная для отображения отладочной информации о топиках в ROS. С ее помощью удобно искать нужные топики, и выводить сообщения в консоль для отладки.

Список основных используемых команд:

```text
rostopic bw     Показать занимаемый сетевой канал
rostopic echo   Вывести сообщения на экран
rostopic find   Поиск топика по типу
rostopic hz     Показать частоту обновления топика
rostopic info   Показать информацию о топике
rostopic list   Показать список существующий топиков
rostopic pub    Опубликовать данные в топик
rostopic type   Показать тип сообщения для топика
```

### Примеры использования

Вывести список существующий топиков

```text
rostopic list
```

Вывести сообщения из топика `topic_name`

```text
rostopic echo /topic_name
```

### rostopic pub <a id="rostopic_pub"></a>

Отправить текстовое сообщение в топик

```text
rostopic pub my_topic std_msgs/String "hello there"
```

Отправить сообщение типа `geometry_msgs/Twist`в топик /cmd\_vel с частотой 10hz

```text
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

Удобно при вызове функций, использовать `Tab` для поиска и подстановки необходимых данных в командную строчку.

Например

```text
rostopic pub /c+Tab -> rostopic pub /cmd_vel (подставиться адрес существующего топика)
rostopic pub /cmd_vel +Tab -> rostopic pub /cmd_vel geometry_msgs/Twist (подставиться тип сообщения выбранного топика)
```

Более подробная информация доступна на Wiki странице [http://wiki.ros.org/rostopic](http://wiki.ros.org/rostopic)

