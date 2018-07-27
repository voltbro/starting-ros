# Работа с Topic

Модель работы в режиме Topic, подразумевает использование одного типа сообщения как для Издателя \(Publisher\) и подписчика \(Subscriber\).

Модель Topic являются однонаправленной и подразумевает непрерывную отправку или получение сообщений. Такой способ коммуникации подходит для датчиков, которым требуются периодическая публикация сообщений. Несколько подписчиков могут получать сообщения от одного издателя и наоборот. Также возможна работа несколько издателей.

На изображении ниже показана модель работы датчика температуры, когда его данные получают различные ноды.

![](../.gitbook/assets/ros_topic.png)

### Примеры работы на Python

Для работы с топиками мы будем использовать библиотеку **rospy.Publisher Code API** [http://docs.ros.org/api/rospy/html/rospy.topics.Publisher-class.html](http://docs.ros.org/api/rospy/html/rospy.topics.Publisher-class.html)

Вы можете создать обработчик для публикации сообщения в Топик с помощью класса `rospy.Publisher`. После инициализации, вы можете пубдликовать сообщения, например:

```python
pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
pub.publish(std_msgs.msg.String("foo"))
```

В данном примере:

* `topic_name`Название топика для публикации сообщений
* `std_msgs.msg.String` Тип передаваемого сообщения
* `"foo"`переданное сообщение

Полный пример кода:

```python
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
   pub.publish("hello world")
   r.sleep()
```

Для приема сообщениями необходимо воспользоваться **rospy.Subscriber Code API** [http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html](http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html)

Пример реализации подписчика

```python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s",data.data)
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
```

Более подробную информацию можно посмотреть на Wiki странице ROS

