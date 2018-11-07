# Работа с Service

Модель коммуникации в режиме Сервис представляет собой двунаправленную синхронную связь между клиентом \(Service Client\), создающим запрос, и сервером \(Service Server\), отвечающим на запрос.

![](../.gitbook/assets/ros_service%20%281%29.png)

Данный способ удобно использовать для периодической передачи данных, или когда существует потребность в синхронной связи \(режим запрос-ответ\).

Сервер отвечает только тогда, когда есть запрос \(Service Request\) и клиент, который может получить ответ \(Service Response\). В отличие от модели работы с топиками, модель сервис работает с "одноразовыми" соединениями. Поэтому, когда цикл запрос и ответ завершен, соединение между двумя нодами будет прервано.

## Описания формата сервиса

Формат запроса и ответа, задается специальным парным Сообщением \(Message\), в котором есть два сообщения, первое для запроса \(Service Request\), второе для ответа \(Service Response\). Файлы с описание сервисов хранятся в директории `srv` и имеют расширение `.srv` Подробное описание файла доступно на странице wiki [http://wiki.ros.org/rosbuild/srv](http://wiki.ros.org/rosbuild/srv)

В файле описания сервиса, первая часть \(до разделителя ---\) это описание сообщения запроса, далее описание сообщения ответа.

Например `srv/AddToInts.srv`

```text
uint32 x

uint32 y

---

uint32 sum
```

При этом, имя файла `AddToInts.srv`, соответсвует имени Сервиса AddToInts.

## Примеры работы на Python

При использовании python на основании данных из файла `srv`, создаются \(при билде пакета\) дополнительные python файлы содержащие описание типа сервиса, сообщение для запроса и сообщение для ответа.

```text
my_package/srv/AddToInts.srv → my_package.srv.AddToInts
my_package/srv/AddToInts.srv → my_package.srv.AddToIntsRequest
my_package/srv/AddToInts.srv → my_package.srv.AddToIntsResponse
```

Более подробно про работу и настройку файлов .srv можно посмотреть в wiki [http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv\#Creating\_a\_srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)

Для создания сервера, выполняющего обработку запросов, используем класс `rospy.Service`

```python
rospy.Service(name, service_class, handler)
```

* `name` Название сервиса \(его адрес\)
* `service_class` тип обрабатываемых сообщений \(должно совпадать с именем файлы `.srv`\)
* `handler` функция обработчик запроса

Пример сервера `src/add_two_ints_server.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from ros_book_samples.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.x, req.y, (req.x + req.y))
    return AddTwoIntsResponse(req.x + req.y)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

При создании запроса на URL `/add_two_ints`, с типом `AddTwoIntsRequest` произойдет суммирование двух числе.

Пример клиента `src/add_two_ints_client.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from ros_book_samples.srv import AddTwoInts

def add_two_ints_client(x, y):

    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp = add_two_ints(x, y)
        return resp.sum

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print add_two_ints_client(10, 22)
```

Запуск примеров возможен или через прямой запуск скриптов python

```text
python src/ros_book_samples/src/add_two_ints_server.py
python src/ros_book_samples/src/add_two_ints_client.py
```

или через запуск утилитой `rosrun`. Для этого `.py` файл должны быть с флагом исполнения \(`chmode 777`\)

```bash
rosrun ros_book_samples add_two_ints_server.py
rosrun ros_book_samples add_two_ints_client.py
```

При запуске клиента и сервера, в консоле можно увидеть

```bash
Для сервера
Ready to add two ints.

Returning [10 + 22 = 32]

Для клиента
rosrun ros_book_samples add_two_ints_client.py
32
```

## Использование Service в новых пакета

Для использования `Service` в новых пакетах, необходимо удостовериться что установлены все зависимости и внесены изменения в конфигурацию `make`.

В файле `CMakeLists.txt` необходимо внести изменения

```text
# Проверяем что пакет message_generation подключен
find_package(catkin REQUIRED
    COMPONENTS message_generation)

# Declare the service files to be built
add_service_files(FILES
    AddTwoInts.srv
)  

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)
```

## Консольная утилита rosservice

Для отладки и тестирования сервисов ROS существует специальная консольная утилита `rosservice`

```bash
rosservice call Выполнение запроса к серверу
rosservice find Поиск сервиса по типу
rosservice info Выводит информацию о сервисе
rosservice list Выводит список запущенных сервисов
rosservice type Выводит тип сообщений используемый сервисом
rosservice uri  Выводит RPC URL сервиса
```

Примеры использования `rosservice`

### rosservice call <a id="rosservice_call"></a>

Вызов сервиса `service_name` c аргументами `service-arg`

```bash
rosservice call /service_name service-args
```

Вызов сервиса `add_two_ints` из примеры выше, с аргументами 1 и 2

```bash
rosservice call /add_two_ints 1 2
```

Для сложных сообщений, аргументы с параметрами возможно писать в YAML синтаксисе например

```bash
rosservice call /add_two_ints "{x: 1, y: 2}"
```

Подробно об использовании YAML [http://wiki.ros.org/ROS/YAMLCommandLine](http://wiki.ros.org/ROS/YAMLCommandLine)

### rosservice list <a id="rosservice_list"></a>

```text
rosservice list
```

Выводит список активных сервисов

### rosservice type <a id="rosservice_type"></a>

```text
rosservice type /service_name
```

Выводит тип обрабатываемого сообщения.

```text
$ rosservice type add_two_ints | rossrv show

result:
int64 a
int64 b
---
int64 sum
```

Пример выводит описание типа сообщения для сервиса

