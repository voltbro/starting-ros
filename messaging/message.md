# Сообщения

Сообщение представляет собой структуру данных, которая используется при обмене информацией между нодами.

Топики \(Topic\), службы \(Services\) и Действия \(Actions\) используют сообщения для взаимодействия между собой. Сообщения могут включать в себя как базовые типы \(целое число, число с плавающей точкой, логические и тд\), так и массивы сообщений.

Помимо этого, сообщения могут инкапсулировать в себе другие существующие типы сообщений и специальные заголовки.

Сообщения описываются в файлах `.msg` как пары значений: тип поля и имя поля.

```text
fieldtype fieldname
fieldtype1 fieldname1
```

Для реального примера, сообщение может выглядеть так:

```text
int32 x
int32 y
```

Мы описали сообщение, содержащее две переменных типа `int32` с именами `x` и `y`

## Базовые типы ROS

Типы данных ROS не заимствованы на прямую ни из одного из языков. В момент "сборки" происходит преобразование типа ROS к типу используемого языка.

В таблице ниже описаны базовые типы ROS и их представление в языках С++ и Python

| в ROS | для C++ | для Python |
| :--- | :--- | :--- |
| bool | uint8\_t | bool |
| int8 | int8\_t | int |
| uint8 | uint8\_t | int |
| int16 | int16\_t | int |
| uint16 | uint16\_t | int |
| int32 | int32\_t | int |
| uint32 | uint32\_t | int |
| int64 | int64\_t | long |
| uint64 | uint64\_t | long |
| float32 | float | float |
| float64 | double | float |
| string | std::string | str |
| time | ros::Time | rospy.Time |
| duration | ros::Duration | rospy.Duration |

Отдельно стоит отметить типы данных

| в ROS | для С++ | для Python |
| :--- | :--- | :--- |
| fixed-length | boost::array, std::vector | tuple |
| variable-length | std::vector | tuple |
| uint8\[\] | std::vector | bytes |
| bool\[\] | std::vector&lt;uint8\_t&gt; | list of bool |

### Наследование типов сообщений

Сообщения могут содержать не только базовые типы, но и переиспользовать уже созданные типы сообщений. Например сообщение типа `geometry_msgs/Pose` описывается конфигурацией

```text
Point position
Quaternion orientation
```

Где `Point` и `Quaternion` это существующие типы сообщений.

Если рассмотреть сообщение `geometry_msgs/Pose` в развернутом виде, то мы увидим структуру:

```text
geometry_msgs/Point position

  float64 x
  float64 y

float64 z

geometry_msgs/Quaternion orientation

  float64 x

  float64 y
  float64 z
  float64 w
```

## Создание новых сообщений

В процессе разработки часто возникает необходимость создавать собственные типы сообщений с уникальной структурой данных. Файлы с описанием сообщений необходимо хранить в директории `./msg` При этом имя файла определяет название типа созданного сообщения.

Например, мы хотим использовать данные с датчика давления и температуры BMP180. Мы предполагаем, что нам понадобится передавать два значения: давление и температура \(стандартные параметры для барометрических датчиков\).

Создадим файл `./msg/Barometer.msg`

```text
uint16  pressure
float32 temperature
```

Для правильного подключения созданных `.msg` файлов необходимо удостовериться, что установлены все зависимости и внесены изменения в конфигурацию пакета.

Файл `package.xml` должен содержать следующие строчки

```text
<build_depend>message_generation</build_depend>
<exec_depend>message_generation</exec_depend>
```

Файл CMakeLists.txt должен содержать следующие изменения для подключения процесса обработки сообщений

```text
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation)

catkin_package(
   CATKIN_DEPENDS
   message_generation
   roscpp
   rospy
   std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(

   FILES

   Barometer.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
)
```

После этого необходимо запустить процесс сборки пакета командой `catkin_make`

```bash
$ cd ~/catkin_ws && catkin_make
```

Проверить правильность создания нового типа сообщений `Barometer.msg` в пакет можно выполнить поиском по названию сообщения:

```bash
$ rosmsg list | grep Baro

ros_book_samples/Barometer
```

Подключить `.msg` файл в `python`

```python
import rospy
from ros_book_samples.msg import Barometer
```

После изменения файла `.msg` всегда необходимо "пересобирать" проект через вызов `catkin_make.` Файлы `.msg` не используются напрямую, а служат инструкцией для генерации вспомогательных файлов, которые подключаются после их вызова. Например из файла `Barometer.msg` создается файл для `python` `devel/lib/python2.7/dist-packages/ros\_book\_samples/msg/\_Barometer.py`.

## Консольная утилита rosmsg

`rosmsg` это удобный инструмент командной строки, который предоставляют справочную информацию для разработчиков, а также служит мощным средством анализа для получения дополнительной информации о данных, передаваемых в ROS.

Например, если вы используете сообщение в своем коде, вы можете выполнить `rosmsg show` в командной строке для получения списка полей используемых в сообщении:

```bash
$ rosmsg show sensor_msgs/CameraInfo

Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
RegionOfInterest roi
  uint32 x_offset
  uint32 y_offset
  uint32 height
  uint32 width
float64[5] D
float64[9] K
float64[9] R
float64[12] P
```

### Команды консольной утилиты `rosmsg`

```text
Commands:
    rosmsg show    Показать информацию о сообщении
    rosmsg info    Алиас для команды rosmsg show
    rosmsg list    Вывести все существующие типы сообщений
    rosmsg md5    Отобразить md5sum сообщения
    rosmsg package    Список всех сообщений в пакете
    rosmsg packages    Список пакетов, использующих сообщение
```

