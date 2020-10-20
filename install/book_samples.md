# Установка примеров

## Установка git

Для скачивания и установки пакета содержащего примеры кода приведенные в книге, необходимо установить систему контроля версий `git`

```bash
sudo apt-get install git
```

После установки git можно проверить его работоспособность

```bash
git --version
```

## Скачивание пакета ros\_book\_samples

Исходные кода примеров находятся в открытом доступе в git репозитории [https://github.com/voltbro/ros\_book\_samples](https://github.com/voltbro/ros_book_samples)

Установка пакета должна происходить в директории `~/catkin_ws/src/` Если у вас нет директории catkin\_ws, выполните инструкцию по установки [ROS для разработчиков](./#nastroika-rabochego-okruzheniya)

Скачать примеры:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/voltbro/ros_book_samples
```

В директории `~/catkin_ws/src/ros_book_samples` должен появиться исходный код пакета.

## Сборка \(make\) пакета ros\_book\_samples

Сборку \(компиляцию и установку\) пакета выполняет команда `catkin_make`

```bash
cd ~/catkin_ws/
$ catkin_make
```

Приблизительный вывод выполнения сборки проекта

```text
Base path: /home/cola/catkin_ws
Source space: /home/cola/catkin_ws/src
Build space: /home/cola/catkin_ws/build
Devel space: /home/cola/catkin_ws/devel
Install space: /home/cola/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/cola/catkin_ws/build"
####
####
#### Running command: "make -j2 -l2" in "/home/cola/catkin_ws/build"
####
[  0%] Built target actionlib_msgs_generate_messages_py
[  0%] Built target _ros_book_samples_generate_messages_check_deps_DoDishesActionResult
[  0%] Built target _ros_book_samples_generate_messages_check_deps_DoDishesGoal
[  0%] Built target _ros_book_samples_generate_messages_check_deps_DoDishesResult
[  0%] Built target _ros_book_samples_generate_messages_check_deps_AddTwoInts
[  0%] Built target _ros_book_samples_generate_messages_check_deps_DoDishesActionFeedback
[  0%] Built target std_msgs_generate_messages_py
[  0%] Built target _ros_book_samples_generate_messages_check_deps_DoDishesActionGoal
[  0%] Built target _ros_book_samples_generate_messages_check_deps_DoDishesAction
[  0%] Built target std_msgs_generate_messages_lisp
[  0%] Built target actionlib_msgs_generate_messages_lisp
[  0%] Built target _ros_book_samples_generate_messages_check_deps_DoDishesFeedback
[  0%] Built target actionlib_msgs_generate_messages_cpp
[  0%] Built target std_msgs_generate_messages_cpp
[  0%] Built target std_msgs_generate_messages_eus
[  0%] Built target actionlib_msgs_generate_messages_eus
[  0%] Built target actionlib_msgs_generate_messages_nodejs
[  0%] Built target std_msgs_generate_messages_nodejs
[ 23%] Built target ros_book_samples_generate_messages_py
[ 41%] Built target ros_book_samples_generate_messages_lisp
[ 60%] Built target ros_book_samples_generate_messages_cpp
[ 81%] Built target ros_book_samples_generate_messages_eus
[100%] Built target ros_book_samples_generate_messages_nodejs
[100%] Built target ros_book_samples_generate_messages
```

После завершения сборки проекта, в директории catkin\_ws появятся директории `build` и `devel` в которых будут находиться файлы готовых к работе пакетов.

## Настройка рабочего окружения ros\_book\_samples

По умолчанию, пакеты `ROS` при установке через `apt-get` устанавливаются в директорию `/opt/ros/noetic/share` собранный пакет `ros_book_smaples` через `catkin_make` по умолчанию не инсталируется в эту директорию.

Для удобства работы, проще всего добавить еще одну директорию, где `ROS` должен искать установленные пакеты. Тогда установленные пакеты через `apt-get` будут располагаться в обычном для них месте, а наши пакеты будут находиться в домашней директории пользователя `~/catkin_ws/src/`.

Если вы используете `bash`

```bash
echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Если `zsh`

```bash
echo "source ${HOME}/catkin_ws/devel/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

Для проверки настройки новой директории для пакетов `ROS` можно выполнить команду

```bash
roscd ros_book_samples
```

После этого вы должны переместиться в директорию установленного пакета `/home/user/catkin_ws/src/ros_book_samples`

Команда

```text
roscd ros
```

Должна переместить вас в директорию `/opt/ros/noetic/share/ros`

Отобразить текущий список директорий для пакетов можно командой

```text
echo $ROS_PACKAGE_PATH

/home/user/catkin_ws/src:/opt/ros/noetic/share
```

Мы скачали, скомпилировали и настроили проект, содержащий примеры. Для установки нового пакета достаточно скачать пакет в директорию `~/catkin_ws/src/` и запустить `catkin_make`

