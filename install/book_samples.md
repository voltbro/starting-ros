# Установка примеров

### Установка git

Для скачивания и установки пакета содержащего примеры кода приведенные в книге,  необходимо установить программу `git`

```bash
sudo apt-get install git
```

После установки git можно проверить его работоспособность запустив

```bash
git --version
```

### Скачивание пакета ros\_book\_samples

Исходные кода примеров находятся в открытом доступе в git репизитории [https://github.com/voltbro/ros\_book\_samples](https://github.com/voltbro/ros_book_samples)

Устанвка пакета должны происходить в директории `~/catkin_ws/src/` Если у вас нет директории catkin\_ws, выполните инструкцию по установки  [ROS для разработчиков](./#nastroika-rabochego-okruzheniya)

Выполните команды

```bash
cd ~/catkin_ws/src/
git clone https://github.com/voltbro/ros_book_samples
```

В директории `~/catkin_ws/src/ros_book_samples` должен появиться исходный код пакета.

### Сборка \(make\) пакета ros\_book\_samples

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

После завершиния сборки проекта, в директории catkin\_ws появятся директории `build` и `devel` в которых бедет находиться файлы "готовых" пакетов.

### Настройка рабочего окружения ros\_book\_samples

По умолчанию, пакеты `ROS` при установке через `apt-get` устанавливаются в директорию `/opt/ros/kinetic/share` собранный пакет `ros_book_smaples` через `catkin_make` по умолчанию не инсталируется в эту директорию.

Для удобства работы, проще всего добавить еще одну директорию, где `ROS` должен искать пакеты.

Для `bash`

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

source /home/cola/catkin\_ws/devel/setup.zsh

```bash
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

