# Установка

Поскольку ROS - это мета-операционная система, вам нужно будет выбрать ОС для использования. ROS поддерживает Debian, Ubuntu, Linux Mint, OS X, Fedora, Gentoo, openSUSE, Arch Linux и Windows \(через виртуализацию Linux\), но самыми популярными операционными системами являются Debian, Ubuntu и Linux Mint. В нашем учебнике мы будем использовать ОС Ubuntu. Для слабых компьютеров, рекомендуеться использовать ОС Lubuntu \(совместима с Ubuntu\) 

На сегодняшний день, актуальным дистрибутивом ROS являеться **ROS Melodic Morenia** [http://wiki.ros.org/melodic](http://wiki.ros.org/melodic) этот дистрибутив мы и установим.

Оригинальная инструкция по устновке ROS находится на сайте ROS [https://www.ros.org/install/](https://www.ros.org/install/). За основу дальнейшей инструкции будет взята страница [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu). 

Для утсановки ROS на Ubuntu существуют готовые пакеты, нам достаточно добавить репозиторий пакетов и установить их. В противном случае, необходимо "собирать" все пакеты из исходного кода.

## Установка пакетов

#### Добавление репозитория пакетов

Открываем окно терминала \(программа terminal\) \(возможно Ctrl-Alt-T\)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Добавление ключей

```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Обратите внимание, что ключ может измениться, поэтому уточните его на официальной Wiki странице [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu).

#### Обновлние списка пакетов

Теперь, когда мы добавили и настроили репозиторий с пакетами ROS, мы должны обновить список пакетов доступных системе для инсталляции. Также мы рекомендуем обновить все установленные пакеты Ubuntu до установки ROS.

```bash
sudo apt-get update && sudo apt-get upgrade -y
```

#### Установка пакетов ROS

Мы рекомендуем установить самую полную версию системы. Данный пакет увтоматический установит все основные пакеты  ROS, rqt, rviz, библиотеки 2D/3D симуляции, навигации и тп.

```bash
sudo apt-get install ros-melodic-desktop-full
```

#### Установка дополнительных пакетов

Если необходимо установить дополнительный пакет, то это можно сделать командой. Например добавить пакет slam-gmapping можно командой.

```bash
sudo apt-get install ros-melodic-slam-gmapping
```

Поиск пакетов можно выполнять командой 

```bash
apt-cache search ros-melodic
```

## Настройка после установки

#### Установка rosdep

Прежде чем использовать ROS, вам необходимо настроить `rosdep`. `rosdep` позволяет вам  устанавливать системные зависимости для исходных кодов, который вы хотите скомпилировать, и требуется для запуска некоторых основных компонентов в ROS.

```bash
sudo rosdep init
rosdep update
```

### Настройка рабочего окружения

Добавим переменные окружения ROS, для их автоматической установки при запуске оболочки bash:

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Если вы просто хотите загрузить переменные ROS в текущем сеансе, то вы можете ввести:

```bash
source /opt/ros/melodic/setup.bash
```

Если вы используете zsh вместо bash, вам нужно запустить следующие команды для настройки вашей оболочки:

```bash
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

## Для разработчиков

Если вы собираетесь самостоятельно разрабатывать или вносить изменения в пакеты, вам необходимо установить дополнительные средства для разработчика. 

```bash
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

#### Создание и настройка рабочего пространства

ROS использует специальную систему сборки под названием `catkin`. Чтобы использовать ее, вам необходимо создать и инициализировать папку рабочего пространства. 

```bash
 mkdir ~/catkin_ws/src
 cd ~/catkin_ws/src 
 catkin_init_workspace
```

## Первый запуск

Установка для ROS завершена успешна, следующая команда запустит сервер главную ноду ROS. Закройте все окна терминала и откройте новое окно терминала. 

```bash
roscore
```

Если ROS установлена верно, то мы увидим приблизительно такое сообщение при запуске:

![](.gitbook/assets/roscore_run.png)

Для остановки `roscore` несобходимо нажать Ctrl+C 

