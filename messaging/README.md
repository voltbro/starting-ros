# Обмен сообщениями

Как мы обсудили ранее, ключевая концепция ROS предполагает создание множества независимых нод, которые взаимодействуют друг с другом. В этом главе мы подробно рассмотрим способы коммуникации между нодами.

Существуют три основных способа (концепции) коммуникации:

* Топик (Theme), который обеспечивает синхронную однонаправленную передачу / прием сообщений;
* Сервис (Service) , который обеспечивает синхронное двунаправленное взаимодействие: запрос / ответ сообщения;
* Действие (Action), которое обеспечивает асинхронное двунаправленное взаимодействие с шагами: цель - результат - обратная связь.

Диаграмму коммуникации можно изобразить схемой:

![](<../.gitbook/assets/Полотно 1.png>)
