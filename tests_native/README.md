# Native C++ tests

Запускают настоящий `protocol_uart.cpp`, `antenna_controller.cpp` и предоставленный
GyverStepper на ПК. GPIO, Servo PWM, очереди RTOS и часы заменены простыми тестовыми
реализациями. Это проверка алгоритмов, не эмуляция времени выполнения AVR/ISR.

Из корня комплекта, при установленном g++:

```sh
g++ -std=c++11 -DANTENNA_MECHANICS_CONFIGURED=1 -Itests_native/stubs -Ifirmware/lib/GyverStepper-main/src -Ifirmware/lib/ArduinoJson-6.21.5/src tests_native/firmware_tests.cpp -o firmware_tests
./firmware_tests
g++ -std=c++11 -Ifirmware/lib/ArduinoJson-6.21.5/src tests_native/protocol_validation.cpp -o protocol_validation
./protocol_validation
```

`firmware_tests` проверяет парсер, запрет движения до калибровки, разрешение после неё,
продолжительное движение, смену направления, границы каждого шага, STALE с остановкой,
недостижимую цель, невалидную ось и DISARM. `protocol_validation` проверяет JSON-хвост,
дублирование ключей, неверные числовые литералы и типы ArduinoJson.

AVR-сборка проверяется отдельно: `tools/build_avr.py` принимает пути к локальным
ArduinoCore-avr и toolchain avr-gcc. Флаг `--configured` нужен только для проверки
сборки активной логики: он включает демонстрационные пределы, не проверенные на стенде.
Не загружайте результат такой сборки в реальный механизм без настройки параметров.
