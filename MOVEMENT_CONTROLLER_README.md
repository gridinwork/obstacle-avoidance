# Movement Controller - Relay-Based Differential Robot

## Описание

Модуль `movement_mavlink.py` реализует управление дифференциальным роботом с реле через RC channel overrides в MAVLink2.

## Архитектура

### Класс MovementController

Централизованный контроллер движения, который управляет:
- **RC5 (ENABLE)**: 1100 = ON, 1900 = OFF
- **RC6 (REVERSE)**: 1100 = FORWARD, 1900 = BACKWARD  
- **RC7 (ARMING)**: 1900 = ARM, 1100 = DISARM
- **CH1 (STEERING)**: 1100 = LEFT, 1500 = CENTER, 1900 = RIGHT
- **CH3 (THROTTLE)**: 1000-2000, формула: `1000 + speed_percent × 10`

### Ключевые особенности

1. **Непрерывная отправка ENABLE**: 
   - Фоновый поток отправляет RC5=1100 каждые 200 мс во время движения
   - Останавливается при вызове `stop()`

2. **Управление реле**:
   - ENABLE должен быть активен во время любого движения
   - REVERSE переключается для движения назад
   - ARMING управляется отдельно

3. **Расчет throttle**:
   - Формула: `PWM = 1000 + speed_percent × 10`
   - Пример: 8% = 1000 + 8×10 = 1080

## API

### Подключение
```python
controller = MovementController(log_fn=print)
controller.connect("/dev/serial0", 57600)
```

### Базовые команды
```python
controller.arm()              # RC7 = 1900
controller.disarm()           # RC7 = 1100
controller.forward(20)        # Движение вперед 20%
controller.backward(15)       # Движение назад 15%
controller.left()             # Поворот влево
controller.right()            # Поворот вправо
controller.stop()             # Остановка (ENABLE OFF)
```

### Управление реле
```python
controller.enable_on()        # RC5 = 1100 (запускает фоновый поток)
controller.enable_off()       # RC5 = 1900 (останавливает поток)
controller.reverse_on()       # RC6 = 1900
controller.reverse_off()      # RC6 = 1100
```

### JSON команды
```python
cmd = {"type": "move", "direction": "forward", "speed": 20}
controller.execute_json_command(cmd)
```

## Интеграция

### В MotionControl

`MovementController` интегрирован в `MotionControl`:
- Используется для всех команд движения
- Синхронизируется с соединением MAVLink автоматически
- Fallback на старую систему, если соединение недоступно

### В AvoidanceController

`AvoidanceController` использует `MovementController` через `MotionControl`:
- Повороты: `motion.movement_controller.left()` / `right()`
- Движение вперед: `motion.movement_controller.forward()`
- Движение назад: `motion.movement_controller.backward()`
- Остановка: `motion.movement_controller.stop()`

## JSON команды (commands.json)

```json
{
  "forward": {"type": "move", "direction": "forward", "speed": 20},
  "backward": {"type": "move", "direction": "backward", "speed": 15},
  "left": {"type": "turn", "direction": "left"},
  "right": {"type": "turn", "direction": "right"},
  "stop": {"type": "stop"},
  "arm": {"type": "system", "action": "arm"},
  "disarm": {"type": "system", "action": "disarm"}
}
```

## Важные замечания

1. **ENABLE должен быть активен**: 
   - Во время движения ENABLE (RC5=1100) отправляется каждые 200 мс
   - При остановке ENABLE выключается (RC5=1900)

2. **Нет DISARM во время движения**:
   - DISARM отправляется только явно через `disarm()`
   - `stop()` НЕ отправляет DISARM

3. **Throttle расчет**:
   - Всегда использует формулу: `1000 + speed_percent × 10`
   - Диапазон: 1000-2000

4. **Steering**:
   - LEFT = 1100
   - CENTER = 1500  
   - RIGHT = 1900

## Тестирование

Для тестирования движения:
```python
from movement_mavlink import MovementController

controller = MovementController()
controller.connect("/dev/serial0", 57600)
controller.arm()
controller.forward(20)  # Движение вперед 20%
time.sleep(2)
controller.stop()
controller.disarm()
```
