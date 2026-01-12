# Movement Controller Update Summary

## Изменения в версии 74

### Новый модуль: `movement_mavlink.py`

Создан класс `MovementController` для управления дифференциальным роботом с реле через RC channel overrides.

#### Ключевые функции:
- **connect(port, baud)**: Подключение к MAVLink
- **disconnect()**: Отключение и остановка
- **arm() / disarm()**: Управление ARMING (RC7)
- **forward(speed_percent)**: Движение вперед
- **backward(speed_percent)**: Движение назад
- **left() / right()**: Повороты
- **stop()**: Остановка (ENABLE OFF)
- **execute_json_command(cmd)**: Выполнение JSON команд

#### Реле и каналы:
- **RC5 (ENABLE)**: 1100 = ON, 1900 = OFF (отправляется каждые 200 мс во время движения)
- **RC6 (REVERSE)**: 1100 = FORWARD, 1900 = BACKWARD
- **RC7 (ARMING)**: 1900 = ARM, 1100 = DISARM
- **CH1 (STEERING)**: 1100 = LEFT, 1500 = CENTER, 1900 = RIGHT
- **CH3 (THROTTLE)**: 1000-2000, формула: `1000 + speed_percent × 10`

### Обновленные модули:

#### `motion_control.py`
- Добавлен `MovementController` как основной контроллер движения
- `handle_key()` использует `MovementController` для ручного управления
- `apply_speed()` использует `MovementController.forward()`
- `stop_vehicle()` использует `MovementController.stop()`
- `move_backward_one_meter()` использует `MovementController.backward()`
- Fallback на старую систему, если соединение недоступно

#### `avoidance.py`
- `_apply_speed()` использует `MovementController.forward()`
- `_handle_yellow()` использует `MovementController.left()` / `right()` для поворотов
- `_reverse_until_yellow()` использует `MovementController.backward()`
- `_drive_until_clear()` использует `MovementController.forward()`
- `_forward_until_clear()` использует `MovementController.forward()`

#### `commands.json`
Обновлен формат команд для поддержки нового контроллера:
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

### Важные особенности:

1. **Непрерывная отправка ENABLE**:
   - Фоновый поток отправляет RC5=1100 каждые 200 мс во время движения
   - Автоматически останавливается при вызове `stop()`

2. **Нет DISARM во время движения**:
   - `stop()` НЕ отправляет DISARM
   - DISARM отправляется только явно через `disarm()`

3. **Синхронизация соединения**:
   - `MovementController` автоматически синхронизируется с соединением MAVLink
   - Использует то же соединение, что и `MavlinkComm`

4. **Обратная совместимость**:
   - Старая система (`MavlinkMotionCommands`) используется как fallback
   - Если `MovementController` не подключен, используется старая система

### Тестирование:

Для проверки работы:
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

### Миграция:

Все команды движения теперь проходят через `MovementController`:
- Ручное управление (клавиатура) → `MovementController`
- Автономное движение (obstacle avoidance) → `MovementController`
- JSON команды → `MovementController.execute_json_command()`

Старая система (`MavlinkMotionCommands`) остается как fallback для совместимости.
