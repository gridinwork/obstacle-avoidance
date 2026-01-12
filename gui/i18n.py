"""
Система локализации (интернационализация) для интерфейса.
Поддерживает русский и английский языки.
"""

from typing import Dict

# Словарь переводов
TRANSLATIONS: Dict[str, Dict[str, str]] = {
    "ru": {
        # Окно и вкладки
        "Obstacle Mission": "Программа объезда препятствий",
        "Settings": "Настройки",
        "Visualization": "Визуализация",
        
        # Кнопки
        "START": "СТАРТ",
        "AUTO": "АВТО",
        "Disconnect Devices": "Отключить устройства",
        "Refresh LiDAR": "Обновить LiDAR",
        "Start Camera Visualization": "Запустить камеру",
        "Stop Camera Visualization": "Остановить камеру",
        
        # Статусы устройств
        "Connected": "Подключено",
        "Not Connected": "Не подключено",
        "Connecting": "Подключение...",
        "Disconnected": "Отключено",
        "Error": "Ошибка",
        
        # Метки устройств
        "LiDAR": "LiDAR",
        "MAVLink": "MAVLink",
        "Camera": "Камера",
        "Mission": "Миссия",
        "Avoidance": "Объезд",
        
        # Настройки
        "Pixhawk Port: /dev/serial0 (UART)": "Порт Pixhawk: /dev/serial0 (UART)",
        "Series": "Серия",
        "Baudrate": "Скорость передачи",
        "LiDAR Port (/dev/ttyUSB*)": "Порт LiDAR (/dev/ttyUSB*)",
        "Camera Selection": "Выбор камеры",
        "Save": "Сохранить",
        "Cancel": "Отмена",
        
        # Группы настроек
        "Speed & Limits": "Скорость и ограничения",
        "Avoidance Settings": "Настройки объезда",
        "LiDAR & Boundaries": "LiDAR и границы",
        
        # Параметры скорости
        "Speed (PWM 1000-2000)": "Скорость (PWM 1000-2000)",
        "Movement Coefficient (sec per meter)": "Коэффициент движения (сек/метр)",
        "Yellow Zone (cm)": "Желтая зона (см)",
        "Red Zone (cm)": "Красная зона (см)",
        
        # Параметры объезда
        "Turn angle (deg)": "Угол поворота (град)",
        "Travel distance (m)": "Дистанция движения (м)",
        "Backoff distance (m)": "Дистанция отката (м)",
        "Reverse duration (s)": "Длительность реверса (с)",
        "Post-turn lock (s)": "Блокировка после поворота (с)",
        "Command/mode cooldown (s)": "Задержка команд/режимов (с)",
        
        # Параметры LiDAR
        "Blue Boundary Distance (cm)": "Расстояние синей границы (см)",
        "Green Boundary Distance (cm)": "Расстояние зеленой границы (см)",
        "LiDAR Buffer Size (bytes)": "Размер буфера LiDAR (байт)",
        "LiDAR Read Frequency (Hz)": "Частота чтения LiDAR (Гц)",
        
        # Чекбоксы
        "Start automatically after 5 seconds": "Автозапуск через 5 секунд",
        
        # Сообщения камеры
        "Camera feed is off": "Камера выключена",
        "Opening camera ({device})...": "Открытие камеры ({device})...",
        "Camera disconnected": "Камера отключена",
        "Camera visualization stopped (device remains connected)": "Визуализация камеры остановлена (устройство подключено)",
        "Visualization locked — press START": "Визуализация заблокирована — нажмите СТАРТ",
        "Camera visualization stopped": "Визуализация камеры остановлена",
        "Camera disabled in Settings": "Камера отключена в настройках",
        "Camera feed unavailable": "Видеопоток недоступен",
        "Camera frame invalid": "Неверный кадр камеры",
        
        # Сообщения LiDAR
        "LiDAR visualization is off": "Визуализация LiDAR выключена",
        "LiDAR warming up...": "LiDAR прогревается...",
        "LiDAR data unavailable": "Данные LiDAR недоступны",
        
        # Сообщения статуса
        "Starting devices...": "Запуск устройств...",
        "Devices connected, initializing...": "Устройства подключены, инициализация...",
        "Device connection failed": "Ошибка подключения устройств",
        "Devices ready": "Устройства готовы",
        "Disconnecting devices...": "Отключение устройств...",
        "Devices disconnected": "Устройства отключены",
        "LiDAR not connected": "LiDAR не подключен",
        "LiDAR refreshed": "LiDAR обновлен",
        "LiDAR refresh failed": "Ошибка обновления LiDAR",
        "Select camera device in Settings": "Выберите камеру в настройках",
        "Camera visualization stopped": "Визуализация камеры остановлена",
        "Camera disabled": "Камера отключена",
        
        # Опции камеры
        "Raspberry Pi Camera (libcamera)": "Камера Raspberry Pi (libcamera)",
        "USB Camera ({dev})": "USB камера ({dev})",
        "Off": "Выкл",
        
        # Отладочные сообщения LiDAR
        "LiDAR cluster size: {size}": "Размер кластера LiDAR: {size}",
        "Obstacle cluster validated": "Кластер препятствия подтвержден",
        "Noise filtered — insufficient points": "Шум отфильтрован — недостаточно точек",
        "Frames stable: {stable} / {required} (>=1 sec)": "Стабильных кадров: {stable} / {required} (>=1 сек)",
    },
    "en": {},  # Английский - оригинальные строки
}

# Текущий язык (по умолчанию русский)
_current_language = "ru"


def set_language(lang: str) -> None:
    """Установить язык интерфейса."""
    global _current_language
    if lang in TRANSLATIONS:
        _current_language = lang
    else:
        _current_language = "en"


def get_language() -> str:
    """Получить текущий язык."""
    return _current_language


def tr(text: str, **kwargs) -> str:
    """
    Перевести текст на текущий язык.
    
    Args:
        text: Текст для перевода
        **kwargs: Параметры для форматирования строки
        
    Returns:
        Переведенный текст или оригинал, если перевод не найден
    """
    if _current_language == "en" or _current_language not in TRANSLATIONS:
        # Английский - возвращаем оригинал
        if kwargs:
            return text.format(**kwargs)
        return text
    
    translations = TRANSLATIONS[_current_language]
    
    # Прямой перевод
    if text in translations:
        translated = translations[text]
        if kwargs:
            return translated.format(**kwargs)
        return translated
    
    # Попытка найти перевод с форматированием
    for key, value in translations.items():
        if key in text or text in key:
            # Если есть параметры, попробуем заменить плейсхолдеры
            if kwargs:
                try:
                    return value.format(**kwargs)
                except (KeyError, ValueError):
                    pass
            return value
    
    # Если перевод не найден, возвращаем оригинал с форматированием
    if kwargs:
        return text.format(**kwargs)
    return text


# Алиас для удобства
_ = tr
