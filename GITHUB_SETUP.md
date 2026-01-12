# Настройка GitHub в Cursor

## Шаг 1: Установка Git (если не установлен)

1. Скачайте Git для Windows: https://git-scm.com/download/win
2. Установите с настройками по умолчанию
3. Перезапустите Cursor после установки

Или через PowerShell (от администратора):
```powershell
winget install Git.Git
```

## Шаг 2: Настройка GitHub аккаунта в Cursor

### Вариант A: Через Source Control панель

1. **Откройте Source Control панель:**
   - Нажмите `Ctrl+Shift+G` 
   - Или кликните на иконку ветки (branch) в левой панели

2. **Войдите в GitHub:**
   - В верхней части Source Control панели нажмите на иконку "..." (три точки)
   - Выберите "Sign in with GitHub" или "Clone Repository"
   - Откроется браузер для авторизации
   - Разрешите доступ Cursor к вашему GitHub аккаунту

### Вариант B: Через Command Palette

1. Нажмите `Ctrl+Shift+P` (открыть Command Palette)
2. Введите: `Git: Clone`
3. Или введите: `GitHub: Sign in`
4. Следуйте инструкциям для авторизации

### Вариант C: Через настройки

1. Нажмите `Ctrl+,` (открыть Settings)
2. Найдите "Git" в поиске
3. Найдите "GitHub" в поиске
4. Настройте авторизацию

## Шаг 3: Инициализация репозитория

После авторизации в GitHub:

1. **Откройте папку проекта:**
   - File → Open Folder → выберите `version_74`

2. **Инициализируйте Git репозиторий:**
   - В Source Control панели (`Ctrl+Shift+G`)
   - Нажмите "Initialize Repository" (если появится)
   - Или через терминал: `git init`

3. **Добавьте файлы:**
   - В Source Control панели все файлы появятся как "Changes"
   - Нажмите "+" рядом с файлами или "Stage All Changes"
   - Введите сообщение коммита: "Initial commit - version 74"
   - Нажмите "Commit"

4. **Опубликуйте на GitHub:**
   - После коммита появится кнопка "Publish Branch"
   - Нажмите на неё
   - Выберите "Publish to GitHub"
   - Создайте новый репозиторий или выберите существующий
   - Нажмите "Publish"

## Шаг 4: Проверка

После публикации:
- Репозиторий будет доступен на GitHub.com
- В Source Control панели появится информация о remote
- Можно делать push/pull через интерфейс Cursor

## Полезные команды в Cursor

- `Ctrl+Shift+G` - Source Control панель
- `Ctrl+Shift+P` - Command Palette (для Git команд)
- `Ctrl+` ` - Терминал (для Git команд вручную)

## Если возникли проблемы

1. **Git не найден:**
   - Убедитесь, что Git установлен и добавлен в PATH
   - Перезапустите Cursor

2. **Не могу войти в GitHub:**
   - Проверьте интернет соединение
   - Попробуйте через браузер: https://github.com/settings/tokens
   - Создайте Personal Access Token и используйте его

3. **Ошибки при push:**
   - Проверьте права доступа к репозиторию
   - Убедитесь, что вы авторизованы в GitHub
