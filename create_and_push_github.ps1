# Скрипт для создания репозитория на GitHub и выгрузки проекта
# Требуется: GitHub Personal Access Token с правами repo

param(
    [Parameter(Mandatory=$true)]
    [string]$GitHubToken,
    
    [Parameter(Mandatory=$false)]
    [string]$RepoName = "obstacle-avoidance-v74",
    
    [Parameter(Mandatory=$false)]
    [string]$Description = "Программа для объезда препятствий - версия 74",
    
    [Parameter(Mandatory=$false)]
    [string]$Username = ""
)

Write-Host "=== Создание репозитория на GitHub ===" -ForegroundColor Green

# Если username не указан, попробуем получить из git config
if ([string]::IsNullOrEmpty($Username)) {
    $Username = git config user.name
    if ([string]::IsNullOrEmpty($Username)) {
        Write-Host "Ошибка: не указан GitHub username" -ForegroundColor Red
        Write-Host "Использование: .\create_and_push_github.ps1 -GitHubToken 'token' -Username 'your-username'" -ForegroundColor Yellow
        exit 1
    }
}

Write-Host "Username: $Username" -ForegroundColor Cyan
Write-Host "Repository: $RepoName" -ForegroundColor Cyan

# Создание репозитория через GitHub API
$headers = @{
    "Authorization" = "token $GitHubToken"
    "Accept" = "application/vnd.github.v3+json"
}

$body = @{
    name = $RepoName
    description = $Description
    private = $false
    auto_init = $false
} | ConvertTo-Json

Write-Host "Создание репозитория на GitHub..." -ForegroundColor Yellow
try {
    $response = Invoke-RestMethod -Uri "https://api.github.com/user/repos" -Method Post -Headers $headers -Body $body -ContentType "application/json"
    $repoUrl = $response.clone_url
    Write-Host "Репозиторий создан: $repoUrl" -ForegroundColor Green
} catch {
    Write-Host "Ошибка при создании репозитория: $_" -ForegroundColor Red
    Write-Host "Возможно, репозиторий уже существует. Продолжаем..." -ForegroundColor Yellow
    $repoUrl = "https://github.com/$Username/$RepoName.git"
}

# Подключение remote
Write-Host "`nПодключение к GitHub..." -ForegroundColor Yellow
git remote remove origin 2>$null
git remote add origin $repoUrl

# Переименование ветки
Write-Host "Переименование ветки в main..." -ForegroundColor Yellow
git branch -M main

# Выгрузка
Write-Host "Выгрузка на GitHub..." -ForegroundColor Yellow
git push -u origin main

Write-Host "`n=== Готово! ===" -ForegroundColor Green
Write-Host "Репозиторий: https://github.com/$Username/$RepoName" -ForegroundColor Cyan
