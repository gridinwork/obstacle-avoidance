# Скрипт для выгрузки проекта на GitHub
# Использование: .\push_to_github.ps1 -RepoUrl "https://github.com/username/repo-name.git"

param(
    [Parameter(Mandatory=$true)]
    [string]$RepoUrl
)

Write-Host "Подключение к GitHub репозиторию..." -ForegroundColor Green
git remote add origin $RepoUrl

Write-Host "Переименование ветки в main..." -ForegroundColor Green
git branch -M main

Write-Host "Выгрузка на GitHub..." -ForegroundColor Green
git push -u origin main

Write-Host "Готово! Проект выгружен на GitHub." -ForegroundColor Green
Write-Host "Репозиторий: $RepoUrl" -ForegroundColor Cyan
