#!/usr/bin/env python3
import os
import subprocess
import sys

def run_command(command, cwd=None):
    """
    Запуск команды и вывод процесса выполнения
    
    Args:
        command (str): Команда для выполнения
        cwd (str, optional): Рабочая директория для выполнения команды
    """
    print(f"Выполняем: {command}")
    if cwd:
        print(f"В директории: {cwd}")
    
    process = subprocess.Popen(
        command, 
        shell=True, 
        stdout=subprocess.PIPE, 
        stderr=subprocess.STDOUT,
        cwd=cwd
    )
    
    for line in iter(process.stdout.readline, b''):
        sys.stdout.write(line.decode('utf-8'))
    
    process.wait()
    if process.returncode != 0:
        print(f"Ошибка: команда '{command}' завершилась с кодом {process.returncode}")
        sys.exit(process.returncode)

def main():
    # Определение путей к проектам
    main_project_dir = os.getcwd()  # TechSimApi (корневая директория)
    submodule_dir = os.path.join(main_project_dir, "InavMSPApi")  # Подмодуль в корне
    
    # Проверка существования директорий
    if not os.path.isdir(submodule_dir):
        print(f"Ошибка: директория подмодуля '{submodule_dir}' не найдена!")
        print("Убедитесь, что вы клонировали подмодуль с помощью 'git submodule update --init'")
        sys.exit(1)
    
    try:
        # 1. Устанавливаем зависимости и пакет для подмодуля InavMSPApi
        print("\n" + "="*50)
        print("Установка зависимостей подмодуля InavMSPApi")
        print("="*50)
        
        # Проверка существования файла requirements.txt
        submodule_req_file = os.path.join(submodule_dir, "requirements.txt")
        if os.path.isfile(submodule_req_file):
            run_command("pip install -r requirements.txt", cwd=submodule_dir)
        else:
            print(f"Предупреждение: файл {submodule_req_file} не найден, пропускаем установку зависимостей подмодуля")
        
        # Проверка существования setup.py
        submodule_setup_file = os.path.join(submodule_dir, "setup.py")
        if os.path.isfile(submodule_setup_file):
            run_command("pip install -e .", cwd=submodule_dir)
        else:
            print(f"Предупреждение: файл {submodule_setup_file} не найден, пропускаем установку подмодуля")
        
        # 2. Устанавливаем зависимости и пакет для главного проекта TechSimApi
        print("\n" + "="*50)
        print("Установка зависимостей основного проекта TechSimApi")
        print("="*50)
        
        # Проверка существования файла requirements.txt
        main_req_file = os.path.join(main_project_dir, "requirements.txt")
        if os.path.isfile(main_req_file):
            run_command("pip install -r requirements.txt", cwd=main_project_dir)
            run_command("pip install tornado backports.ssl_match_hostname", cwd=main_project_dir)
        else:
            print(f"Предупреждение: файл {main_req_file} не найден, пропускаем установку зависимостей основного проекта")
        
        # Проверка существования setup.py
        main_setup_file = os.path.join(main_project_dir, "setup.py")
        if os.path.isfile(main_setup_file):
            run_command("pip install -e .", cwd=main_project_dir)
        else:
            print(f"Предупреждение: файл {main_setup_file} не найден, пропускаем установку основного проекта")
        
        print("\n" + "="*50)
        print("Установка успешно завершена!")
        print("="*50)
        
    except Exception as e:
        print(f"Произошла ошибка: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
