#!/usr/bin/env python3
import os
import subprocess
import sys
import shutil

def run_command(command, cwd=None, error_message="Command execution error"):
    """
    Execute a command and display the execution process
    
    Args:
        command (str): Command to execute
        cwd (str, optional): Working directory for command execution
        error_message (str): Error message if execution fails
    """
    print(f"Executing: {command}")
    if cwd:
        print(f"In directory: {cwd}")
    
    try:
        process = subprocess.Popen(
            command, 
            shell=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.STDOUT,
            cwd=cwd,
            text=True,
            encoding='utf-8',
            errors='replace'
        )
        
        for line in process.stdout:
            sys.stdout.write(line)
        
        process.wait()
        if process.returncode != 0:
            print(f"\n{error_message}: command '{command}' exited with code {process.returncode}")
            return False
            
        return True
        
    except Exception as e:
        print(f"\nException while executing command: {str(e)}")
        return False

def check_pip_list():
    """Check installed packages via pip list"""
    print("\n" + "="*60)
    print("Checking installed packages via pip list")
    print("="*60)
    
    print("\nSearching for inavmspapi in installed packages:")
    run_command("pip list | findstr /i inavmspapi", error_message="inavmspapi not found in pip list")
    
    print("\nSearching for agrotechsimapi in installed packages:")
    run_command("pip list | findstr /i agrotechsimapi", error_message="agrotechsimapi not found in pip list")
    
    print("\nChecking installation via pip show:")
    run_command("pip show inavmspapi", error_message="pip show inavmspapi failed")
    run_command("pip show agrotechsimapi", error_message="pip show agrotechsimapi failed")

def clone_and_install_inavmspapi(base_dir):
    """
    Clone and install inavmspapi in a local directory
    
    Args:
        base_dir (str): Base directory for installation
        
    Returns:
        bool: True if successful, False otherwise
    """
    print("\n" + "="*60)
    print("Installing inavmspapi from GitHub (tag v1.1.0)")
    print("="*60)
    
    inavmspapi_dir = os.path.join(base_dir, "inavmspapi_local")
    
    print(f"Base directory: {base_dir}")
    print(f"Directory for inavmspapi: {inavmspapi_dir}")
    
    try:
        # Repository URL for inavmspapi
        repo_url = "https://github.com/AgroTechAvia/InavMSPApi.git"
        tag = "v1.1.0"
        
        # If directory already exists, remove it
        if os.path.exists(inavmspapi_dir):
            print(f"\nRemoving existing directory: {inavmspapi_dir}")
            shutil.rmtree(inavmspapi_dir)
        
        # Create directory for inavmspapi
        os.makedirs(inavmspapi_dir, exist_ok=True)
        
        # Clone repository
        print(f"\nCloning repository {repo_url} with tag {tag}")
        if not run_command(f"git clone {repo_url} .", cwd=inavmspapi_dir,
                          error_message="Error cloning repository"):
            return False
        
        # Switch to the specified tag
        if not run_command(f"git checkout {tag}", cwd=inavmspapi_dir,
                          error_message=f"Error switching to tag {tag}"):
            return False
        
        print(f"[ OK ]  Repository successfully cloned with tag {tag}")
        
        # Install the inavmspapi package in development mode
        print("\nInstalling inavmspapi package (development mode)...")
        
        # Check if pyproject.toml or setup.py exists
        pyproject_file = os.path.join(inavmspapi_dir, "pyproject.toml")
        setup_file = os.path.join(inavmspapi_dir, "setup.py")
        
        if os.path.isfile(pyproject_file):
            # For modern packaging with pyproject.toml
            if not run_command("pip install -e .", cwd=inavmspapi_dir,
                             error_message="Error installing inavmspapi"):
                return False
        elif os.path.isfile(setup_file):
            # For legacy setup.py
            if not run_command("pip install -e .", cwd=inavmspapi_dir,
                             error_message="Error installing inavmspapi"):
                return False
        else:
            print("Error: pyproject.toml or setup.py not found in inavmspapi")
            return False
        
        print(f"[ OK ]  inavmspapi successfully installed at: {inavmspapi_dir}")
        
        return True
        
    except Exception as e:
        print(f"Error installing inavmspapi: {str(e)}")
        return False

def install_agrotechsimapi(agrotechsimapi_dir):
    """
    Install the main agrotechsimapi module
    
    Args:
        agrotechsimapi_dir (str): Directory containing agrotechsimapi
        
    Returns:
        bool: True if successful, False otherwise
    """
    print("\n" + "="*60)
    print("Installing main agrotechsimapi module")
    print("="*60)
    
    print(f"agrotechsimapi directory: {agrotechsimapi_dir}")
    
    # Check if directory exists
    if not os.path.isdir(agrotechsimapi_dir):
        print(f"Error: directory {agrotechsimapi_dir} does not exist!")
        return False
    
    # Install the agrotechsimapi package
    print("\nInstalling agrotechsimapi package...")
    
    # Check if pyproject.toml or setup.py exists
    pyproject_file = os.path.join(agrotechsimapi_dir, "pyproject.toml")
    setup_file = os.path.join(agrotechsimapi_dir, "setup.py")
    
    if os.path.isfile(pyproject_file):
        # For modern packaging with pyproject.toml
        if not run_command("pip install -e .", cwd=agrotechsimapi_dir,
                         error_message="Error installing agrotechsimapi"):
            return False
    elif os.path.isfile(setup_file):
        # For legacy setup.py (development mode -e)
        if not run_command("pip install -e .", cwd=agrotechsimapi_dir,
                         error_message="Error installing agrotechsimapi"):
            return False
    else:
        print("Error: pyproject.toml or setup.py not found in agrotechsimapi")
        print(f"Searching in: {agrotechsimapi_dir}")
        print(f"Directory contents:")
        for item in os.listdir(agrotechsimapi_dir):
            print(f"  - {item}")
        return False
    
    print("[ OK ]  agrotechsimapi successfully installed")
    
    return True

def verify_installation_simple():
    """
    Simplified installation verification via subprocess
    
    Returns:
        bool: True if verification successful, False otherwise
    """
    print("\n" + "="*60)
    print("Verifying installation via Python")
    print("="*60)
    
    success = True
    
    # Verify inavmspapi via subprocess (fixed command)
    print("\nVerifying inavmspapi...")
    check_inav = run_command(
        'python -c "import inavmspapi; print(\'[ OK ]  inavmspapi installed, version: {}\'.format(getattr(inavmspapi, \'__version__\', \'version not defined\')))"',
        error_message="Failed to verify inavmspapi"
    )
    
    if not check_inav:
        success = False
    
    # Verify agrotechsimapi via subprocess (fixed command)
    print("\nVerifying agrotechsimapi...")
    check_agro = run_command(
        'python -c "import agrotechsimapi; print(\'[ OK ]  agrotechsimapi installed, version: {}\'.format(getattr(agrotechsimapi, \'__version__\', \'version not defined\')))"',
        error_message="Failed to verify agrotechsimapi"
    )
    
    if not check_agro:
        success = False
    
    return success

def find_agrotechsimapi_dir(base_dir):
    """
    Find directory containing agrotechsimapi
    
    Args:
        base_dir (str): Base directory to search from
        
    Returns:
        str: Path to agrotechsimapi directory or None if not found
    """
    # Option 1: Check TechSimApi subdirectory
    techsimapi_dir = os.path.join(base_dir, "TechSimApi")
    if os.path.isdir(techsimapi_dir):
        print(f"Found TechSimApi directory: {techsimapi_dir}")
        
        # Look for pyproject.toml or setup.py in TechSimApi
        if (os.path.isfile(os.path.join(techsimapi_dir, "pyproject.toml")) or 
            os.path.isfile(os.path.join(techsimapi_dir, "setup.py"))):
            return techsimapi_dir
        
        # Look for subdirectories inside TechSimApi
        for item in os.listdir(techsimapi_dir):
            item_path = os.path.join(techsimapi_dir, item)
            if os.path.isdir(item_path):
                if (os.path.isfile(os.path.join(item_path, "pyproject.toml")) or 
                    os.path.isfile(os.path.join(item_path, "setup.py"))):
                    print(f"Found project subdirectory: {item_path}")
                    return item_path
    
    # Option 2: Check current directory
    if (os.path.isfile(os.path.join(base_dir, "pyproject.toml")) or 
        os.path.isfile(os.path.join(base_dir, "setup.py"))):
        return base_dir
    
    return None

def main():
    """
    Main installation function
    """
    print("="*60)
    print("Installer for agrotechsimapi with local inavmspapi dependency")
    print("="*60)
    
    # Determine base directory
    base_dir = os.getcwd()
    print(f"Current directory: {base_dir}")
    
    try:
        # Find directory containing agrotechsimapi
        print("\nSearching for agrotechsimapi directory...")
        agrotechsimapi_dir = find_agrotechsimapi_dir(base_dir)
        
        if not agrotechsimapi_dir:
            print("\n[ X ]  agrotechsimapi directory not found!")
            print("Make sure:")
            print("1. The script is running from the project directory")
            print("2. The project has pyproject.toml or setup.py")
            sys.exit(1)
        
        print(f"[ OK ]  agrotechsimapi directory found: {agrotechsimapi_dir}")
        
        # 1. Install inavmspapi locally
        if not clone_and_install_inavmspapi(base_dir):
            print("\n[ X ]  Installation aborted: error installing inavmspapi")
            sys.exit(1)
        
        # 2. Verify installation via pip list
        check_pip_list()
        
        # 3. Install the main agrotechsimapi module
        if not install_agrotechsimapi(agrotechsimapi_dir):
            print("\n[ X ]  Installation aborted: error installing agrotechsimapi")
            sys.exit(1)
        
        # 4. Verify again via pip list
        check_pip_list()
        
        # 5. Verify installation via Python
        print("\n" + "="*60)
        print("Final installation verification")
        print("="*60)
        
        if verify_installation_simple():
            print("\n" + "="*60)
            print("[ OK ] Installation successfully completed!")
            print("="*60)
            print(f"\ninavmspapi installed at: {os.path.join(base_dir, 'inavmspapi_local')}")
            print(f"agrotechsimapi installed at: {agrotechsimapi_dir}")
            print("\nAll packages installed and ready to use.")
        else:
            print("\n" + "="*60)
            print("[ ! ]   Installation completed with warnings")
            print("="*60)
            print("\nPackages installed, but there are import issues.")
            print("Try restarting the terminal or checking the PYTHONPATH variable.")
            
    except KeyboardInterrupt:
        print("\n\n[ X ]  Installation interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ X ]  Critical error: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()