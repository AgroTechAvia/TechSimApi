import os
import time
import argparse
import cv2
import sys
import signal
from agrotechsimapi import SimClient, CaptureType


# Global variable for execution control
running = True


def signal_handler(signum, frame):
    """Signal handler for graceful shutdown"""
    global running
    print(f"\nReceived signal {signum}. Shutting down...")
    running = False


def create_folder_if_not_exists(folder_path):
    """Creates a folder (including parent directories) if it doesn't exist"""
    try:
        # Convert to absolute path if it's relative
        abs_path = os.path.abspath(folder_path)
        
        if not os.path.exists(abs_path):
            os.makedirs(abs_path, exist_ok=True)
            print(f"Created folder: {abs_path}")
        else:
            print(f"Using existing folder: {abs_path}")
        
        return abs_path
    except Exception as e:
        print(f"Error creating folder {folder_path}: {e}")
        return None


def get_next_image_number(folder, prefix):
    """Determines the next image number in the folder"""
    try:
        max_number = -1
        
        # Check if folder exists
        if not os.path.exists(folder):
            return 0
        
        # Check all files in the folder
        for filename in os.listdir(folder):
            if filename.startswith(prefix) and filename.endswith('.png'):
                # Extract number from filename
                try:
                    # Remove prefix and extension
                    number_part = filename[len(prefix):-4]
                    # Try to convert to integer
                    number = int(number_part)
                    if number > max_number:
                        max_number = number
                except ValueError:
                    # Skip if not a number
                    continue
        
        return max_number + 1
    except Exception as e:
        print(f"Error determining image number: {e}")
        return 0


def capture_and_save_image(client, camera_id, capture_type, folder, image_prefix, image_number):
    """Captures and saves an image"""
    try:
        # Check client availability
        if client is None:
            print("Error: client not initialized")
            return False
        
        # Get image from camera
        result = client.get_camera_capture(camera_id=camera_id, type=capture_type)
        
        if result is not None and len(result) != 0:
            # Create filename: prefix + number
            image_name = f"{image_prefix}{image_number}.png"
            file_path = os.path.join(folder, image_name)
            
            # Save image
            success = cv2.imwrite(file_path, result)
            if success:
                print(f"Saved: {image_name}")
                return True
            else:
                print(f"Error saving file: {image_name}")
                return False
        else:
            print("Error: received empty image")
            return False
            
    except Exception as e:
        print(f"Error capturing image: {str(e)}")
        return False


def connect_to_simulator(address="127.0.0.1", port=8080):
    """Connects to simulator with checks"""
    try:
        print(f"Attempting to connect to simulator {address}:{port}...")
        client = SimClient(address=address, port=port)
        print(f"Successfully connected to simulator")
        return client
    except ConnectionRefusedError:
        print(f"Error: could not connect to simulator {address}:{port}")
        print("Make sure the simulator is running and accessible")
        return None
    except Exception as e:
        print(f"Error connecting to simulator: {e}")
        return None


def validate_parameters(frequency, camera_id, capture_type_value):
    """Validates parameter correctness"""
    errors = []
    
    # Frequency validation
    if frequency <= 0:
        errors.append(f"Frequency must be greater than 0 Hz (received: {frequency})")
    elif frequency > 100:  # Limit for very high frequency
        print(f"Warning: very high frequency {frequency} Hz")
    
    # Camera ID validation
    if camera_id not in [0, 1, 2]:
        errors.append(f"Camera ID must be 0, 1, or 2 (received: {camera_id})")
    
    # Capture type validation
    if capture_type_value is None:
        errors.append("Invalid capture type")
    
    return errors


def main():
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Termination signal
    
    parser = argparse.ArgumentParser(description='Capture images from camera')
    
    # Required parameters
    parser.add_argument('--frequency', type=float, required=True, 
                       help='Capture frequency in Hz (frames per second)')
    parser.add_argument('--folder', type=str, required=True,
                       help='Relative or absolute folder path for saving photos')
    parser.add_argument('--prefix', type=str, required=True,
                       help='Prefix for image filenames')
    parser.add_argument('--camera_id', type=int, required=True,
                       help='Camera ID (0-2)')
    parser.add_argument('--capture_type', type=str, required=True,
                       help='Camera image type (color, thermal, depth, spectrum_NIR, etc.)')
    
    try:
        args = parser.parse_args()
    except SystemExit:
        # Argument parsing error
        print("Error in command line parameters")
        return
    
    # Convert string type to CaptureType object
    try:
        # Find corresponding CaptureType
        capture_type_value = None
        for ct in CaptureType:
            if ct.name.lower() == args.capture_type.lower():
                capture_type_value = ct
                break
        
        if capture_type_value is None:
            # If not found by name, try by value
            try:
                # Convert string to number for direct value specification
                type_int = int(args.capture_type)
                capture_type_value = CaptureType(type_int)
            except:
                # Display all available types
                print("Available capture types:")
                for ct in CaptureType:
                    print(f"  {ct.name} = {ct.value}")
                print(f"\nError: unknown capture type '{args.capture_type}'")
                return
                
    except Exception as e:
        print(f"Error determining capture type: {e}")
        return
    
    # Validate parameters
    validation_errors = validate_parameters(args.frequency, args.camera_id, capture_type_value)
    if validation_errors:
        print("Parameter errors:")
        for error in validation_errors:
            print(f"  - {error}")
        return
    
    # Calculate target period (time between frames)
    target_period = 1.0 / args.frequency  # in seconds
    
    # Create folder for saving (supports relative paths)
    folder_path = create_folder_if_not_exists(args.folder)
    if folder_path is None:
        print("Failed to create save folder")
        return
    
    # Display folder information
    print(f"Working directory: {os.getcwd()}")
    print(f"Save folder: {folder_path}")
    
    # Determine number for first image
    start_number = get_next_image_number(folder_path, args.prefix)
    print(f"Starting save from number: {start_number}")
    
    # Connect to simulator
    client = connect_to_simulator()
    if client is None:
        print("Failed to connect to simulator. Exiting.")
        return
    
    print(f"\nStarting image capture:")
    print(f"  Frequency: {args.frequency} Hz (period: {target_period:.3f} sec)")
    print(f"  Folder: {folder_path}")
    print(f"  Prefix: {args.prefix}")
    print(f"  Camera ID: {args.camera_id}")
    print(f"  Image type: {capture_type_value.name}")
    print("Press Ctrl+C to stop\n")
    
    current_number = start_number
    frame_count = 0
    start_time = time.time()
    error_count = 0
    max_errors = 10  # Maximum consecutive errors
    
    try:
        while running and error_count < max_errors:
            # Record cycle start time
            cycle_start_time = time.time()
            
            # Capture and save image
            success = capture_and_save_image(
                client=client,
                camera_id=args.camera_id,
                capture_type=capture_type_value,
                folder=folder_path,
                image_prefix=args.prefix,
                image_number=current_number
            )
            
            if success:
                current_number += 1
                frame_count += 1
                error_count = 0  # Reset error counter on success
            else:
                error_count += 1
                print(f"Capture error {error_count}/{max_errors}")
                if error_count >= max_errors:
                    print("Maximum error count reached. Exiting.")
                    break
            
            # Check running flag after each operation
            if not running:
                break
            
            # Calculate cycle execution time
            cycle_time = time.time() - cycle_start_time
            
            # Calculate wait time to maintain target frequency
            wait_time = target_period - cycle_time
            
            if wait_time > 0:
                # Wait with running flag check
                wait_start = time.time()
                while time.time() - wait_start < wait_time and running:
                    time.sleep(0.001)  # Short intervals for quick response
            else:
                # Cycle took longer than target period
                actual_frequency = 1.0 / cycle_time if cycle_time > 0 else 0
                print(f"Warning: cannot maintain {args.frequency} Hz. "
                      f"Current frequency: {actual_frequency:.2f} Hz")
            
            # Periodically display statistics
            if frame_count % 10 == 0 and frame_count > 0:
                elapsed_time = time.time() - start_time
                if elapsed_time > 0:
                    actual_fps = frame_count / elapsed_time
                    print(f"Statistics: saved {frame_count} frames in {elapsed_time:.1f} sec. "
                          f"Actual frequency: {actual_fps:.2f} Hz")
            
            # Check available disk space
            try:
                stat = os.statvfs(folder_path) if hasattr(os, 'statvfs') else None
                if stat and stat.f_bavail * stat.f_frsize < 100 * 1024 * 1024:  # 100 MB
                    print("Warning: less than 100 MB free space remaining")
            except:
                pass  # Ignore disk check errors
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user (KeyboardInterrupt)")
    except Exception as e:
        print(f"\nUnexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nShutting down...")
        
        elapsed_time = time.time() - start_time
        saved_count = current_number - start_number
        
        if elapsed_time > 0 and saved_count > 0:
            actual_fps = saved_count / elapsed_time
        else:
            actual_fps = 0
            
        print(f"\nFinal statistics:")
        print(f"  Images saved: {saved_count}")
        print(f"  Total time: {elapsed_time:.1f} sec")
        print(f"  Actual frequency: {actual_fps:.2f} Hz")
        print(f"  Target frequency: {args.frequency} Hz")
        print(f"  Last saved number: {current_number - 1 if saved_count > 0 else 'none'}")
        print(f"  Save location: {folder_path}")
        
        # Close connection if possible
        try:
            if hasattr(client, 'close'):
                client.close()
                print("Simulator connection closed")
        except:
            pass
        
        print("Program execution completed")


if __name__ == "__main__":
    main()