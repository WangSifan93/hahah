import os
import subprocess

def format_code_with_clang(directory, excluded_path_components=None):
    """
    Formats source code files in the specified directory using clang-format.

    Args:
        directory (str): The root directory to search for source files.
        excluded_path_components (list): List of path components to exclude. Files containing
                                       any of these components in their path will be skipped.
    """
    source_extensions = ('.cpp', '.h', '.c', '.hpp', '.cc')
    excluded_path_components = excluded_path_components or []
    directory = os.path.abspath(directory)

    for root, dirs, files in os.walk(directory):
        # Check if current path contains any excluded components
        current_path = os.path.abspath(root)
        if any(component in current_path for component in excluded_path_components):
            continue

        for file in files:
            if file.endswith(source_extensions):
                file_path = os.path.join(root, file)
                
                # Double-check file path doesn't contain excluded components
                if any(component in file_path for component in excluded_path_components):
                    continue
                
                try:
                    # Call clang-format to format the file
                    result = subprocess.run(['clang-format', '-i', '--style=Google', file_path], 
                                          check=True, capture_output=True, text=True)
                    print(f"Formatted: {file_path}")
                except subprocess.CalledProcessError as e:
                    print(f"Error formatting {file_path}: {e}")
                except FileNotFoundError:
                    print("Error: clang-format not found. Please ensure it is installed and in your PATH.")
                    return

if __name__ == "__main__":
    # Get the current script's directory as the base directory
    base_directory = os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
    repos = ['pnc_interface','interactive_speedplan']

    # Specify path components to exclude
    excluded_components = ["deps", "build", "proto", "config"]

    # Format code starting from the base directory
    for rep in repos:
        dirs = os.path.join(base_directory, rep)
        print("Format dir:",dirs)
        format_code_with_clang(dirs, excluded_path_components=excluded_components)