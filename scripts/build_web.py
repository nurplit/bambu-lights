Import("env")
import os
import subprocess

def build_web():
    """Build web assets for the project"""
    print("Building web assets...")
    
    # Check if npm is available
    try:
        subprocess.run(["npm", "--version"], check=True, capture_output=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("npm not found. Skipping web build - using pre-built files.")
        return
    
    # Change to web directory
    web_dir = os.path.join(env.get("PROJECT_DIR"), "web")
    if not os.path.exists(web_dir):
        print(f"Web directory not found: {web_dir}")
        return
    
    original_dir = os.getcwd()
    
    try:
        os.chdir(web_dir)
        
        # Install dependencies if needed
        if os.path.exists("package.json") and not os.path.exists("node_modules"):
            print("Installing npm dependencies...")
            subprocess.run(["npm", "install"], check=True)
        
        # Build web assets
        if os.path.exists("gulpfile.js"):
            print("Building with Gulp...")
            subprocess.run(["npx", "gulp"], check=True)
        elif os.path.exists("package.json"):
            print("Building with npm...")
            subprocess.run(["npm", "run", "build"], check=True)
        
        print("Web assets built successfully")
        
    except subprocess.CalledProcessError as e:
        print(f"Web build failed: {e}")
        print("Continuing with existing files...")
    except Exception as e:
        print(f"Unexpected error during web build: {e}")
    finally:
        os.chdir(original_dir)

# Run web build before compilation
build_web()