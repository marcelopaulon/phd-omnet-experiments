import os
import glob

# Define the directory containing the .sca files
directory = os.getcwd()  # Current working directory

# Search for .sca files in the current directory
sca_files = glob.glob("*.sca")

# Define the output directory
output_directory = "csv"

# Create the output directory if it doesn't exist
if not os.path.exists(output_directory):
    os.makedirs(output_directory)

# Iterate through each .sca file and run scavetool
for filename in sca_files:
    input_path = os.path.join(directory, filename)
    output_path = os.path.join(output_directory, filename + ".csv")
    
    command = f"scavetool x {input_path} -o {output_path}"
    
    # Run the scavetool command
    os.system(command)

print("Conversion complete.")
