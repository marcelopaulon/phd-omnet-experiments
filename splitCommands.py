import os

# Create the splitCommands folder if it doesn't exist
if not os.path.exists("splitCommands"):
    os.makedirs("splitCommands")

# Clean all .txt files inside the splitCommands folder
for file in os.listdir("splitCommands"):
    if file.endswith(".txt"):
        os.remove(os.path.join("splitCommands", file))

# Input file path
input_file = "commands.txt"

lines = None

# Read the input file
with open(input_file, "r") as f:
    lines = f.readlines()

# Prompt the user for the number of split files
num_split_files = int(input("Enter the number of split files: "))

if num_split_files > 12 or num_split_files < 2:
    print('Invalid value. Please specify a number between 2 and 12.')
    exit(-1)

files = []

for i in range(num_split_files):
    temp_file_path = os.path.join("splitCommands", f"split_{i + 1}.txt")
    files.append(open(temp_file_path, "w"))

curFile = 0
buf = []

# Split the lines into multiple files
for line_index in range(len(lines)):

    if line_index > 0 and line_index % 2 == 0:
        files[curFile].writelines(buf)
        buf = []
        curFile += 1
        if curFile >= num_split_files:
            curFile = 0

    buf.append(lines[line_index])


for file in files:
    file.close()

print(f"{num_split_files} split files have been created in the 'splitCommands' folder.")
