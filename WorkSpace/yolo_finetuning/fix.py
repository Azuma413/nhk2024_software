import os

file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'labels'))
# labels下のすべてのファイルを取得
files = os.listdir(file_path)
# file_pathとファイル名を結合してフルパスにする
files = [os.path.join(file_path, file) for file in files]

for file in files:
    # Read the file
    with open(file, 'r') as f:
        lines = f.readlines()

    # Modify the lines
    modified_lines = []
    for line in lines:
        if line.startswith('1'):
            modified_lines.append('2' + line[1:])
        elif line.startswith('2'):
            modified_lines.append('1' + line[1:])
        else:
            modified_lines.append(line)

    # Write the modified lines back to the file
    with open(file, 'w') as f:
        f.writelines(modified_lines)