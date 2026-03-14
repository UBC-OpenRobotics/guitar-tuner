import os

def print_tree(path, prefix=""):
    items = sorted(os.listdir(path))
    
    for i, item in enumerate(items):
        full_path = os.path.join(path, item)
        is_last = (i == len(items) - 1)

        connector = "└── " if is_last else "├── "
        print(prefix + connector + item)

        if os.path.isdir(full_path):
            extension = "    " if is_last else "│   "
            print_tree(full_path, prefix + extension)

if __name__ == "__main__":
    print(".")
    print_tree(os.getcwd())