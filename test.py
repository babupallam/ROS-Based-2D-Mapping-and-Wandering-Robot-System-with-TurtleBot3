import os

# Define the folder structure
structure = {
    "ROS-Mapping-Wandering-Robot": {
        "docs": [
            "installation_guide.md",
            "setup_instructions.md",
            "usage_guide.md",
            "experiments_and_results.md"
        ],
        "scripts": {
            "mapping": [
                "mapperv1.py",
                "mapperv2.py",
                "mapperv3.py",
                "mapperv4.py"
            ],
            "wandering_robot": [
                "wandering_robot.py"
            ]
        },
        "launch": [
            "turtlebot3_world.launch",
            "rviz_visualization.launch",
            "wandering_robot.launch",
            "mapping.launch"
        ],
        "config": [
            "rviz_config.rviz",
            "turtlebot3_config.yaml"
        ],
        "data": {
            "maps": [
                "mapperv1_map.png",
                "mapperv2_map.png",
                "mapperv3_map.png",
                "mapperv4_map.png"
            ],
            "logs": [],
            "performance_metrics.csv": None
        },
        "supporting_documents": {
            "screenshots": [
                "mapperv1_demo.png",
                "mapperv2_demo.png",
                "mapperv3_demo.png",
                "mapperv4_demo.png"
            ],
            "assignment_guidelines.pdf": None,
            "demo_steps.pdf": None,
            "notes.md": None
        },
        "": ["README.md", "LICENSE", ".gitignore"]
    }
}

# Create the folder structure
def create_structure(base_path, structure):
    for name, content in structure.items():
        folder_path = os.path.join(base_path, name)
        if isinstance(content, dict):  # Subdirectory
            os.makedirs(folder_path, exist_ok=True)
            create_structure(folder_path, content)
        elif isinstance(content, list):  # Files in a directory
            os.makedirs(folder_path, exist_ok=True)
            for file in content:
                open(os.path.join(folder_path, file), 'a').close()
        elif content is None:  # Single file
            open(folder_path, 'a').close()

# Run the script to create the structure
base_directory = "ROS-Mapping-Wandering-Robot"
create_structure("", structure)

print(f"The project structure has been created under the folder '{base_directory}'.")
