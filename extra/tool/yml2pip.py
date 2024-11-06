# import os
# import yaml

# with open("environment.yaml") as file_handle:
#     environment_data = yaml.safe_load(file_handle)

# for dependency in environment_data["dependencies"]:
#     if isinstance(dependency, dict):
#       for lib in dependency['pip']:
#         os.system(f"pip install {lib}")
import yaml

with open("environment.yaml") as file_handle:
    environment_data = yaml.load(file_handle)

with open("requirements.txt", "w") as file_handle:
    for dependency in environment_data["dependencies"]:
        package_name, package_version = dependency.split("=")
        file_handle.write("{} == {}".format(package_name, package_version))