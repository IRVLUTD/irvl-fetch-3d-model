#----------------------------------------------------------------------------------------------------
# Work done while being at the Intelligent Robotics and Vision Lab at the University of Texas, Dallas
# Please check the licenses of the respective works utilized here before using this script.
# 🖋️ Jishnu Jaykumar Padalunkal (2024).
#----------------------------------------------------------------------------------------------------

import trimesh

# Load the STL file
mesh = trimesh.load_mesh('../3DModels/parts/legion_centroid.stl')

# Get the bounding box of the mesh
bounding_box = mesh.bounding_box
length = bounding_box.extents[0]  # Length along the X axis
width = bounding_box.extents[1]   # Width along the Y axis

# Print the dimensions of the mesh
print(f"Length: {length}, Width: {width}")

# Define tray thickness (height)
tray_thickness = 1.1  # Adjust as needed

# Create a tray based on the bounding box dimensions
# The tray will be a box with the same length and width, and a defined thickness

# Create a mesh for the tray (rectangular base with height = tray_thickness)
tray = trimesh.creation.box(extents=[length, width, tray_thickness])

# Position the tray to align with the bottom of the original model
tray.apply_translation([0, 0, -tray_thickness / 2])  # Move it down by half of tray thickness

# Save the tray model to an STL file
tray.export('../3DModels/parts/ram_tray_based_on_laptop.stl')

print("Tray STL created and saved as 'ram_tray.stl'.")
