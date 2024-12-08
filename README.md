
# 📦 Model Sources

- **RAM Mount**: The RAM mount model was obtained from [GrabCAD](https://grabcad.com/library/ram-102u-b-2461-ram-mounts-1).  
- **Lenovo Legion Laptop**:  
  - Primary model: [GrabCAD](https://grabcad.com/library/lenovo-legion-laptop-1).  
  - Alternative model: [3D Warehouse](https://3dwarehouse.sketchup.com/model/95491b39-f3bf-48d7-a95e-672f4af7d85a/Lenovo-Legion-7). [Adhere to the license] 
- **9105-NETB**: This model was sourced from [3D Content Central](https://www.3dcontentcentral.com/Model-Preview-Resp.aspx?catalogId=201&id=1239590).  

---

# 🛠️ Tools and Software

1. **FreeCAD Installation**:  
   Install FreeCAD for handling 3D models with the command:  
   ```bash
   sudo apt-get install freecad
   ```

2. **File Conversion Tools**:  
   - **[ImageToSTL](https://imagetostl.com)**:  
     - Convert `.stp` files to `.stl`.  
     - Convert `.dae` files (e.g., laptop models) to `.stl`.  
     - ⚠️ *Note*: The tool may fail for some file types.  

   - **[3DEncoder](https://3dencoder.com)**:  
     - Convert `.SKP` or `.model` files to `.stl`.  
     - 🆓 Free users can only convert **3 files per day** and upload files up to **20 MB**. Paid users get increased quotas and file size limits.

---

# ✏️ Adjusting Model Origin (ATI NetB)

To shift the origin of the ATI NetB model from one end to the 2D centroid (ignoring height), use the following Python script with `trimesh`:

```python
import trimesh

# Load the STL file
mesh = trimesh.load("path_to_your_input_file.stl")

# Get the centroid and create a writable copy
centroid = mesh.centroid.copy()

# Set the Z-coordinate of the centroid to 0
centroid[2] = 0.0

# Translate the mesh to align with the modified centroid
mesh.apply_translation(-centroid)

# Export the updated STL file
mesh.export("path_to_your_output_file.stl")
```



### 🌟 Reference for Visualizing URDF with TF Frames

To visualize the updated URDF with TF frames, **Problem 2** from the following reference has been referred:  
- [📄 CS 6301 Homework 1 (Fall 2024)](https://yuxng.github.io/Courses/CS6301Fall2024/CS_6301_Homework_1_Fall_2024.pdf)  
- [🔗 Course Homepage](https://labs.utdallas.edu/irvl/courses/fall-2024-cs-6301/)  

---


License
This project is MIT licensed. But the 3D models used have their own license. Before using this project, please read the respective license files.