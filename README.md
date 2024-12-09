This repository is only for 3d modelling of the fetch robot in IRVL.

For the sensor and related ros pkgs and scripts, see [https://github.com/IRVLUTD/fetch_ros_IRVL](https://github.com/IRVLUTD/fetch_ros_IRVL) which is a submodule here so that the 3d modelling can be integrated easily with the 

### Setup
```shell
git clone --recurse-submodules https://github.com/IRVLUTD/irvl-fetch-3d-model
cd irvl-fetch-3d-model
```

Please add the following to the respective folders

- Add the following to [`src/fetch_ros_IRVL/fetch_description/meshes`](src/fetch_ros_IRVL/fetch_description/meshes)
  - [3DModels/ATI-9105-NETB.stl](3DModels/ATI-9105-NETB.stl)
  - [3DModels/fetch_ram_spine_with_mount_and_legion_laptop.stl](3DModels/fetch_ram_spine_with_mount_and_legion_laptop.stl)
- Add all (*.urdf) in [`urdf/`](./urdf) -> [src/fetch_ros_IRVL/fetch_description/robots](src/fetch_ros_IRVL/fetch_description/robots)
  - add a symlink from the desired .urdf to fetch.urdf within the same dir
  - `fetch.urdf` will be read for display purposes

### 🌟 Reference for Visualizing URDF with TF Frames

To visualize the updated URDF with TF frames, **Problem 2** from the following reference has been referred:  
- [📄 CS 6301 Homework 1 (Fall 2024)](https://yuxng.github.io/Courses/CS6301Fall2024/CS_6301_Homework_1_Fall_2024.pdf)  
- [🔗 Course Homepage](https://labs.utdallas.edu/irvl/courses/fall-2024-cs-6301/)  
---

- Once you have the `urdf_tutorial/launch/display_fetch.launch` ready

```shell
# rm if any exists
rm -rf build/ devel/;

# 
catkin_make; source  devel/setup.bash;

# launch the display_fetch.launch
roslaunch urdf_tutorial display_fetch.launch
```

TODO:
Show the video of running rviz with TF frames



Following changes have been performed to the default ATI net box
# ✏️ Adjusting Model Origin (ATI NetB)

<div style="display: flex; justify-content: space-around;">
  <figure style="text-align: center;">
    <img src="./media/ati/raw-ati-netb.png" alt="ati-netb-pre" style="width: 100%;">
    <figcaption>Before Origin Shift</figcaption>
  </figure>
  <figure style="text-align: center;">
    <img src="./media/ati/ati-netb-after-origin-shift.png" alt="ati-netb-post" style="width: 100%;">
    <figcaption>After Origin Shift</figcaption>
  </figure>
</div>



TODO: add images

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



[Blender file](blender/fetch_ram_mount_with_legion.blend):
- The laptop mount consists of
  - 2 x [`ram round plates with ball`](3DModels/parts/ram-mount-round-plate-with-ball/Ram%20Mount.stl)
  - 1 x [`ram spine`](3DModels/parts/RAM-201U-B-spine/ram-201u-b-spine.stl)
  - 1 x [`tray for holding the laptop created manually`](3DModels/parts/ram_tray_based_on_laptop.stl)
  - 1 x [`lenovo legion latop 7`](3DModels/parts/legion_centroid.stl)
- All the previous parts have been assembled to match the shape and pose of the real laptop mount in the fetch robot at IRVL.



# 📦 Model Sources

- **RAM Mount**: The RAM mount model was obtained from [GrabCAD](https://grabcad.com/library/ram-102u-b-2461-ram-mounts-1).  
- **Lenovo Legion Laptop**:  
  - Primary model: [GrabCAD](https://grabcad.com/library/lenovo-legion-laptop-1).  
  - Alternative model: [3D Warehouse](https://3dwarehouse.sketchup.com/model/95491b39-f3bf-48d7-a95e-672f4af7d85a/Lenovo-Legion-7). [Adhere to the license] 
- **9105-NETB**: This model was sourced from [3D Content Central](https://www.3dcontentcentral.com/Model-Preview-Resp.aspx?catalogId=201&id=1239590).  

---

# 🛠️ Tools and Software used for making this

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



License
This project is MIT licensed. But the 3D models used have their own license. Before using this project, please read the respective license files.