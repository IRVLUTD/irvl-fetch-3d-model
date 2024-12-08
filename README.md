ram-mount model is obtained from [grabcad](https://grabcad.com/library/ram-102u-b-2461-ram-mounts-1)
lenovo-legion is obtained from [grabcad](https://grabcad.com/library/lenovo-legion-laptop-1)
https://3dwarehouse.sketchup.com/model/95491b39-f3bf-48d7-a95e-672f4af7d85a/Lenovo-Legion-7
9105-NETB is obtained from [3dcontentcentral](https://www.3dcontentcentral.com/Model-Preview-Resp.aspx?catalogId=201&id=1239590)


sudo apt-get install freecad


https://imagetostl.com: convert stp file to stl ()
https://imagetostl.com: convert dae file to stl (laptop)

https://imagetostl.com didn't work
https://3dencoder.com/SKP-to-stl


https://3dencoder.com/model-to-stl
Free users can only convert 3 files within a day, and joining paid members has more conversion quotas
Also max 20 mb file allowed for upload for free users

To change the origin of the ati netb model from one end to the 2d centroid (height omitted)

```python
import trimesh

# Load the STL file
mesh = trimesh.load("path_to_your_input_file.stl")

# Get the centroid and create a writable copy
centroid = mesh.centroid.copy()

# Modify the Z-coordinate of the centroid
centroid[2] = 0.0

# Apply the translation to center the mesh along the modified centroid
mesh.apply_translation(-centroid)

# Export the updated STL
mesh.export("path_to_your_output_file.stl")
```
