# Digital Elevation Map to grid_map

This package contains two main parts:
- a python script to generate the DEM as an image from a .tif file  
- a ROS2 node serving this DEM as a grid_map  

---

## Generate a DEM from input .tif

In `dem_tools/`

**process_tile_to_dem.py**  
- This code is a driver for TileToDEM  
- It reads a .tif file containing the lidar data (converted to EPSG:32632 before)  
- Extracts a bbox inside, and saves it as an image or .XYZ file  
- BBox should be provided in lat/long coordinates (WSG:84, such as points in Google Maps)  
- Or directly in UTM 32N coordinates  
- Useful website to check coordinates: [https://coordinates-converter.com/](https://coordinates-converter.com/)  

This code also saves the DEM image metadata either as a classic YAML file or as a config file for ROS params.  

**Example usage:**
```bash
python3 process_tile_to_dem.py --output_folder <your_ws>/src/lake_grid_map/data \
 --xp lake_side --bbox_latlong 49.101949 6.216037 49.103196 6.217239 \
 --ros_cfg_output <your_ws>/lake_grid_map/config \
 --input_folder <input_folder_of_your_tif> \
 --tile <filename>.tif
```

---

## Serves the generated image as a grid_map

The node `dem2gridmap` is the ROS Wrapper on the class *DEM* implemented in `DEM.h`

- **class DEM**: 
    - loads an image representation of DEM, with associated metadata, and stores as elevation matrix

- **class Dem2Gridmap**: 
    - ROS2 wrapper on DEM 
    - This wrapper convert the DEM into a grid_map msgs, stores it in an elevation layer, and publishes it
    - This wrapper also correct the Z offset from the robot localization to the DEM frame

Example usage:
`ros2 launch lake_grid_map lake_grid_map.launch.py`

Note: the yaml file in `config/` is providing the ros parameters passed to the node in the example launchfile.
**Edit the path to your DEM in this file.**

Options:
- the node can wait for an initial pose of the robot to correct the z offset due to the different input modalities. Enable it by setting      `wait_for_odom: false `
- by default: `use_sim_time: false`


An example DEM is provided : **data/lake_side.png**