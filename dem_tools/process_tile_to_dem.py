from tile_to_dem import TileToDEM
import geopandas as gpd
import os
import argparse

'''
 This code is a driver for TileToDEM
 It reads a .tif file containing the lidar data (converted to EPSG:32632 before)
 Extract a bbox inside, and save it as an image or .XYZ file
 BBox should be provided in lat, long coordinates (WSG:84, such as points in Google Maps)
 Or directly in UTM 32N Coordinates
 Useful website to check coordinates: https://coordinates-converter.com/

 This code also save the DEM image metadata either as a classic yaml file or as a config file for ROS params

Example usage:
 python3 process_tile_to_dem.py --output_folder <your_ws>/src/lake_grid_map/data \
 --xp lake_side --bbox_latlong 49.101949 6.216037 49.103196 6.217239 \
 --ros_cfg_output <your_ws>/lake_grid_map/config \
 --input_folder <input_folder_of_your_tif> \
 --tile <filename>.tif
'''

if __name__ == "__main__":

    ## Parse args
    parser = argparse.ArgumentParser(description="DEM preprocessing driver")
    parser.add_argument("--tile", required=True,
                        help="Digital Elevation Map Tile file (GeoTIFF) to process")
    parser.add_argument("--color_tile", required=False, default="",
                        help="Aerial View (color) Tile file (GeoTIFF) to process")
    parser.add_argument("--xp", required=True,
                        help="Experiment name")
    parser.add_argument("--input_folder", required=True,
                        help="Input folder containing DEM data and polygons")
    parser.add_argument("--output_folder", required=False, default=".",
                        help="Output folder for results")
    parser.add_argument("--ros_cfg_output", required=False, default="",
                        help="Output folder for ROS config yaml file with metadata as params")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--polygon", type=str,
                       help="Polygon shapefile (.shp) to crop DEM. \
                        Coordinates are expected in UTM 32U: EPSG:32632 ")
    group.add_argument("--bbox_latlong", nargs=4, type=float,
                       metavar=("MIN_LAT", "MIN_LON", "MAX_LAT", "MAX_LON"),
                       help="Bounding box coordinates (lat/long)")
    group.add_argument("--bbox_utm", nargs=4, type=float,
                       metavar=("MIN_X", "MIN_Y", "MAX_X", "MAX_Y"),
                       help="Bounding box coordinates in UTM 32U")

    args = parser.parse_args()

    ## Load tile
    tile_path = os.path.join(args.input_folder, args.tile)
    # If aerial view is provided, we load it, else we load the DEM only
    if len(args.color_tile)>0:
        color_path = os.path.join(args.input_folder, args.color_tile)
        tile_to_dem = TileToDEM(tile_path, color_path)
    else:
        tile_to_dem = TileToDEM(tile_path)
    print("Tile loaded. Bounds are: ", tile_to_dem.get_tile_box())
    print(args.bbox_latlong)
    ## Get the bbox to extract
    ## The bbox is converted to utm from either: polygon / min max latlong / min max utm
    if args.polygon:
        shape = gpd.read_file(os.path.join(args.input_folder, args.polygon))
        tile_to_dem.set_box_from_polygon(shape)
    elif args.bbox_latlong:
        min_lat, min_lon, max_lat, max_lon = args.bbox_latlong
        tile_to_dem.set_box_from_latlongcorners(min_lat, min_lon, max_lat, max_lon)
    elif args.bbox_utm:
        min_x, min_y, max_x, max_y = args.bbox_utm
        tile_to_dem.set_box_from_utmcorners(min_x, min_y, max_x, max_y)
    else:
        raise NotImplementedError("Else statement not implemented. You should provide a polygon or min max corners either in lat long or UTM 32N.")

    print("Selected bbox to extract: ", tile_to_dem.get_selected_box())

    # From the tile, interpolate data on a 2d grid on the required resolution
    tile_to_dem.set_grid(resolution=0.1)

    # Save outputs
    output_basename = os.path.join(args.output_folder, str(args.xp))

    tile_to_dem.save_outputs(output_basename) 
    print("DEM outputs saved in :", args.output_folder)
    
    # If ros_cfg_output provided, save metadata as ROS yaml config file
    # Else, classic yaml has been saved with previous call to save_outputs
    if (len(args.ros_cfg_output)>0):
        cfg_basename = os.path.join(args.ros_cfg_output, str(args.xp))
        yaml_dir = os.path.dirname(cfg_basename)
        if not os.path.exists(yaml_dir):
            os.makedirs(yaml_dir)
            print("Maked directory: ", yaml_dir)
        tile_to_dem.save_metadata(cfg_basename, ros=True)
        print("ROS yaml saved as ", tile_to_dem.get_yaml_fname())



