
import numpy as np
import rasterio
import cv2
import os
import yaml
from rasterio.windows import Window
from rasterio.plot import reshape_as_image
import geopandas as gpd
from shapely.geometry import Point
from scipy.interpolate import griddata
from math import ceil, floor, isclose


# This class convert a tile containing aerial lidar data into a DEM, from a requested box and resolution
# The DEM can be saved as an image and/or a .XYZ file
# Additionally, a colored image corresponding to the aerial view of the same place can be also extracted and saved (if color_path is provided)
class TileToDEM:
    def __init__(self, tile_path, color_path=""):
        self.tile = rasterio.open(tile_path)
        self.box = []
        self.resolution = 0.0
        self.grid = []
        self.xyz_fname = ""
        self.dem_fname = ""
        self.yaml_fname = ""
        self.do_color = False
        if (len(color_path)>0):
            self.do_color = True
            self.color_tile = rasterio.open(color_path)
    
    # Accessors
    def get_tile_box(self):
        return self.tile.bounds
    
    def get_selected_box(self):
        return self.box
    
    def get_interpolated_grid(self):
        return self.grid
    
    def get_xyz_fname(self):
        return self.xyz_fname
    
    def get_dem_fname(self):
        return self.dem_fname

    def get_yaml_fname(self):
        return self.yaml_fname

    # Get a rounded bbox (in meters), to a given box. 
    # Returned bbox is included in given box.
    def round_box(self, box):
        nbox = [.0,.0,.0,.0]
        nbox[0] = float(ceil(box[0]))
        nbox[1] = float(ceil(box[1]))
        nbox[2] = float(floor(box[2]))
        nbox[3] = float(floor(box[3]))
        return nbox

    # Gets a Window object based on the coordinatinates of a bounding box
    def get_window_in_tile_from_box(self, tile, single_box):
        tl = tile.index(single_box[0],single_box[3])#topleft
        bl = tile.index(single_box[0],single_box[1])#bottomleft
        br = tile.index(single_box[2],single_box[1])#bottomright
        tr = tile.index(single_box[2],single_box[3])#topright
        xmin = tl[1]
        xmax = tr[1]
        ymax = br[0]
        ymin = tr[0]
        img_width = xmax - xmin
        img_height = ymax - ymin
        cropw = Window(xmin, ymin, img_width, img_height)
        return cropw

    # Check that a given geometry (bbox) is included in the tile
    def is_in_tile(self, geom, tile):
        if hasattr(geom, "bounds"):
            left, bottom, right, top = geom.bounds
        else:
            left, bottom, right, top = geom  # already a tuple
        
        if (left < tile.bounds.left):
            return False
        if (right > tile.bounds.right):
            return False
        if (bottom < tile.bounds.bottom):
            return False
        if (top > tile.bounds.top):
            return False
        return True

    # Extract the BBOX containing a shape (polygon)
    # Check that the bbox is included in the tile
    # bbox in UTM(coordinates rounded to meters)
    # The polygon is also expected in UTM 32N (EPSG:32632)
    def set_box_from_polygon(self, shape):
        geom = shape.iloc[0].geometry
        if not self.is_in_tile(geom, self.tile):
            raise ValueError("Provided polygon is not included in tile. " \
                "Are the CRS consistent? Tile bounds: %s, Geom bounds %s"\
                %(str(self.tile.bounds), str(geom.bounds)))
        self.box = self.round_box(geom.bounds)

    # Extract the BBOX from min/max corners in lat, long
    # Check that the bbox is included in the tile
    # bbox in UTM(coordinates rounded to meters)
    # input coordinates are expected in lat, long (EPSG:4326)
    def set_box_from_latlongcorners(self, min_lat, min_lon, max_lat, max_lon):
        coords = [(min_lat, min_lon), (max_lat, max_lon)]
        # create GeoDataFrame in WGS84 (EPSG:4326)
        gdf = gpd.GeoDataFrame(
            # Beware: Point is (lon, lat) and not (lat, lon)...
            geometry=[Point(lon, lat) for lat, lon in coords],
            crs="EPSG:4326"
        )
        # convert to UTM zone 32N (EPSG:32632)
        gdf_utm = gdf.to_crs(epsg=32632)
        geom_bounds = gdf_utm.total_bounds
        if not self.is_in_tile(geom_bounds, self.tile):
            raise ValueError("Provided polygon is not included in tile. " \
                "Are the CRS consistent? Tile bounds: %s, Geom bounds %s"\
                %(str(self.tile.bounds), str(geom_bounds)))
        self.box = self.round_box(geom_bounds)

    # Extract the BBOX from min/max corners in utm
    # Check that the bbox is included in the tile
    # bbox in UTM(coordinates rounded to meters)
    # input coordinates are expected in utm (EPSG:32632)
    def set_box_from_utmcorners(self, min_x, min_y, max_x, max_y):
        coords = [(min_x, min_y), (max_x, max_y)]
        # create GeoDataFrame in UTM 32N (EPSG:32632)
        gdf = gpd.GeoDataFrame(
            geometry=[Point(lon, lat) for lon, lat in coords],
            crs="EPSG:32632"
        )
        # convert to UTM zone 32N (EPSG:32632)
        geom_bounds = gdf.total_bounds
        if not self.is_in_tile(geom_bounds, self.tile):
            raise ValueError("Provided polygon is not included in tile. " \
                "Are the CRS consistent? Tile bounds: %s, Geom bounds %s"\
                %(str(self.tile.bounds), str(geom_bounds)))
        self.box = self.round_box(geom_bounds)


    # Check that the resolution is valid
    # if grid size divided by resolution yield an integer, returns True
    def is_valid_resolution(self, res):
        size_x = self.box[2]-self.box[0]
        size_y = self.box[3]-self.box[1]
        qx = size_x / res
        qy = size_y / res
        return (isclose(qx, round(qx)) and isclose(qy, round(qy)))


    # Set_the grid:
    # Extract from the tile the Window corresponding to the bbox
    # Interpolate a grid on the required resolution
    def set_grid(self, resolution):
        if not self.is_valid_resolution(resolution):
           raise ValueError("Requested resolution is invalid. Make sure grid_size can be divided by resolution. \
                            Grid size is : (%d, %d)"%(self.box[2]-self.box[0],self.box[3]-self.box[1]))
        self.resolution = resolution
        cropw = self.get_window_in_tile_from_box(self.tile, self.box)
        area = self.tile.read(1, window=cropw)
        self.interpolate_grid(area)
        if self.do_color:
            self.crop_color_img(cropw)

    # Extract in the color_tile the Window corresping to the bbox
    # Save as an image        
    def crop_color_img(self, cropw):
        # Initialize the color with grey
        if not self.is_in_tile(self.box, self.color_tile):
            raise NotImplementedError("Current bbox in DEM is outside the bounds of the colored image, not implemented")
            #TODO: initialize the image with gray, and color only the pixels inside the bbox of the image
        cropw = self.get_window_in_tile_from_box(self.color_tile, self.box)
        raster_color = self.color_tile.read(window=cropw)
        rgb = reshape_as_image(raster_color)
        self.color_img =  rgb[:,:,[2,1,0]] # Convert to BGR

    # Interpolate the selected area on a regular grid sampled on the selected resolution
    def interpolate_grid(self, area):
        # First, we prepare the data for interpolation
        # points is the list of points corresponding to values in area
        # values is the 1D array of values in area
        nx_area = area.shape[1]
        ny_area = area.shape[0]
        points = []
        for j in range(ny_area):
            for i in range(nx_area):
                points.append((i,j))
        values = area.flatten()
        # Next, we build the interpolation grid, based on the resolution
        width_m = int(self.box[3]-self.box[1])
        height_m = int(self.box[2]-self.box[0])
        nx = int(height_m/self.resolution)
        ny = int(width_m/self.resolution)
        grid_x, grid_y = np.mgrid[0:nx_area-1:(nx)*1j,0:ny_area-1:(ny)*1j]
        # Finally, we interpolate on the selected grid
        grid_z = griddata(points, values, (grid_x, grid_y), method="cubic")
        # And we take the transpose to get back the grid_z as expected
        self.grid = grid_z.T
    
    # Interpolate the selected area on a regular grid sampled on the selected resolution
    def make_color_grid(self, area):
        # First, we prepare the data for interpolation
        # points is the list of points corresponding to values in area
        # values is the 1D array of values in area
        nx_area = area.shape[1]
        ny_area = area.shape[0]
        points = []
        for j in range(ny_area):
            for i in range(nx_area):
                points.append((i,j))
        values = area.flatten()
        # Next, we build the interpolation grid, based on the resolution
        width_m = int(self.box[3]-self.box[1])
        height_m = int(self.box[2]-self.box[0])
        nx = int(height_m/self.resolution)
        ny = int(width_m/self.resolution)
        grid_x, grid_y = np.mgrid[0:nx_area-1:(nx)*1j,0:ny_area-1:(ny)*1j]
        # Finally, we interpolate on the selected grid
        grid_z = griddata(points, values, (grid_x, grid_y), method="cubic")
        # And we take the transpose to get back the grid_z as expected
        self.colored_grid = grid_z.T

    # Save all the generated outputs
    # xyz, dem image, colored image, yaml metadata
    def save_outputs(self, outputbasename, ros=True, create_dirs = True):
            output_folder = os.path.dirname(outputbasename)
            if not os.path.exists(output_folder):
                if create_dirs:
                    os.makedirs(output_folder)
                    print("Created output directory: ", output_folder)
                else:
                    raise FileNotFoundError("Output Directory does not exists: %s"%(output_folder))
            self.save_as_xyz(outputbasename)
            self.save_as_img(outputbasename)
            self.save_metadata(outputbasename, ros)
            if self.do_color:
                self.save_color_img(outputbasename)

    # Save the color_img
    def save_color_img(self, outputbasename):
        fname = outputbasename+"_color_.png"
        cv2.imwrite(fname, self.color_img)
            
    # Save the interpolated grid into XYZ coordinates
    def save_as_xyz(self, outputbasename):
        xyz = []
        # We need to rotate it and construct xyz this way to have the coordinates expressed in x,y with origin bottom left
        gridr = np.rot90(self.grid, k=-1)
        init_z = self.grid[0,0]
        for i in range(gridr.shape[0]):
            for j in range(gridr.shape[1]):
                xyz.append((i*self.resolution, j*self.resolution, gridr[i,j]-init_z))
        self.xyz_fname =  outputbasename+".XYZ"
        np.savetxt(self.xyz_fname, xyz, delimiter=' ',fmt='%02f')

    # Save the grid as a grayscale image (min max norm on z values)
    def save_as_img(self, outputbasename):
        img = np.zeros_like(self.grid)
        img = img.copy()
        img = cv2.normalize(self.grid, img, 0, 65535, cv2.NORM_MINMAX)
        img = img.astype(np.uint16)
        ## Check that we didn't miss a step:
        ## box and image size must match
        diff_x = (self.box[2]-self.box[0])- img.shape[1]*self.resolution
        diff_y = (self.box[3]-self.box[1])- img.shape[0]*self.resolution
        if not (isclose(diff_x, 0) and isclose(diff_y, 0)):
           raise ValueError("Requested resolution is invalid. Make sure grid_size can be divided by resolution. \
                            Grid size is : (%d, %d)"%(self.box[2]-self.box[0],self.box[3]-self.box[1]))

        self.dem_fname =  outputbasename+".png"
        cv2.imwrite(self.dem_fname, img)

    # Save the meta data, either directly or in ROS yaml config file
    # If ros is True, the file is saved in ../config if it exists, else in the output folder
    def save_metadata(self, outputbasename, ros=True):
        min_z = float(np.min(self.grid))
        max_z = float(np.max(self.grid))
        z0 = float(self.grid[-1,0])
        ztl = float(self.grid[0,0])
        
        base = {
            "z_topleft": ztl,
            "z0": z0,
            "z0-coords": "xmin,ymin",
            "zmin": min_z,
            "zmax": max_z,
            "resolution": self.resolution,
            "xmin": self.box[0],
            "ymin": self.box[1],
            "xmax": self.box[2],
            "ymax": self.box[3],
            "crs": "UTM zone 32U (EPSG:32632)",
        }

        if ros:
            metadata = {
                "/**": {
                    "ros__parameters": {
                        "use_sim_time": True,
                        "wait_for_odom": True,
                        "frame_id": "utm_local_symphonie",
                        "parent_frame_id": "utm_local_gte",
                        "dem_file": self.dem_fname,
                        **base,
                    }
                }
            }
        else:
            metadata = base
        
        # Set custom yaml dumper to floating point precision :.2f
        def float_representer(dumper, value):
            return dumper.represent_scalar("tag:yaml.org,2002:float", f"{value:.2f}")

        yaml.SafeDumper.add_representer(float, float_representer)

        self.yaml_fname = outputbasename + "_metadata.yaml"
        with open(self.yaml_fname, "w") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)
