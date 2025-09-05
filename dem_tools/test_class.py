import os
from tile_to_dem import TileToDEM

tile_path = "/home/saravecchia/data/aerial_moselle/mnt_1m_to_utm.tif"
color_path = "/home/saravecchia/data/aerial_moselle//ortho_5cm_2024_utm_lake.tif"
bbox = {
    "xmin": 296802.00,
    "ymin": 5442522.00,
    "xmax": 296893.00,
    "ymax": 5442657.00}
output_folder = "tmp"
xp = "test"
tdm = TileToDEM(tile_path, color_path)
tdm.set_box_from_utmcorners(bbox["xmin"], bbox["ymin"], bbox["xmax"], bbox["ymax"])
tdm.set_grid(resolution=0.1)
output_basename = os.path.join(output_folder, str(xp))
tdm.save_outputs(output_basename, ros=False)