from bs4 import BeautifulSoup
import sys
import matplotlib.pyplot as plt
import pandas as pd
import pymap3d

def parse_args(argv):
    parsed_args = {}
    for arg in argv:
        if '=' in arg:
            key, value = arg.split('=', 1)
            if key == 'file':
                parsed_args['file'] = str(value)
            elif key == 'polygon':
                parsed_args['polygon'] = str(value)
            elif key == 'origin_lat':
                parsed_args['origin_lat'] = float(value)
            elif key == 'origin_lon':
                parsed_args['origin_lon'] = float(value)
            elif key == 'origin_alt':
                parsed_args['origin_alt'] = float(value)
    
    # Set default values if not provided
    parsed_args.setdefault('file', None)
    parsed_args.setdefault('polygon', None)
    # UCSD Origin
    parsed_args.setdefault('origin_lat', 32.88128944751862)
    parsed_args.setdefault('origin_lon', -117.2353175472838)
    parsed_args.setdefault('origin_alt', 0)
    
    return parsed_args

# Get input arguments
args = parse_args(sys.argv)
origin_lat = args['origin_lat']
origin_lon = args['origin_lon']
origin_alt = args['origin_alt']

with open(args['file'], "r") as file:
    doc = file.read()

soup = BeautifulSoup(doc, "html.parser")

# Find all kmls
kml_names = soup.find_all("name")[1:]
kml_coords = soup.find_all("coordinates")
for index, kml_name in enumerate(kml_names):
    name = kml_name.text.strip()
    if args['polygon'] != name and args['polygon'] is not None:
        continue
    coords = kml_coords[index].text.strip().split(" ")
    coords = [[float(st_coord) for st_coord in coord.split(",")] for coord in coords]
    df = pd.DataFrame(coords, columns = ["x", "y", "z"])
    print(f'alt mean: {df["z"].mean()}')
    df["x"], df["y"], df["z"] = pymap3d.geodetic2enu(df["y"], df["x"], df["z"], origin_lat, origin_lon, origin_alt)
    plt.plot(df["x"].to_numpy(), df["y"].to_numpy())
    plt.show()
    filename = args['file'].replace(".kml", "_") + name + '.csv'
    df.to_csv(filename, index = False, lineterminator="\n")
