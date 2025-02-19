import pathlib
import pandas as pd
import argparse
import numpy as np

YAW_COL = 3


def main(ttl_dir : pathlib.Path):
    for ttl_path in ttl_dir.glob("**/*.csv"):
        first_line = ''
        with open(ttl_path) as f:
            first_line = f.readline().strip()
        df = pd.read_csv(ttl_path, skiprows=1, header = None)
        norm_x = -np.sin(df[YAW_COL])
        norm_y = np.cos(df[YAW_COL])
        df['norm_x'] = norm_x
        df['norm_y'] = norm_y
        with open(ttl_path, 'w') as f:
            f.write(first_line + '\n')
            f.write(df.to_csv(index=False, header=False))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", help="Path to the ttls directory", type=pathlib.Path, default=pathlib.Path("src/common/race_metadata/ttls"))
    args = parser.parse_args()
    main(args.i)