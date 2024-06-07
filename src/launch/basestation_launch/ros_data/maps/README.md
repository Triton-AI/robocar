## Map Saving Directory

Welcome to __Map Saving Directory__, where we save our map data.

#### Rules:
- Create a new folder under `${WORKSPACE}/src/launch/basestation_launch/ros_data/maps/${CATEGORY}/`, named it `${your_map_name}_${MMDD}/`

- `${WORKSPACE}` is the absolute directory of your workspace. `${CATEGORY}` could be either `custom`,`icra`,`iros`,`lab_test`, or `self_test` (whose subfolders should be in `.gitignore`). Do not create a new CATEGORY!

- name your map in this format `${your_map_name}_${MMDD}`, eg: `lab_halway_0505`.

- put your map data (`*.pgm`/`*.png`and `*.yaml`) under the map folder you created

- `self_test` is a category for you to store your own map data if you wish not to push back to the remote repository.

## Note: `(IMPORTANT)`
You should not modify the `basestation_launch` pkg on any branch without any permission.

If you really need to modify anything in the pkg and push it back to branch, please contact the maintainer and create a Pull Request.