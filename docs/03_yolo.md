YOLO
==============
Train an object identification model with a pretrained YOLO v11 model.

The trained YOLO model should be able to identify traffic signs at a T-junction.

# 1&emsp;View the Camera Feed

1. Start VSCode and connect to the robot (See [02_pc.md](02_pc.md)).

2. Bringup the robot sensors, motor controller and camera with the robot's `bringup.sh` script, on a VSCode terminal `RobotTerminalA`.
    1. On the VSCode window that is connected to the robot, type `` CTRL+` `` to open a new terminal.
    
    2. Let this VSCode terminal be `RobotTerminalA`.

    3. In the terminal, change directory to the `tb3_yolo` directory, and start up the sensors, motor controller and camera, using `./bringup.sh`:

    ```bash
    cd ~/tb3_yolo
    ./bringup.sh
    ```
    Note that `./bringup.sh` can be run directly without `cd` as long as the terminal is in the `tb3_yolo` directory.
    

4. View the camera feed with the PC's `view_cam` script, on a terminal `PCTerminalA`.
    1. Press `CTRL+ALT+T` to open a new terminal on the PC.

    2. Let this terminal be `PCTerminalA`.

    3. Suppose your robot's username is `ece83` and your team number is `83`. Then the value after `ROS_DOMAIN_ID` in the following command should be `83`. Replace the value with your team number accordingly.
    
    4. In the terminal, run the following command by replacing the `ROS_DOMAIN_ID` value with your team number:
        ```bash
        cd ~/tb3_yolo_pc
        ROS_DOMAIN_ID=83 ./view_cam.sh
        ```
        Ensure that there are **no spaces** before and after `=`. Note that `ROS_DOMAIN_ID=83 ./view_cam.sh` can be run directly without `cd` as long as the terminal is in the `tb3_yolo_pc` directory.

    5. Wait for a few seconds for the feed to show up.

5. When complete, press `CTRL+C` on **both** terminals to stop the programs.

# 2&emsp;Capturing Images

# 2.1&emsp;Default Capture
1. Run `bringup.sh` in `RobotTerminalA`. Open a new terminal on VSCode with `` CTRL+` `` if needed.

2. Run `view_cam.sh` in `PCTerminalA`. Open a new terminal with `CTRL+ALT+T` if needed.

3. Run the `capture.sh` script to start collecting images on a terminal `PCTerminalB`.
    1. Open a new terminal with `CTRL+ALT+T`.

    2. Let this terminal be `PCTerminalB`.

    3. In the terminal, run the `capture.sh` script. Replace the value in `ROS_DOMAIN_ID` with the team number.
    ```bash
    cd ~/tb3_yolo_pc
    ROS_DOMAIN_ID=83 ./capture.sh
    ```

4. Try to take multiple images by pressing `SPACE` in `PCTerminalB` where `capture.sh` is run.

5. When complete, press `CTRL+C` on all three terminals.

6. The pictures can be found in the `images` directory at the path `~/tb3_yolo_pc/train_yolo/datasets/traffic_signs/images/`. 
Open `Files` in Ubuntu and type the directory path into the address bar to navigate to the folder.

# 2.2&emsp;Overwriting Some Images

The images in the `images` directory are always overwritten every time `capture.sh` is run. By default, the images are stored as `img001.jpg`, `img002.jpg` etc. everytime `capture.sh` is run. 

To save or overwrite from say `img010.jpg` onwards:
1. Run `bringup.sh` in `RobotTerminalA`. Open a new terminal on VSCode with `` CTRL+` `` if needed.

2. Run `view_cam.sh` in `PCTerminalA`. Open a new terminal with `CTRL+ALT+T` if needed.

3. On `PCTerminalB` (open a new terminal with `CTRL+ALT+T` if needed), by replacing the value of `ROS_DOMAIN_ID`, run:
    ```bash
    ROS_DOMAIN_ID=83 ./capture.sh img 9
    ```

4. `CTRL+C` on all terminals when complete.

# 3&emsp;Preparing The Dataset

# 3.1&emsp;11 Images, 3 Image Classes
11 different images of traffic signs are provided. 7 of the signs are directional (left or right), and 4 of the signs are QR codes.

A total of 3 image classes are required. Take note of the class ID (an integer) associated with each class:

| ID | Label / Class | Number of Signs | Description |
| - | - | - | - |
| `0` | `left` | 8 | Rotate the directional signs so the arrows point left. |
| `1` | `right` | 8 | Rotate the directional signs so the arrows point right. |
| `2` | `qr` | 4 | `dance left`, `dance right`, `spin left`, `spin right`. |

The task is to capture multiple images of each sign to form a dataset in which YOLO would train on.

# 3.2&emsp;Setting up the Capture

To capture the images, first place a sign on a cell wall, and then place the robot about a cell (~0.3 m) away from the sign. 

# 3.3&emsp;The Methodology

For the trained model to be robust, images of the same sign must be captured slightly differently from each other.

To capture different images of the same sign,
- Point the robot at a different angle so that the sign does not appear in the center of the captured image.
- Move the robot between 0.25m to 0.35m away from the sign so the sign is not always the same size in the captured image.
- Move the sign out of the image frame slightly so that at least half but not all of the sign can be seen.
- The sign can be rotated slightly so that the sign is not always upright in the captured image.

# 3.4&emsp;Begin Capturing

1. Run `bringup.sh` in `RobotTerminalA`. Open a new terminal on VSCode with `` CTRL+` `` if needed.

2. Run `view_cam.sh` in `PCTerminalA`. Open a new terminal with `CTRL+ALT+T` if needed.

3. Run `capture.sh` in `PCTerminalB`. Open a new terminal with `CTRL+ALT+T` if needed.

4. Capture the images. Follow the method from the previous section to capture different images of the signs.
    1. Capture 10 different images for every left directional sign, to a total of 70 different images. Ensure that the arrows of the directional signs are pointing left.
    2. Capture 10 different images for every right directional sign, to a total of 70 different images. Ensure that the arrows of the directional signs are pointing right.
    3. Capture 20 different images for each QR code, to a total of 80 different images.

5. Press `CTRL+C` on all three terminals when complete.

As mentioned, the images can be found in the `~/tb3_yolo_pc/train_yolo/datasets/traffic_signs/images/` directory. 


# 4&emsp;Labelling the Dataset

1. Open a web browser and go to https://www.makesense.ai/. 

2. Click `Get Started` on the bottom right.

3. Upload all the images from the `images` directory. 

4. Click `Object Detection`.

5. Create a label for each of the image class. As mentioned:
    - The `left` class has an ID of `0`.
    - The `right` class has an ID of `1`.
    - The `qr` class has an ID of `2`.

6. Click `Start project`.

7. Begin to spend about an hour annotating, by drawing bounding boxes across the object in every image. Ensure that the correct label and ID is chosen.

8. Once the labelling is complete, click `Actions` on the top left, and click `Export Annotations`.

9. Select `A .zip package containing files in YOLO format`.

10. Click `Export`.

11. Download the `.zip` file, and extract all the `.txt` annotation files into the `~/train_yolo/datasets/traffic_signs/labels/` directory which is adjacent to the `images` directory.

12. Once extracted, double check that the `.txt` files have the correct IDs. Each `.txt` file of the corresponding image will have the following format:

    ```
    ID  x_center  y_center  width  height
    ```

    The first number, `ID` should correspond to the correct class ID. The other coordinates are normalized to 0 and 1 and represent the bounding boxes that were annotated on to the images.


# 5&emsp;Begin Training YOLO Model

1. With the images and labels in place, edit the `.yaml` configuration file for training the dataset.

    1. Open the `~/train_yolo/datasets.yaml` file from `PCTerminalA`:
        ```bash
        code ~/tb3_yolo_pc/train_yolo/datasets.yaml
        ```

    2. Look for `names` in the `yaml` file. The class labels and integer class IDs should correspond to the ones that were used during the annotation:

    ```yaml
    # ...

    # Classes
    names:
        0: left
        1: right
        2: qr
    ```

    3. Save and close with `CTRL+S` and `CTRL+SHIFT+W`.

2. in `PCTerminalA`, run the following script to train the model from pre-trained weights.

    ```bash
    cd ~/tb3_yolo_pc/train_yolo
    python3 train_yolo.py
    ```

    Wait for about an hour. Go have lunch!

# 6&emsp;Transfer Trained YOLO Model to Robot
1. Once the model has been trained, locate the best model using `Files`. The best model may appear in the directory
    `~/tb3_yolo_pc/train_yolo/runs/detect/train/weights/`, but always as `best.pt` (pick the most recent). 

2. Copy the `best.pt` into the robot. 
    1. On the VSCode that is connected to the robot, open the Explorer pane by using `CTRL+SHIFT+E`.
    2. If the folder is not open on the explorer pane, press `Open Folder`, and navigate to the `tb3_yolo` folder. For `ece83`, the path is `/home/ece83/tb3_yolo`. Trust the authors if prompted. This may close some terminals, but simply open them again later.
    3. Drag the `best.pt` from `Files` into the empty space in VSCode's explorer pane, and wait for the explorer pane to turn a slightly different shade while hovering the dragged file over it. Place the file only if a different shade is seen.

# 7&emsp;Test the YOLO Model
    1. Run `bringup.sh` in `RobotTerminalA`. Open a new terminal on VSCode with `` CTRL+` `` if needed.
    
    2. Run `run.sh` in `RobotTerminalB`. Open a new terminal on VSCode with `` CTRL+` `` if needed.
        ```bash
        cd ~/tb3_yolo
        ./run.sh
        ```

    3. Run `view_yolo.sh` in `PCTerminalA`. Open a new terminal with `CTRL+ALT+T` if needed. Replace the value of `ROS_DOMAIN_ID` with the team number:
        ```bash
        cd ~/tb3_yolo_pc
        ROS_DOMAIN_ID=83 ./view_yolo.sh
        ```

    4. Wait for a few seconds. Present the images in front of the camera, and the image classes and confidence values should appear on the camera feed.