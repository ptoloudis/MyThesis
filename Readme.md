# My Thesis Project

This repository contains the code and resources for my thesis project. The project is organized into several main directories, each serving a specific purpose.

## Project Structure

### Main Directories

- **Ardupilot/** – Contains the ArduPilot codebase and related configuration files.
- **Drone/** – Code and resources related to drone operations.
- **ML/** – Machine learning models and scripts.
- **MyMission/** – Mission-specific code and configurations.
- **Physics/** – Physics simulations and related code.
- **UnityRenderStreaming/** – Unity Render Streaming code and resources.

## Demo Video

Check out the demo video showcasing the object detection fly example:
[Object Detection Fly Demo](https://youtu.be/cW9KFUT225E)

## Getting Started

To get started with this project, clone the repository and navigate to the directory of interest. Follow the specific setup instructions provided in each directory.

```sh
git clone https://github.com/ptoloudis/MyThesis.git
cd MyThesis
```

## Running the Project

Follow these steps to set up and run the project:

### 1. Start Unity
- Open Unity and launch the project.

### 2. Start MATLAB
- Navigate to the physics simulation directory:
  ```sh
  cd Physics
  ```
- Run the following command in MATLAB:
  ```matlab
  Copter_SIM_multicopter("copter.json")
  ```

### 3. Start Docker Containers (Rover & Drone)
Ensure Docker is installed and running. You need **two separate Docker containers**.

#### Pull the ArduPilot Docker Image (if not already installed)
```sh
docker pull ardupilot/ardupilot
```

#### Start the Rover Container
```sh
docker run -d --name ardupilot_rover ardupilot/ardupilot

docker exec -it ardupilot_rover bash -c "sim_vehicle.py -j4 -v Rover --out <ip>:14551 --out <ip>:14550"
```

#### Start the Drone (ArduCopter) Container
```sh
docker run -d --name ardupilot_copter ardupilot/ardupilot

docker exec -it ardupilot_copter bash -c "sim_vehicle.py -j4 -v ArduCopter -f json:<ip> --add-param-file=MyMission/Copter/param.param --out <ip>:14549"
```

#### Verify Running Containers
To check if both containers are active:
```sh
docker ps
```

#### Stopping and Removing Containers
If you need to stop the simulation:
```sh
docker stop ardupilot_rover ardupilot_copter
docker rm ardupilot_rover ardupilot_copter
```

### 4. Copy Mission Files to the Containers
If you want to run auto missions, use the following commands after starting the Docker containers.

#### For the Rover:
```sh
docker cp MyMission/ ardupilot_rover:/home/ardupilot/MyMission/
```

#### For the Drone (ArduCopter):
```sh
docker cp MyMission/ ardupilot_copter:/home/ardupilot/MyMission/
```

Verify that the files have been copied successfully:
```sh
docker exec -it ardupilot_rover ls /home/ardupilot/MyMission/
docker exec -it ardupilot_copter ls /home/ardupilot/MyMission/
```

### 5. Start Unity Render Streaming
Navigate to the UnityRenderStreaming directory and run:
```sh
npm run start
```

### 6. Run Object Detection
If you want to run object detection to follow the Rover:
- Update the IP addresses as needed in the script.
- Execute the object detection script:
  ```sh
  python main.py
  ```

## Important Notes ⚠️
- Ensure that the IP addresses are correctly configured.
- Run MATLAB and Unity on the same machine.
- Verify that all assets are correctly loaded before starting the project.

