# Docker

We have two search models

1. Optimized planning Method
2. Inch-by-Inch Search

## How to use the optimized planning Method

You will open 4 Terminals

1. In folder`/Docker`, build the image 

	in Terminal 1:
	
	```
	sudo docker build -t project_terminus . --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --no-cache 
	```
	
	- If your ssh key is id_ed25519
	
	  1. use`sudo docker build -t project_terminus . --build-arg ssh_prv_key="$(cat ~/.ssh/id_ed25519)" --no-cache `
	
	  2. edit the Dockerfile(line36-37): change`id_rsa` to `id_ed25519`
	
2. Run the image 
	
	in Terminal 1:
	
	```
	sudo docker run -it -d --network=host --name project_terminus project_terminus:latest bash
	```

3. Start the docker container 

   in Terminal 1:

   ```
   sudo docker start project_terminus
   ```

4. Open the unity environment

   in Terminal 1:

   ```
   roslaunch unity_bridge unity_sim.launch
   ```

   - please make sure the code has also been cloned into your computer and been built first.

5. Open the first Terminal in Docker container 

   in Terminal 2:

   ```
   sudo docker exec -it project_terminus bash
   ```

6. Run victim signal generation 

   in Terminal 2:

   ```
   source devel/setup.bash
   roslaunch victim_signal_gen victim.launch
   ```

7. Run rviz to see the trajectory in coordinate system 

   in Terminal 3:

   ```
   roslaunch trajectory_visualization traj_visualize.launch
   ```

8. Open the second Terminal in Docker container

   in Terminal 4:

   ```
   sudo docker exec -it project_terminus bash
   ```

9. Run trajectory planning

   in Terminal 4:

   ```
   source devel/setup.bash
   rosrun planning planning_node
   ```
   

## How to use the Inch-by-Inch Search

You will open 3 Terminals.

1. In folder`/Docker`, build the image 

   in Terminal 1:

   ```
   sudo docker build -t project_terminus . --build-arg ssh_prv_key="$(cat ~/.ssh/id_ed25519)" --no-cache 
   ```

2. Run the image 

   in Terminal 1:

   ```
   sudo docker run -it -d --network=host --name project_terminus project_terminus:latest bash
   ```

3. Start the docker container 

   in Terminal 1:

   ```
   sudo docker start project_terminus
   ```

4. Open the unity environment

   in Terminal 1:

   ```
   roslaunch unity_bridge unity_sim.launch
   ```

   - please make sure the code has also been cloned into your computer and been built first.

5. Open the first Terminal in Docker container 

   in Terminal 2:

   ```
   sudo docker exec -it project_terminus bash
   ```

6. Run victim signal generation 

   in Terminal 2:

   ```
   source devel/setup.bash
   rosrun unity_bridge victim
   ```

7. Open the second Terminal in Docker container

   in Terminal 3:

   ```
   sudo docker exec -it project_terminus bash
   ```

8. Run Trajectory

   in Terminal3:

   ```
   source devel/setup.bash
   roslaunch basic_waypoint_pkg waypoint_mission.launch
   ```

9. Go back to Terminal2 to see if the victims are found
