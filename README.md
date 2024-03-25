## Authors 
### Tathya Bhatt (tathyab | 120340246) & Mohammed Munnawar (mmunawwa | 120241642)

# A* Path Planning Algorithm with Obstacle Avoidance

In this project we implement the A* path planning algorithm to find the shortest path from the start point to the goal point while avoiding the obstacles present in the environment.

## Github Repository

Added the TAs as collaborators and the link for the github repo is : https://github.com/tathya7/Astar_Path_Planner.git

## Requirements that were used in this 

- Python 3
- OpenCV (cv2)
- NumPy
- heapq
- math

## Steps to run the file:

1. Clone this repository or download the `a_star_path_planning.py` file.
2. Ensure you have the required dependencies installed. You can install them using pip:
   
    ```bash
    pip install numpy opencv-python-headless
    ```
   
3. Run the script:

    ```bash
    python a_star_path_planning.py
    ```

4. Once you've installed the required libraries, you can run the code. Follow the prompts to input the parameters such as step size, clearance, robot radius, initial position and goal position.

5. Make sure to select positions within these ranges to ensure the path is generated correctly and to avoid obstacles.

6. In the provided video, the start position is (80, 50), and the goal position is (600, 400).

7. Once the simulation begins, it will continue until the goal is reached. At that point, a simulation video will be created as output.

8. You can find all the necessary files and implementations in the provided GitHub repository.

## Inputs

- Step size (1-10): Determines the granularity of movement.
- Clearance: Specifies the clearance around obstacles.
- Robot Radius: Radius of the robot.
- Initial Position: X and Y coordinates along with orientation angle.
- Goal Position: X and Y coordinates along with desired orientation angle.

## Outputs

- Video: `a_star_path_final.mp4` showing the path planning process and the final path.

