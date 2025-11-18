## Table of Contents

- [Introduction](#introduction)
- [Datasets Used](#datasets-used)
- [Libraries Used](#libraries-used)
- [Installation](#installation)
- [Code Explanation](#code-explanation)
- [Results](#results)
- [Authors](#authors)

---

## Introduction

This project implements 3D point cloud registration from RGB-D frames of the TUM dataset using the Iterative Closest Point (ICP) algorithm in two different approaches:

* **Task 1 – Native ICP Registration:**
    Using Open3D’s built-in ICP as a baseline to register the two point clouds. We experiment with different variants (Point-to-Point and Point-to-Plane) and parameters. We also explore two different approaches for the initial transformation matrix (Identity Matrix vs. Global Registration) to compare their effects on the final alignment.

* **Task 2 – Custom ICP Implementation:**
    Developing a fully custom ICP from scratch. The iterative loop was implemented manually, using Open3D's KD-Tree for nearest-neighbor search and SciPy's `scipy.optimize.least_squares` to solve the non-linear least-squares optimization problem at each iteration. This task uses an initial transformation matrix obtained manually (through CloudCompare).

* **Task 3 – Minimum Enclosing Sphere:**
    Computing the minimum enclosing sphere that contains all points from the two registered point clouds. The problem was formulated as a non-linear constrained optimization: the sphere center (xc, yc, zc) and radius (r) were jointly optimized using `scipy.optimize.minimize` (SLSQP solver). The objective is to minimize *r*, subject to the constraint that every point in both clouds lies inside or on the sphere.

---

## Datasets Used

This project uses data from the [TUM RGB-D Dataset]((https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download)), a standard benchmark for SLAM and computer vision algorithms.

The dataset provides synchronized color (RGB) and depth images. For this project, **two distinct frames**—each consisting of one RGB and one depth image—were used to generate the two point clouds for registration.

---

## Libraries Used

This project relies on several essential Python libraries for scientific computing and 3D processing:

* **[Open3D](http://www.open3d.org/):** The main library for all point cloud processing. It is used for:
    * Creating point clouds from RGB+D images.
    * Applying pre-processing (Voxel Downsampling).
    * Estimating normals.
    * Running the native ICP registration (T1).
    * Building KD-Trees for neighbor search (T2).
    * Visualizing all results.

* **[NumPy](https://numpy.org/):** The fundamental library for numerical computing. Used for:
    * All matrix and vector operations.
    * Manipulating lists of points and normals.
    * Creating and composing 4x4 transformation matrices.

* **[SciPy](https://scipy.org/):** Used for its robust optimization functions:
    * `scipy.optimize.least_squares`: The "engine" behind our custom ICP (T2), used to find the 6 transformation parameters (rotation/translation).
    * `scipy.optimize.minimize`: Used with the 'SLSQP' method to solve the minimum enclosing sphere problem (T3), a constrained optimization problem.

* **Standard Python Libraries:**
    * `pathlib`: To ensure the dataset paths work on any operating system.
    * `copy` (deepcopy): To create independent copies of point clouds.
    * `time`: Used in the ICP animation (T2) to create a short pause between iterations.

---

## Installation

To run this project locally, follow these steps.

1.  **Clone the Repository**

    ```bash
    git clone [YOUR-REPO-GIT-URL]
    cd [REPO-FOLDER-NAME]
    ```
ts\activate
    ```

2.  **Install Dependencies**

    You can install the required libraries directly with `pip`.

    ```bash
    pip install open3d numpy scipy
    ```

3.  **Run the Scripts**

    Ensure the `tum_dataset` folder is in the same directory as the scripts.

    ```bash
    # To run Task 1 (Native ICP)
    python3 main_ipc_id.py or main_icp_gr.py
    
    # To run Task 2 (Custom ICP)
    python3 main_custom_icp.py
    
    # To run Task 3 (Sphere)
    python3 main_minimum_enclosing_sphere.py
    ```

---

## Results

The main script (`tarefa2_custom_icp.py`) is designed to visually demonstrate the project's complete flow, from alignment to final analysis.

1.  **"Before" Visualization (Manual Alignment):**
    * When the script runs, the first window (`draw_geometries`) shows the point clouds (Red - Source, Blue - Target) with the initial manual transformation (obtained from CloudCompare) applied. The misalignment is visible.

2.  **"Intermediate" Visualization (ICP Animation):**
    * After closing the first window, the custom ICP (Task 2) begins.
    * A new window (`Visualizer`) opens and shows, iteration by iteration, the red cloud "pulling" and snapping into place against the blue cloud.
    * The terminal prints the MSE (Mean Squared Error) at each step, allowing us to watch the error decrease until convergence.

3.  **"After" Visualization (Final Result + Sphere):**
    * When the ICP converges, the animation window closes.
    * The script automatically starts Task 3, calculating the minimum enclosing sphere that contains *both* of the now-aligned clouds.
    * A final window (`draw`) is opened, showing the end result: the two perfectly aligned clouds, encased in a semi-transparent green sphere representing the smallest possible sphere that contains them.

---

## Authors

* **Ana [YOUR_LAST_NAME_HERE]** - (@Your-Github-User)
* **Rafael Morais** - (@Your-Github-User)
