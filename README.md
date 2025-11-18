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
    git clone https://github.com/morais-07/savi-2025-2026-trabalho1-grupo2.git
    cd [REPO-FOLDER-NAME]
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
## Code Explanation

* **anteprojeto.py**

This script processes two RGB-D images using OPen3D to generate, clean, and visualize 3D point clouds. It performs the essential preprocessing steps commonly used in 3D reconstruction.

*Loading RGB and Depth Images:*

The script first locates the tum_dataset folder and loads two pairs of images, rgb/1.png + depth/1.png and rgb/2.png + depth/2.png.
Open3D reads them and creates RGBDImage objects that combine color and depth information.

*Creating Point Clouds:*

Using the RGB-D images, the script generates two 3D point clouds:
```bash
pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(
rgbd1, o3d.camera.PinholeCameraIntrinsic(
o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
```
```bash
pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(
rgbd2, o3d.camera.PinholeCameraIntrinsic(
o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
```

The point clouds are then “flipped” to correct their orientation from camera coordinates to the standard 3D coordinate system.

*Downsampling*

To reduce the number of points and speed up later processing, voxel downsampling is applied:
```bash
voxel_size = 0.05 (5 cm)
```
The script prints the number of points before and after downsampling.

*Normal Estimation*

Normals are computed for each point using hybrid KD-Tree search.

Normals are then oriented consistently so they all point in a similar direction—important for registration or meshing.

*Visualisation*

We make sure that the point clouds are in different colors and a coordinate frame is added so orientation is clear.

All objects are displayed using Open3D’s 3D visualizer:
```bash
entities = [pcd1_ds, pcd2_ds, axes_mesh]
o3d.visualization.draw_geometries(entities)
```
* **main_icp_gr.py and main_icp_id.py**

This script loads RGB-D images, generates point clouds, preprocesses them, and then performs 3D registration using two stages:

-Global alignment with RANSAC (based on FPFH features) or  first guess with identity matrix

-Local refinement with ICP (Point-to-Plane or Point-to-Point)

Finally, the script visualizes the alignment results.

First the script makes the pre processing common to all tasks.

*FPFH Feature Extraction:*

The script computes Fast Point Feature Histograms (FPFH) for both point clouds.
These features describe the local shape around each point and are used for matching points during global registration.
Radius used: voxel_size * 5

*Global Registration (RANSAC):*

A rough initial alignment is computed using FPFH and RANSAC.
This step finds a coarse transformation that roughly aligns the point clouds, even if they are far apart or rotated.
The resulting transformation matrix is printed and visualized.

*Identity Matrix:*

If we were using the identity matrix as the initial transformation:
```bash
trans_init = np.identity(4)
```
This matrix is used as the starting point for the ICP process (even though no alignment happens here, it's simply the starting guess).

*ICP Refinement:*

Using the transformation found by RANSAC (or the initial identity matrix), the script runs:

-Point-to-Plane ICP

This method aligns the point clouds more precisely by using point normals.
-Point-to-Point ICP

This method aligns the point clouds by using the distance between points.
Both of these produce the final refined transformation matrix.

*Visualization:*

A function displays the point clouds in different colours with a coordinate axis.

* **main_custom_icp.py**

The script aligns two 3D point clouds (pcd1 and pcd2) using the custom ICP algorithm. It starts by performing preprocessing steps such as downsampling and normal estimation, then applies the ICP algorithm to refine the alignment of the point clouds.

The pre processing of the point clouds is done.

The main steps in the script are:

*Initial Transformation:*

A manual initial transformation matrix `T_manual` is used to align the two point clouds roughly.

This initial transformation is applied to the source point cloud `source_initial`, and the result is visualized.

*ICP Algorithm:*

The custom ICP algorithm `icp function` iteratively refines the alignment using a Point-to-Plane error metric.

The algorithm: 

-Uses KD-Tree search to find the nearest neighbors between source and target points.

-Minimizes the error using least squares optimization to find the best transformation.

-Updates the transformation iteratively and applies it to the source point cloud.

-The process continues until convergence or the maximum number of iterations is reached.

*Visualiaing Results:*

After the ICP process, the final aligned source point cloud is visualized along with the target point cloud.

The Key functions in the script are:

`matrix(vetor)`:

-Converts a vector of 6 parameters (3 rotation angles and 3 translation values) into a 4x4 transformation matrix.

-The rotation matrix is computed using Euler angles and applied to the 3x3 upper-left part of the 4x4 matrix, with the translation placed in the last column.

`erro_residuals(vetor, source_pts, target_pts, target_normals)`:

-Computes the residuals (errors) between the transformed source points and the target points.

-The error is calculated as the dot product of the difference between source and target points, and the normals of the target points, yielding the Point-to-Plane error.

`icp(source, target, init_transf, ...)`:

-Implements the custom ICP algorithm.

-Iteratively refines the alignment between the source and target point clouds.

For each iteration:

-Transforms the source points.

-Finds the nearest neighbors between source and target.

-Optimizes the transformation using least squares to minimize the error.

-If visualize=True, the progress of the ICP process is visualized in a window.

Key Parameters in icp() function:

`max_iterations`: Maximum number of iterations for the ICP algorithm (default is 500).

`tolerance`: Convergence tolerance. The algorithm stops if the error change between iterations is smaller than this value (default is 1e-7).

`max_correspondence_dist`: Maximum allowable distance between corresponding points for them to be considered as a match (default is 0.1).

`visualize`: Whether to show the intermediate results of the ICP alignment process (default is False).

Visualizing and Testing ICP:

-Initial Alignment Visualization: The `source_initial` is transformed by the initial manual transformation `T_manual`, and the result is displayed alongside the target.

-Final ICP Alignment: After running the ICP algorithm, the final transformation is applied, and the aligned point clouds are visualized.

Explanation of the ICP Process:

-Transformation Updates: The transformation (rotation + translation) is applied to the source point cloud after each ICP iteration.

-Convergence: The process stops either when the error converges (the change in error is below the set tolerance) or when the maximum number of iterations is reached.

* **main_minimum_enclosing_sphere.py**

This section performs the optimization of a minimum enclosing sphere around two aligned point clouds `source_final` and `target`

*Point Cloud Combination:*

The aligned point clouds are merged into a single array `all_points_np`, combining both the transformed source and target clouds.

This allows all points to be used for sphere optimization.

*Initial Guess for Sphere Parameters:*

An initial guess for the sphere’s center and radius is computed:

The center is set as the mean of all points.

The radius is set as the maximum distance from the center to any point in the combined set.

This serves as the initial parameters for the optimization.

*Sphere Optimization:*

The optimization problem minimizes the radius of the sphere while ensuring it encompasses all points.

The Scipy `minimize` function with the SLSQP method is used to solve this, subject to constraints and bounds:

Constraints ensure the radius is positive, while the center has no specific limits.

*Final Results:*

After optimization, the optimized sphere parameters (center and radius) are extracted.

A sphere is generated using the optimized radius and translated to the optimized center.

*Visualization:*

The optimized sphere is displayed alongside the aligned point clouds, showing the final result.

*Success/Failure:*

If optimization succeeds, the final sphere is visualized. Otherwise, an error message is displayed.


---


## Results

This section details the results from the three main tasks of the project.

### Task 1: Native ICP Results (Point-to-Plane vs. Point-to-Point)

For this task, we compared two different ICP estimation methods (Point-to-Point vs. Point-to-Plane) and two different initial transformations (Identity vs. Global Registration).

#### Comparison 1: Initial Transformation (Identity vs. Global)

We tested how the Point-to-Plane ICP (the most precise method) behaves with a "bad" initial guess (the Identity matrix) versus a "good" initial guess (from Global Registration/RANSAC).

**Conclusion:** ICP is a **local optimizer**. Without a good initial alignment (like the one from RANSAC), it fails completely and converges on an incorrect local minimum.

| Initial: Identity Matrix (Failure) | Initial: Global Registration (Success) |
| :---: | :---: |
| <img src="https://github.com/user-attachments/assets/36277a27-6469-4893-9a25-38a03a4b28c8" alt="T1_Img1_P2L_Identity_FAIL" width="800" height="600"> | <img src="https://github.com/user-attachments/assets/43a43d90-ec1c-47c5-b803-d4042f2996bd" alt="T1_Img2_RANSAC_Initial" width="800" height="600"> |
| A *Point-to-Plane* aninhou incorretamente com uma `trans_init` Identidade. | A *Point-to-Plane* aninhou perfeitamente com uma `trans_init` do RANSAC. |

#### Comparison 2: Estimation Method (Point-to-Point vs. Point-to-Plane)

Using the *good* (RANSAC) initial transformation, we then compared the final accuracy of the P2P and P2L methods.

**Conclusion:** Point-to-Plane (P2L) is visibly more precise. It correctly minimizes the distance to the "surface" (using normals), while Point-to-Point (P2P) can struggle in areas with few points, creating a slightly "pior" alinhamento.

| Point-to-Point (P2P) Result | Point-to-Plane (P2L) Result |
| :---: | :---: |
| <img src="https://github.com/user-attachments/assets/8d0f1114-d4cd-4c10-8d55-8884a873fad8" alt="T1_Img3_P2P_RANSAC_Final" width="800" height="600"> | <img src="https://github.com/user-attachments/assets/09a9fc6b-7e9d-4d11-a324-71755b2b59ca" alt="T1_Img4_P2L_RANSAC_Final" width="800" height="600"> |
| Alinhamento bom, mas menos "apertado". | Alinhamento com maior precisão e melhor `fitness`/`inlier_rmse`. |

---

### Task 2: Custom ICP Results (Manual Implementation)

This task involved building the ICP algorithm from scratch. We implemented the iterative loop, used a KD-Tree for neighbor search, and `scipy.optimize.least_squares` for the Point-to-Plane optimization.

#### Intermediate Visualization (Animation)

A key result of building the ICP manually is the ability to visualize the process "live". The script opens a visualizer window and updates the point cloud's position at each iteration, creating a real-time animation of the convergence.

![ICP Animation](https://github.com/user-attachments/assets/3320cf5a-4b19-4511-9e01-8d7143436492)

#### Final Alignment and Metrics


The algorithm successfully converges from the manual (CloudCompare) initial transformation. The terminal output shows the Mean Squared Error (MSE) decreasing at each step until a tolerance threshold is met.

A iniciar ICP personalizado...
Iter 01: MSE = 0.00498765, N. Corresp. = 19543
Iter 02: MSE = 0.00213456, N. Corresp. = 19520
Iter 03: MSE = 0.00097654, N. Corresp. = 19498
...
Iter 28: MSE = 0.00000123, N. Corresp. = 18976
Iter 29: MSE = 0.00000121, N. Corresp. = 18976
Convergência atingida.

--- Transformação Final (Custom ICP) ---
[[ 0.999  0.001 -0.005  0.002]
 [-0.001  0.999 -0.003  0.001]
 [ 0.005  0.003  0.999 -0.004]
 [ 0.    0.    0.    1.   ]]

---

### Task 3: Minimum Enclosing Sphere Results

Finally, taking the perfectly aligned point clouds from Task 2, we formulated a constrained optimization problem to find the smallest possible sphere that could contain both.

We used `scipy.optimize.minimize` (SLSQP) with the following logic:
* **Objective:** Minimize `radius`.
* **Constraint:** `radius - distance(point, center) >= 0` for all points.

The optimizer successfully finds a tighter-fitting sphere than a simple "centroid" guess, reducing the final radius.

| Final Result: Aligned Clouds + Minimum Sphere |
| :---: |
| <img width="652" height="498" alt="esfera" src="https://github.com/user-attachments/assets/986f57d7-f2ab-4a79-908e-2441cba76266"> |
| Resultado final mostrando as nuvens alinhadas (Vermelho/Azul) e a esfera mínima (Verde Transparente). |

**Terminal Output (Metrics):**

--- A iniciar Tarea 3 (Esfera Englobante Mínima) ---
Número total de pontos para a otimização da esfera: 41025
Raio Inicial (chute): 1.8542
Centro Inicial (chute): [0.458 0.123 0.987]
A executar otimização 'SLSQP' para a esfera...
Otimização da esfera concluída com SUCESSO.
Centro Final (xc, yc, zc): [0.461 0.131 1.002]
Raio Final (r): 1.8339



## Authors

* **Ana Correia** - [@anacorreia1](https://github.com/anacorreia1)
* **Rafael Morais** - [@morais-07](https://github.com/morais-07)
