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
