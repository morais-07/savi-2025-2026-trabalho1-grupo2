import open3d as o3d
import cv2 as cv
import scipy
import numpy as np
import matplotlib
import argparse
import copy

def main():
    # Carregamento de imagens e filtragem de profundidade
    # imagem 1
    filename_rgb1 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/rgb/1.png'
    rgb1 = o3d.io.read_image(filename_rgb1)
    filename_depth1 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/depth/1.png'
    depth1 = o3d.io.read_image(filename_depth1)
    # imagem 2
    filename_rgb2 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/rgb/2.png'
    rgb2 = o3d.io.read_image(filename_rgb2)
    filename_depth2 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/depth/2.png'
    depth2 = o3d.io.read_image(filename_depth2)

    # Create the rgbd image
    rgbd1 = o3d.geometry.RGBDImage.create_from_tum_format(rgb1, depth1)
    print(rgbd1)
    rgbd2 = o3d.geometry.RGBDImage.create_from_tum_format(rgb2, depth2)
    print(rgbd2)

    # Gerar as Point Clouds
    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd1, o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd2, o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

    # Flip the Point Clouds
    pcd1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pcd2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    # Pré-processamento das Point Clouds
    voxel_size = 0.05  # 0.25 m = 25 cm
    pcd1_ds = pcd1.voxel_down_sample(voxel_size)
    pcd2_ds = pcd2.voxel_down_sample(voxel_size)
    print(f"Downsampling: {len(pcd1_ds.points)}")

    # 2- Estimação de normais
    r = voxel_size * 2.5
    m = 50
    pcd1_ds.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=r, max_nn=m))
    pcd2_ds.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=r, max_nn=m))

    # orientação consistente das normais
    pcd1_ds.orient_normals_consistent_tangent_plane(50)
    pcd2_ds.orient_normals_consistent_tangent_plane(50)

    # Visualização da point cloud
    def draw_registration_result(source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          zoom=0.4459,
                                          front=[0.9288, -0.2951, -0.2242],
                                          lookat=[1.6784, 2.0612, 1.4451],
                                          up=[-0.3402, -0.9189, -0.1996])

    # Definir source e target
    source = pcd1_ds
    target = pcd2_ds
    threshold = 0.02

    # Matriz inicial de identidade (substitui o global registration)
    trans_init = np.identity(4)
    print("Matriz de identidade como transformação inicial")
    print("Matriz de transformação inicial:")
    print(trans_init)

    # Visualizar alinhamento inicial (sem alinhamento real)
    draw_registration_result(source, target, trans_init)

    print("Alinhamento Inicial:")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print(evaluation)

    #------------------------------------------------------------------------------------
    #Point to Point ICP 
    #Distância Direta entre pontos
    #print("Point-to-Point ICP")
    #reg_p2p = o3d.pipelines.registration.registration_icp(
        #source, target, threshold, trans_init,
        #o3d.pipelines.registration.TransformationEstimationPointToPoint())
    #print(reg_p2p)
    #print("Transformation:")
    #print(reg_p2p.transformation)
    #draw_registration_result(source, target, reg_p2p.transformation)

    # Point to Plane ICP
    print("Point-to-Plane ICP")
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    print(reg_p2l)
    print("Transformation:")
    print(reg_p2l.transformation)
    draw_registration_result(source, target, reg_p2l.transformation)

if __name__ == '__main__':
    main()