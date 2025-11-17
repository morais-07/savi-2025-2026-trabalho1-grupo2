import open3d as o3d
import scipy
import numpy as np
import matplotlib
import argparse

#Processamento de dados comum a todas as tarefas

def main():

    #Carregamento de imagens e filtragem de profundidade
    #imagem 1
    filename_rgb1 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/rgb/1.png'
    rgb1 = o3d.io.read_image(filename_rgb1)

    filename_depth1 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/depth/1.png'
    depth1 = o3d.io.read_image(filename_depth1)
    #imagem 2
    filename_rgb2 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/rgb/2.png'
    rgb2 = o3d.io.read_image(filename_rgb2)

    filename_depth2 = '/home/anacorreia/Desktop/SAVI_2025/savi-2025-2026-trabalho1-grupo2/tum_dataset/depth/2.png'
    depth2 = o3d.io.read_image(filename_depth2)

    # Create the rgbd image
    rgbd1 = o3d.geometry.RGBDImage.create_from_tum_format(rgb1, depth1)
    rgbd2= o3d.geometry.RGBDImage.create_from_tum_format(rgb2, depth2)

    #Gerar as Point Clouds 
    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd1, o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

    pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd2, o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    #Flip the Point Clouds

    pcd1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pcd2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    #Pré-processamento das Point Clouds

    #1- downsampling- reduz a densidade das nuvens, acelera o processo
    #A smaller voxel size will result in more points being retained and more detail being preserved, but 
    #will increase the computational costs and memory requirements
    
    voxel_size = 0.05 #0.25 m = 25 cm
    pcd1_ds = pcd1.voxel_down_sample(voxel_size)
    pcd2_ds = pcd2.voxel_down_sample(voxel_size)
    
    #2- Estimação de normais
    #é feita uma KD-Tree de todos os pontos, apanha todos o nº de vizinhos máx dentro do raio r
    r = voxel_size * 2.5
    m = 50
    pcd1_ds.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    radius=r, max_nn=m))

    pcd2_ds.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    radius=r, max_nn=m))
    #orientação consistente das normais
    pcd1_ds.orient_normals_consistent_tangent_plane(50)
    pcd2_ds.orient_normals_consistent_tangent_plane(50)

    #Visualização da point cloud

    axes_mesh = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)
    pcd1_ds.paint_uniform_color([1, 0, 0])  # red, green, blue
    pcd2_ds.paint_uniform_color([0, 0, 1])

    entities = [pcd1_ds, pcd2_ds, axes_mesh]
    o3d.visualization.draw_geometries(entities)

if __name__== '__main__':
    main() 