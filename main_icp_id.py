import open3d as o3d
import cv2 as cv
import scipy
import numpy as np
import copy
from pathlib import Path

def main():
    #Carregamento de imagens e filtragem de profundidade
    #imagem 1
    script_dir = Path(__file__).parent.resolve()
    # Define o 'base_path' como sendo a pasta 'tum_dataset'
    base_path = script_dir / 'tum_dataset'

    filename_rgb1 = base_path / 'rgb' / '1.png'
    filename_depth1 = base_path / 'depth' / '1.png'
    filename_rgb2 = base_path / 'rgb' / '2.png'
    filename_depth2 = base_path / 'depth' / '2.png'

    rgb1 = o3d.io.read_image(str(filename_rgb1))
    depth1 = o3d.io.read_image(str(filename_depth1))
    rgb2 = o3d.io.read_image(str(filename_rgb2))
    depth2 = o3d.io.read_image(str(filename_depth2))

    # Create the rgbd image
    rgbd1 = o3d.geometry.RGBDImage.create_from_tum_format(rgb1, depth1)
    rgbd2= o3d.geometry.RGBDImage.create_from_tum_format(rgb2, depth2)

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
        
        source_temp.paint_uniform_color([1, 0, 0]) # Vermelho
        target_temp.paint_uniform_color([0, 0, 1]) # Azul
        
        source_temp.transform(transformation)
        axes_mesh = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)
        o3d.visualization.draw_geometries([source_temp, target_temp, axes_mesh])

    # Definir source e target
    source = pcd1_ds
    target = pcd2_ds
    threshold = 0.05 # distância máxima para correspondência de pontos

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