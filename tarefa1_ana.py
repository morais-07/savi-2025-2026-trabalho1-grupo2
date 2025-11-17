import open3d as o3d
import cv2 as cv
import scipy
import numpy as np
import matplotlib
import argparse
import copy

#Processamento de dados comum a todas as tarefas

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

    #axes_mesh = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)
    #pcd1_ds.paint_uniform_color([1, 0, 0])  # red, green, blue
    #pcd2_ds.paint_uniform_color([0, 0, 1])

    #entities = [pcd1_ds, pcd2_ds, axes_mesh]
    #o3d.visualization.draw_geometries(entities)

    #Registar uma das PC como source e outra como target
    #The input are two point clouds and an initial transformation that roughly aligns the source point cloud to the target point cloud.
    #o output é uma transformação que alinha +/- as 2 pointclouds 
    #ICP variants, the point-to-point ICP and the point-to-plane ICP
    
    #the function below visualizes a target pointcloud and a source pointcloud transformed with the alignment transformation
    #the more and tightly the 2 point clouds overlap with eachother with the initial trasnformation the better 
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
        
    def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5  # Perceber isto 
    
        print("Global Registration com RANSAC...")
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, 
            mutual_filter=True,  # Usa correspondências mútuas ???
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=3,  # 3 pontos para definir transformação rígida
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
        )
        return result   
        

    #Global Registration
    #This family of algorithms do not require an alignment for initialization.
    #They usually produce less tight alignment results and are used as initialization of the local methods. Alinhamento grosseiro, depois refina-se com ICP
    #we already downsampled the point clouds and computed normals, to global registration we have to compute an FPFH (Fast Point Feature Histograms) for 
    #each point.The FPFH feature is a 33-dimensional vector that describes the local geometric property of a point. 
    source = pcd1_ds
    target = pcd2_ds
    threshold = 0.02

    radius_feature = voxel_size * 5
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    #. Global Registration
    result_ransac = execute_global_registration(source, target, source_fpfh, target_fpfh, voxel_size)

    #. Resultados
    print("Global Registration concluída!")
    #print(f"Fitness: {result_ransac.fitness:.4f} (quanto maior, melhor)")
    #print(f"Inlier RMSE: {result_ransac.inlier_rmse:.4f} (quanto menor, melhor)")
    print("Matriz de transformação inicial (RANSAC):")
    print(result_ransac.transformation)

    # 4. Visualizar alinhamento grosseiro
    draw_registration_result(source, target, result_ransac.transformation)
    

    #Lê a PC source e a PC target, dá também uma matriz transformação incial (pode ser a identidade ou não)
    #demo_icp_pcds = o3d.data.DemoICPPointClouds()
    #source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
    #source = pcd1_ds    
    #target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
    #target = pcd2_ds
    #threshold = 0.02 #o que é isto?
    #trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         #[-0.139, 0.967, -0.215, 0.7],
                         #[0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    
    #trans_init = np.asarray([[1.000, 0.000, 0.000, 0.000],
                         #[0.000, 1.000, 0.000, 0.000],
                         #[0.000, 0.000, 1.000, 0.000], [0.0, 0.0, 0.0, 1.000]])
    #draw_registration_result(source, target, trans_init)
    
    #fitness: rácio entre o nº de pontos correspondentes com o nº de pontos na source, quanto maior melhor
    #inlier_rmse: root mean squared root, média do erro, quanto menor melhor
    trans_init = result_ransac.transformation

    print("Initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print(evaluation)

    #Point to Point ICP 
    #Distância Direta entre pontos
    #print("Apply point-to-point ICP")
    #reg_p2p = o3d.pipelines.registration.registration_icp(
        #source, target, threshold, trans_init,
        #o3d.pipelines.registration.TransformationEstimationPointToPoint())
    #print(reg_p2p)
    #print("Transformation is:")
    #print(reg_p2p.transformation)
    #draw_registration_result(source, target, reg_p2p.transformation)

    #resultado um bocaod mid (??), n está nem closely alinhado
    #Tanto o ptpoint como o ptplane têm by default nº máx de iterações igual a 30, podemos aumentar se achamos que n estão bem alinahdas

    #reg_p2p = o3d.pipelines.registration.registration_icp(
        #source, target, threshold, trans_init,
        #o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        #o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=4000))
    #print(reg_p2p)
    #print("Transformation is:")
    #print(reg_p2p.transformation)
    #draw_registration_result(source, target, reg_p2p.transformation)
    #continua um resultado mid mesmo com mais iterações, o que sugere que este é o melhor que o PtPoint consegue fazer com aquela matriz de transformação inicial, talvez mudar isso possa ajudar

    #Point to Plane ICP
    print("Apply point-to-plane ICP")
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    draw_registration_result(source, target, reg_p2l.transformation)
    #resultado não muito bom
    #visualizar resultado antes e depois, devia dar pc's com transformação inicial e transformação final
    #draw_registration_result(source,target,trans_init)
    #resultado antes e depois é o mm pq? n devia ser

if __name__== '__main__':
    main() 