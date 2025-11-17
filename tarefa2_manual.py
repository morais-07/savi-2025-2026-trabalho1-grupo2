import open3d as o3d
import cv2 as cv
import scipy
import numpy as np
import matplotlib
import argparse
from copy import deepcopy
from scipy.optimize import least_squares

#Definição de Funções

def matrix(vetor):     #esta função recebe um vetor de 6 parâmetros r e t e retorna uma matriz de Transformação 4x4
    T = np.identity(4) #cria primeiro matriz identidade 4x4
    #Cria matriz de rotação R a partir dos ângulos de Euler (estes são os angulos que transformam um sistema de coordenadas noutro através dos angulos dados no vetor por rx ry rz)
    R = o3d.geometry.get_rotation_matrix_from_xyz(vetor[:3])
    T[:3,:3] = R #dá como valores nas 3 primeiras linhas e nas 3 primeiras colunas a matriz de rotação
    T[:3, 3] = vetor[3:] #dá como valores na ultima coluna e nas primeiras 3 linhas os valores de translação dados no vetor 
    return T #esta função devolve uma matriz transformação T com rotação, translação e 4ª linha 0001

def erro_residuals(vetor, source_pts, target_pts, target_normals): #definição da função de custo que recebe um vetor de 6 elementos e devolve os valores do erro(já filtrados)(valores da menor distancia entre pontos correspondentes)
    #Com o vetor que recebe a função constrói a matriz de Rotação e o vetor de translação
    R = o3d.geometry.get_rotation_matrix_from_xyz(vetor[:3])
    t = vetor[3:] #retira os valores do vetor começando no indice 3 até ao final, translação
    #Aplicar a transformação do vetor inserido aos pontos da source
    source_transformed = (R @ source_pts.T) + t
    #calcular a distancia da source transformed até à target
    dist = source_transformed - target_pts
    #Calcular o erro Point-to-Plane 
    residuals = np.sum(dist * target_normals, axis=1)

    return residuals

def icp(source,target, init_transf, max_iterations=500, tolerance=1e7, max_correspondence_dist=0.1):
    print(f"A iniciar ICP personalizado. Iterações={max_iterations}, Tolerância={tolerance}")

    Transformation = deepcopy(init_transf) #matriz de transformação que se vai atualizando, começa como uma cópia da inicial
    kdtree = o3d.geometry.KDTreeFlann(target)
    source_pts_np = np.asarray(source.points) #converter os pontos da pc source para um array
    target_pts_np = np.asarray(target.points)
    target_normals_np = np.asarray(target.normals)#Converter as normais da target pc para array

    prev_error = 9999999999

    #ciclo for que vai passar pelos pontos da source até ao número máx de iterações
    for i in range(max_iterations):
        source_current_points_np = (Transformation[:3, :3]@ source_pts_np.T).T + Transformation[:3,3] #transforma os pontos da source com a nossa matriz de transformação atual, para fazer este produto temos de fazer a transposta dos pontos da source, para meter no formato original voltamos a fazer T e adicionamos a translação (Acabámos de aplicar a matriz de transformação aos pontos da source atuais)
        #criar listas vazias para fazer a correspondencia dos indices dos pontos
        source_idx = []
        target_idx = []

        #iterar por todos os pontos da source já transformada e encontra vizinho mais próximo na target tal como a distancia a esse
        for idx_s,pt_s in enumerate(source_current_points_np): #enumerate retorna indice,valor
            [k,idx_t,dist_st] = kdtree.search_knn_vector_3d(pt_s,1) #procura o vizinho mais próximo do ponto na source na kdtree

            #vamos filtrar, só se aceita o idx e dist se a distancia estiver neste int
            if dist_st[0] < max_correspondence_dist**2: #só se aceita se dist for menor que max dist ²
                source_idx.append(idx_s)
                target_idx.append(idx_t[0]) #append o primeiro valor de idx_t no DoubleVector
        if len(source_idx) < 10:
            print("ICP falhou: Poucas correspondências")
            break
        
        #Preparação de dados para otimização
        current_source_align = source_current_points_np[source_idx] #Coloca apenas as linhas de source_current_point_np cujos indices estão na source_idx, os indices que passaram no filtro de distancia
        target_corr_points = target_pts_np[target_idx]
        target_corr_normals = target_normals_np[target_idx]

        #Otimização
        #Objetivo: encontrar a transformação que minmiza o erro (distancia entre a soruce e target)
        #ou seja entre current_source_align e target_corr_points

        #O initial guess é 0 pq procuramos uma pequena transformação incremental
        initial_guess_params = np.zeros(6)   
        
        result = least_squares(
            erro_residuals,
            initial_guess_params,
            args = (current_source_align, target_corr_points, target_corr_normals)
        )

        optimal_params = result.x 
        #Converter para a matriz de transformação incremental
        T_inc = matrix(optimal_params) 

        #Atualizar a transformação acumulada
        Transformation = T_inc @ Transformation   #soma das transformações feitas até agora (total) mais a transformaçãozinha de agora

        current_error = np.mean(result.fun**2) #Mean Squared Error

        print(f"Iter {i+1:02d}: MSE = {current_error:.8f}, N. Corresp. = {len(source_idx)}")

        
        if np.abs(prev_error - current_error) < tolerance or current_error < tolerance:
            print( "Convergência atingida.")
            break

        prev_error = current_error

    return Transformation

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
    print(rgbd1)

    rgbd2= o3d.geometry.RGBDImage.create_from_tum_format(rgb2, depth2)
    print(rgbd2)

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

    #Execução da tarefa 2
    #Definir source e target
    source = pcd1_ds
    target = pcd2_ds

    source.paint_uniform_color([1,0,0])    #Vermelho(Source)
    target.paint_uniform_color([0,0,1])    #Azul(Target)
    axes_mesh = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)

    #Dar alinhamento inicial
    T_manual = #[METE A MATRIZ MORAIS]

    print("---Transformação Inicial Manual---")
    print(T_manual)
    print("----------------------------------")

    source_initial = deepcopy(source)
    source_initial.transform(T_manual) #aplicar a transformação estimada à nuvem de pontos source
    
    #Visualizar alinhamento inicial 
    print("A mostrar alinhamento inicial")
    o3d.visualization.draw_geometries([source_initial, target, axes_mesh])

    #Executar ICP Personalizado
    max_corr_dist = voxel_size * 3 #distância máxima de correspondencia, máx de distância entre pontos para estes serem considerados correspondentes

    final_transformation = icp(
        source,
        target,
        T_manual,
        max_iterations=500,
        tolerance=1e-8,
        max_correspondence_distance=max_corr_dist
    )

    #Visualização da Transformação Final - resultado do ICP
    print("\n--- Transformação Final (Custom ICP) ---")
    print(final_transformation)
    print("---------------------------------")

    source_final = deepcopy(source)
    source_final.transform(final_transformation)  #Aplicar a transformação final à source pc (cópia)

    print("A mostrar resultado final do ICP personalizado")
    o3d.visualization.draw_geometries([source_final, target, axes_mesh])






