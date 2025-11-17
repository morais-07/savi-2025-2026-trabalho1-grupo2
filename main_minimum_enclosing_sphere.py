import open3d as o3d
import scipy
import numpy as np
from copy import deepcopy
from scipy.optimize import least_squares
from pathlib import Path
import time
from scipy.optimize import minimize #novo, perceber


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
    source_transformed = (R @ source_pts.T).T + t
    #calcular a distancia da source transformed até à target
    dist = source_transformed - target_pts
    #Calcular o erro Point-to-Plane 
    residuals = np.sum(dist * target_normals, axis=1)

    return residuals

def objective_function(params): # definição da função objetivo que recebe o vetor de 4 parâmetros (xc,yc,zc,r)
    radius = params[3]
    return radius

def constraint_function(params, all_points): #definição da função de restrição que recebe o vetor de 4 parâmetros (xc,yc,zc,r) e todos os pontos
    center = params[:3] # centro da esfera (xc,yc,zc)
    radius = params[3] # raio da esfera (r)
    distances = np.linalg.norm(all_points - center, axis=1) #calcula a distância euclidiana de todos os pontos ao centro proposto
    return radius - distances #retorna um array [r-d1, r-d2, ...]. O otimizador vai forçar todos os elementos deste array a serem >= 0.

def icp(source,target, init_transf, max_iterations=500, tolerance=1e7, max_correspondence_dist=0.1, visualize=False):
    print(f"A iniciar ICP personalizado. Iterações={max_iterations}, Tolerância={tolerance}")

    Transformation = deepcopy(init_transf) #matriz de transformação que se vai atualizando, começa como uma cópia da inicial
    
    kdtree = o3d.geometry.KDTreeFlann(target)
    source_pts_np = np.asarray(source.points) #converter os pontos da pc source para um array
    target_pts_np = np.asarray(target.points)
    target_normals_np = np.asarray(target.normals)#Converter as normais da target pc para array

    vis = None
    source_vis = None
    if visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Progresso do ICP')
        
        # Criar a geometria 'source' que vamos ATUALIZAR
        source_vis = deepcopy(source)
        source_vis.transform(init_transf) # Começa na posição inicial
        # Adicionar as geometrias à janela
        vis.add_geometry(source_vis) # Geometria que vamos animar
        vis.add_geometry(target)     # Geometria estática (alvo)

    prev_error = 9999999999 #valor grande inicial

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

        if visualize:
            # Aplicamos a transformação INCREMENTAL à geometria que está a ser mostrada na janela
            source_vis.transform(T_inc)
            
            vis.update_geometry(source_vis) # Diz ao Open3D que a geometria mudou
            vis.poll_events()               # Processa eventos 
            vis.update_renderer()           # Redesenha o ecrã
            time.sleep(0.2) # <--- Pausa de x segundos
        
        if np.abs(prev_error - current_error) < tolerance or current_error < tolerance:
            print( "Convergência atingida.")
            break

        prev_error = current_error

    if visualize:
        vis.destroy_window()

    return Transformation

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

    #Downsampling- reduz a densidade das nuvens, acelera o processo
    #A smaller voxel size will result in more points being retained and more detail being preserved, but 
    #will increase the computational costs and memory requirements
    
    voxel_size = 0.05 #0.25 m = 25 cm
    pcd1_ds = pcd1.voxel_down_sample(voxel_size)
    pcd2_ds = pcd2.voxel_down_sample(voxel_size)
    
    #Estimação de normais
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
    T_manual = np.array([
    [0.977, -0.113,  0.179,  0.931],
    [0.111,  0.994,  0.023,  0.112],
    [-0.180, -0.003,  0.984, -0.001],
    [0.000,  0.000,  0.000,  1.000]])

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
        max_correspondence_dist=max_corr_dist,
        visualize=True)

    #Visualização da Transformação Final - resultado do ICP
    print("\n--- Transformação Final (Custom ICP) ---")
    print(final_transformation)
    print("---------------------------------")
    
    print("\n--- A iniciar Tarefa 3 (Esfera Englobante Mínima) ---")

    # Análise para Otimização da Esfera Mínima (Tarefa 3)
    
    # Criar as nuvens de pontos finais e alinhadas para as utilizarmos na otimização da esfera
    source_final = deepcopy(source)
    source_final.transform(final_transformation)

    # Combinar as duas nuvens (JÁ ALINHADAS) para a otimização da esfera, assim temos todos os pontos disponíveis num único array
    all_points_np = np.vstack((
        np.asarray(source_final.points),
        np.asarray(target.points) # Target nunca se moveu
    ))
    print(f"Número total de pontos para a otimização da esfera: {len(all_points_np)}") # ver quantos pontos temos no total

    # Criar um "chute" inicial (x0) para os parâmetros [xc, yc, zc, r]
    initial_center = np.mean(all_points_np, axis=0) # média de todos os pontos (centróide)
    distances_from_initial = np.linalg.norm(all_points_np - initial_center, axis=1) # distância do centróide ao ponto mais longínquo
    initial_radius = np.max(distances_from_initial) # raio inicial, o maior valor de distância desde o centro inicial aos pontos
    x0 = np.concatenate((initial_center, [initial_radius])) # x0 é o nosso vetor de parâmetros inicial
    
    print(f"Raio Inicial (chute): {initial_radius:.4f}") #mostrar o raio inicial
    print(f"Centro Inicial (chute): {initial_center}")   #mostrar o centro inicial
    
    # Definir restrições e limites
    constraints = ({'type': 'ineq', 'fun': constraint_function, 'args': (all_points_np,)}) 
    # 'type': 'ineq' significa que fun(x) >= 0
    # 'fun': é a nossa função de restrição
    # 'args': são os argumentos extra a passar à 'fun' (neste caso, os pontos)
    bounds = [(None, None), (None, None), (None, None), (0.001, None)] # Raio > 0, centro sem limites

    print("A executar otimização para a esfera.")
    result_sphere = minimize( # função do scipy, precisa de todos estes argumentos
        objective_function, # minimizar o raio
        x0, # chute inicial
        method='SLSQP', #Least Squares
        constraints=constraints, # restrições
        bounds=bounds, # limites
        options={'disp': False, 'ftol': 1e-6} # 'disp: False' para menos spam
    )

    # Análise e Visualização Final
    if result_sphere.success:
        optimized_params = result_sphere.x # parâmetros otimizados
        optimized_center = optimized_params[:3] # centro otimizado
        optimized_radius = optimized_params[3] # raio otimizado
        
        print(f"Otimização da esfera concluída com SUCESSO.")
        print(f"Centro Final (xc, yc, zc): {optimized_center}")
        print(f"Raio Final (r): {optimized_radius:.4f}")

        # Criar a geometria da esfera
        sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=optimized_radius) # criar esfera com o raio otimizado
        sphere_mesh.translate(optimized_center) # Mover para o centro otimizado
        sphere_mesh.compute_vertex_normals() 
        
        # Definir o material transparente
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultLitTransparency"
        mat.base_color = [0.1, 0.8, 0.1, 0.5] # Cor: Verde, Alfa: 50%
        mat.base_roughness = 0.7
        mat.base_reflectance = 0.3

        # Mostrar o resultado final (Alinhamento + Esfera)
        print("\nA mostrar resultado final (Nuvens alinhadas + Esfera).")
        o3d.visualization.draw(
            [
                {"name": "Source (Final)", "geometry": source_final},
                {"name": "Target", "geometry": target},
                {"name": "Esfera Mínima", "geometry": sphere_mesh, "material": mat}
            ],
            show_skybox=False, 
            show_ui=True, 
            title="Resultado Final (ICP + Esfera Mínima)"
        )

    else:
        print(f"Otimização da esfera FALHOU. Mensagem: {result_sphere.message}")

if __name__== '__main__':
    main()