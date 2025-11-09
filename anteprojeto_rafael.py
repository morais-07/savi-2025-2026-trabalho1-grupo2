#!/usr/bin/env python3
# shebang line for linux / mac

from copy import deepcopy
from functools import partial
import glob
from random import randint
from matplotlib import pyplot as plt
import numpy as np
import argparse
import open3d as o3d
import cv2 # Importar OpenCV

def main():

    base_path = '/home/rafael-morais/Desktop/SAVI/SAVI_Rafael_Morais/Trabalho 1/tum_dataset'

    # --- Carregamento de Imagens e Filtragem de Profundidade (OpenCV) ---
    # Imagem 1
    filename_rgb1 = f'{base_path}/rgb/1.png'
    filename_depth1 = f'{base_path}/depth/1.png'

    # Carregar com OpenCV
    # cv2.IMREAD_GRAYSCALE para RGB se o create_from_tum_format espera 1 canal
    # cv2.IMREAD_UNCHANGED para profundidade para garantir que os valores brutos sejam lidos
    cv_rgb1 = cv2.imread(filename_rgb1, cv2.IMREAD_COLOR) # Ou cv2.IMREAD_GRAYSCALE se for o caso
    cv_depth1 = cv2.imread(filename_depth1, cv2.IMREAD_UNCHANGED)

    if cv_rgb1 is None:
        print(f"ERRO: Não foi possível carregar a imagem RGB: {filename_rgb1}")
        return
    if cv_depth1 is None:
        print(f"ERRO: Não foi possível carregar a imagem de profundidade: {filename_depth1}")
        return

    # Exemplo de filtragem de profundidade (filtro bilateral)
    # Converter para float32, filtrar, e converter de volta para uint16
    cv_depth1_float = cv_depth1.astype(np.float32)
    cv_depth1_filtered_float = cv2.bilateralFilter(cv_depth1_float, d=9, sigmaColor=75, sigmaSpace=75)
    cv_depth1_filtered = cv_depth1_filtered_float.astype(np.uint16)
    # Ou cv_depth1_filtered = cv2.medianBlur(cv_depth1, 5) # Exemplo de filtro de mediana

    # Imagem 2
    filename_rgb2 = f'{base_path}/rgb/2.png'
    filename_depth2 = f'{base_path}/depth/2.png'

    cv_rgb2 = cv2.imread(filename_rgb2, cv2.IMREAD_COLOR) # Ou cv2.IMREAD_GRAYSCALE
    cv_depth2 = cv2.imread(filename_depth2, cv2.IMREAD_UNCHANGED)

    if cv_rgb2 is None:
        print(f"ERRO: Não foi possível carregar a imagem RGB: {filename_rgb2}")
        return
    if cv_depth2 is None:
        print(f"ERRO: Não foi possível carregar a imagem de profundidade: {filename_depth2}")
        return

    # Converter para float32, filtrar, e converter de volta para uint16
    cv_depth2_float = cv_depth2.astype(np.float32)
    cv_depth2_filtered_float = cv2.bilateralFilter(cv_depth2_float, d=9, sigmaColor=75, sigmaSpace=75)
    cv_depth2_filtered = cv_depth2_filtered_float.astype(np.uint16)
    # --- FIM: Carregamento de Imagens e Filtragem de Profundidade (OpenCV) ---


    # --- Criação de Nuvens de Pontos (Open3D a partir de dados OpenCV) ---
    # Converta imagens OpenCV (NumPy arrays) para open3d.geometry.Image
    # Para RGB, o Open3D espera RGB, mas o OpenCV lê BGR. Converta.
    # Se create_from_tum_format espera 1 canal para RGB, você precisará converter cv_rgb para escala de cinza antes de criar o o3d.Image.
    # Ex: o3d_rgb1 = o3d.geometry.Image(cv2.cvtColor(cv_rgb1, cv2.COLOR_BGR2GRAY))
    
    # Se o create_from_tum_format pode lidar com RGB de 3 canais, então:
    o3d_rgb1 = o3d.geometry.Image(cv2.cvtColor(cv_rgb1, cv2.COLOR_BGR2RGB)) 
    o3d_depth1 = o3d.geometry.Image(cv_depth1_filtered)

    o3d_rgb2 = o3d.geometry.Image(cv2.cvtColor(cv_rgb2, cv2.COLOR_BGR2RGB))
    o3d_depth2 = o3d.geometry.Image(cv_depth2_filtered)

    # Crie o objeto rgbd image
    # Note: create_from_tum_format espera a imagem RGB em escala de cinza (1 canal).
    # Se você carregou cv_rgb1 com IMREAD_COLOR, o o3d_rgb1 terá 3 canais.
    # Isso pode causar um erro ou comportamento inesperado com create_from_tum_format.
    # É mais seguro converter para grayscale ANTES de criar o o3d.geometry.Image para RGB.
    # Ex: rgbd1 = o3d.geometry.RGBDImage.create_from_tum_format(
    #        o3d.geometry.Image(cv2.cvtColor(cv_rgb1, cv2.COLOR_BGR2GRAY)), o3d_depth1)

    # Assumindo que você quer seguir o padrão TUM com RGB em grayscale para a RGBDImage
    rgbd1 = o3d.geometry.RGBDImage.create_from_tum_format(
        o3d.geometry.Image(cv2.cvtColor(cv_rgb1, cv2.COLOR_BGR2GRAY)), o3d_depth1)
    print(f"RGBDImage 1: {rgbd1}")

    rgbd2 = o3d.geometry.RGBDImage.create_from_tum_format(
        o3d.geometry.Image(cv2.cvtColor(cv_rgb2, cv2.COLOR_BGR2GRAY)), o3d_depth2)
    print(f"RGBDImage 2: {rgbd2}")

    # Usar os parâmetros intrínsecos default do Open3D (ex: PrimeSense)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    print(f"Parâmetros intrínsecos da câmera (Default): {intrinsic}")

    # Obtenha a nuvem de pontos da imagem rgbd
    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd1, intrinsic)
    pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd2, intrinsic)

    print(f"Número de pontos em pcd1 (antes do downsampling): {len(pcd1.points)}")
    print(f"Número de pontos em pcd2 (antes do downsampling): {len(pcd2.points)}")

    # Flip it, otherwise the pointcloud will be upside down
    pcd1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pcd2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # --- FIM: Criação de Nuvens de Pontos ---


    # --- Pré-processamento de Nuvens de Pontos (Open3D) ---
    # Downsampling (Subamostragem)
    voxel_size = 0.025 # Ajuste este valor. Um valor maior reduz mais pontos.
                      # 0.05 metros (5 cm) é um bom ponto de partida.
    pcd1_downsampled = pcd1.voxel_down_sample(voxel_size=voxel_size)
    pcd2_downsampled = pcd2.voxel_down_sample(voxel_size=voxel_size)
    print(f"Número de pontos em pcd1 (depois do downsampling): {len(pcd1_downsampled.points)}")
    print(f"Número de pontos em pcd2 (depois do downsampling): {len(pcd2_downsampled.points)}")

    # Estimação de Normais
    # search_param: Ajuste radius e max_nn.
    # radius: raio de busca para vizinhos (em metros, deve ser > voxel_size)
    # max_nn: número máximo de vizinhos
    radius_normals = voxel_size * 2 # Exemplo: 2x o tamanho do voxel
    max_nn_normals = 30 # Número de vizinhos para considerar
    pcd1_downsampled.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normals, max_nn=max_nn_normals))
    pcd2_downsampled.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normals, max_nn=max_nn_normals))

    # Opcional: Orientar as normais para fora da câmera (se for o caso)
    # pcd1_downsampled.orient_normals_to_align_with_direction([0, 0, 1])
    # pcd2_downsampled.orient_normals_to_align_with_direction([0, 0, 1])
    # --- FIM: Pré-processamento de Nuvens de Pontos ---


    # --- Visualização (para verificar as etapas) ---
    axes_mesh = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)

    pcd1_downsampled.paint_uniform_color([1, 0, 0])  # Vermelho
    pcd2_downsampled.paint_uniform_color([0, 0, 1])  # Azul
    
    # Para visualizar as normais, você pode adicionar:
    # pcd1_downsampled.orient_normals_to_align_with_direction([0, 0, 1]) # Exemplo: orientar para o eixo Z positivo
    # o3d.visualization.draw_geometries([pcd1_downsampled], point_show_normal=True)


    entities = [pcd1_downsampled, pcd2_downsampled, axes_mesh]
    o3d.visualization.draw_geometries(entities)


if __name__ == '__main__':
    main()