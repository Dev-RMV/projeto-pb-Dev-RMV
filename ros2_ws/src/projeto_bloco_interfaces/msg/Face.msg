# Um rosto detectado (Haar cascade) e suas features.
int32 x                    # caixa (pixels)
int32 y
int32 width
int32 height
float32[] landmarks        # centros dos olhos: [x, y] x 0..2 olhos
int32 num_keypoints        # keypoints ORB na regiao do rosto
uint8[] orb_descriptors    # num_keypoints x 32 bytes
float32[] descriptor       # HOG do recorte 64x64 (1764 valores)
