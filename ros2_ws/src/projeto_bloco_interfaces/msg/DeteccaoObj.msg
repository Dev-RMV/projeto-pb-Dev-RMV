# Um objeto detectado pelo YOLO e rastreado (pixels do frame de /camera/image_raw).
string classe          # nome da classe no modelo YOLO (ex.: "alvo")
float32 confianca      # 0..1
int32 x                # canto superior esquerdo da caixa
int32 y
int32 w                # largura e altura da caixa
int32 h
int32 id               # id do rastreador (ByteTrack); -1 = sem id
float32 cx             # centro suavizado pelo filtro de Kalman
float32 cy
float32 vx             # velocidade estimada pelo filtro (px/s)
float32 vy
