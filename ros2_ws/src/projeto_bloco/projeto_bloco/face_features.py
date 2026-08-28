"""
Subscriber: detecta rostos em /camera/image_raw (Haar cascade) e extrai features de cada rosto.

Features por rosto (so OpenCV, sem modelos baixados):
  - olhos: centros achados pelo Haar cascade de olhos (landmarks)
  - ORB: keypoints e descritores binarios na regiao do rosto
  - HOG: vetor de aparencia do rosto (recorte 64x64 -> 1764 floats)

Publica /vision/faces (projeto_bloco_interfaces/msg/FaceArray) e /vision/faces_image (anotado).
Servico /vision/face_status: numero de rostos no ultimo frame.
"""

import signal
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from projeto_bloco_interfaces.msg import Face, FaceArray
from projeto_bloco_interfaces.srv import FaceStatus

HAAR = '/usr/share/opencv4/haarcascades/'   # cascades do pacote opencv-data do sistema


class FaceFeatures(Node):

    def __init__(self):
        super().__init__('face_features')
        self.declare_parameter('min_face_size', 30)
        self.declare_parameter('show', False)
        self.min_face = int(self.get_parameter('min_face_size').value)
        self.show = bool(self.get_parameter('show').value)

        self.face_cascade = cv2.CascadeClassifier(HAAR + 'haarcascade_frontalface_alt2.xml')
        self.eye_cascade = cv2.CascadeClassifier(HAAR + 'haarcascade_eye.xml')
        self.orb = cv2.ORB_create(nfeatures=200, fastThreshold=10)   # FAST 10: pele e lisa
        self.hog = cv2.HOGDescriptor((64, 64), (16, 16), (8, 8), (8, 8), 9)   # 1764 valores

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.num_faces = 0
        self.frames = 0

        self.create_subscription(Image, '/camera/image_raw', self.processar,
                                 qos_profile_sensor_data)
        self.pub_faces = self.create_publisher(FaceArray, '/vision/faces', 10)
        self.pub_img = self.create_publisher(Image, '/vision/faces_image', qos_profile_sensor_data)
        self.create_service(FaceStatus, '/vision/face_status', self.status)
        self.get_logger().info('Haar frontal + olhos + ORB + HOG; /vision/face_status pronto')

    def extrair(self, gray, x, y, w, h):
        """Monta a msg Face de um rosto: caixa, olhos, ORB e HOG; devolve (msg, keypoints)."""
        roi = gray[y:y + h, x:x + w]
        face = Face()
        face.x, face.y, face.width, face.height = int(x), int(y), int(w), int(h)

        # olhos: na metade superior do rosto, os 2 maiores, da esquerda p/ direita
        olhos = self.eye_cascade.detectMultiScale(roi[:int(h * 0.55)], 1.1, 5,
                                                  minSize=(max(w // 10, 8),) * 2)
        olhos = sorted(sorted(olhos, key=lambda e: -e[2] * e[3])[:2], key=lambda e: e[0])
        face.landmarks = [float(v) for (ex, ey, ew, eh) in olhos
                          for v in (x + ex + ew / 2, y + ey + eh / 2)]

        kps, des = self.orb.detectAndCompute(roi, None)
        for k in kps:                                  # coordenadas do ROI -> frame
            k.pt = (k.pt[0] + x, k.pt[1] + y)
        face.num_keypoints = len(kps)
        face.orb_descriptors = des.flatten().tolist() if des is not None else []

        recorte = cv2.equalizeHist(cv2.resize(roi, (64, 64)))
        face.descriptor = self.hog.compute(recorte).ravel().tolist()
        return face, kps

    def processar(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        caixas = self.face_cascade.detectMultiScale(gray, 1.1, 5, minSize=(self.min_face,) * 2)
        rostos = [self.extrair(gray, *c) for c in caixas]

        with self.lock:
            self.num_faces = len(rostos)
            self.frames += 1

        arr = FaceArray()
        arr.header = msg.header
        arr.faces = [f for f, _ in rostos]
        self.pub_faces.publish(arr)

        out = frame.copy()
        for f, kps in rostos:
            cv2.drawKeypoints(out, kps, out, color=(0, 220, 255), flags=0)
            cv2.rectangle(out, (f.x, f.y), (f.x + f.width, f.y + f.height), (0, 255, 0), 2)
            for i in range(0, len(f.landmarks), 2):
                cv2.circle(out, (int(f.landmarks[i]), int(f.landmarks[i + 1])), 4, (255, 0, 0), -1)
            cv2.putText(out, f'kp={f.num_keypoints} olhos={len(f.landmarks) // 2}',
                        (f.x, max(f.y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(out, f'rostos: {len(rostos)}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 255), 2)
        img = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        img.header = msg.header
        self.pub_img.publish(img)
        if self.show:
            cv2.imshow('rostos', out)
            cv2.waitKey(1)

    def status(self, request, response):
        with self.lock:
            response.num_faces = self.num_faces
            response.frames_processed = self.frames
        response.message = (f'{response.num_faces} rosto(s) no ultimo frame '
                            f'({response.frames_processed} frames processados)')
        self.get_logger().info(f'/vision/face_status -> {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FaceFeatures()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 2o Ctrl+C (launch) nao interrompe a limpeza
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
