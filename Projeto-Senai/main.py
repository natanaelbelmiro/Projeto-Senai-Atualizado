import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

class PPEApp:
    def __init__(self):
        # Carregar modelo YOLO
        self.model = YOLO("C:/Users/Natan/Desktop/TRABALHO ATUALIADO/Projeto-Senai/bestn.pt")
        self.confidence_threshold = 0.8
        self.previous_detection = None  # Estado anterior

        # Inicializar webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Erro ao acessar a webcam.")
            exit()

        # Configurar janela
        cv2.namedWindow("PPE Detection", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("PPE Detection", cv2.WND_PROP_TOPMOST, 1)  # Manter janela em foco

        # Conectar ao Arduino
        try:
            port = 'COM5'  # Altere se necessário
            print(f"Conectando ao Arduino na porta {port}...")
            self.arduino = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Espera a inicialização
            self.arduino.reset_input_buffer()
            print("Arduino conectado com sucesso!")
        except Exception as e:
            print(f"Erro ao conectar ao Arduino: {e}")
            exit()

    def acionar_dispositivos(self, epi_detectado):
        try:
            if self.previous_detection != epi_detectado:
                comando = b'1' if epi_detectado else b'0'
                self.arduino.write(comando)
                self.arduino.flush()
                self.previous_detection = epi_detectado
                print(f"Comando enviado: {comando.decode()}")
        except Exception as e:
            print(f"Erro ao enviar comando para Arduino: {e}")

    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Erro ao capturar o frame da câmera.")
            return False

        # Fazer detecção
        results = self.model(frame, conf=self.confidence_threshold, iou=0.3)

        hardhat_count = 0
        if len(results[0].boxes) > 0:
            detections = results[0].boxes.cls
            for det in detections:
                label = results[0].names[int(det)]
                if label == 'hardhat':
                    hardhat_count += 1

        # Acionar Arduino
        epi_detectado = hardhat_count > 0
        self.acionar_dispositivos(epi_detectado)

        # Mostrar resultado
        annotated_frame = results[0].plot()
        cv2.putText(
            annotated_frame,
            f"Hardhat: {'Sim' if hardhat_count else 'Nao'}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if hardhat_count else (0, 0, 255),
            2
        )

        cv2.imshow("PPE Detection", annotated_frame)
        return True

    def run(self):
        try:
            print("Iniciando detecção de EPI. Pressione 'q' para sair.")
            while True:
                success = self.update_frame()
                if not success:
                    print("Erro na captura. Encerrando aplicação...")
                    break

                # Checar se 'q' foi pressionado
                key = cv2.waitKey(30)  # Aumentado para 30ms
                if key == ord('q'):
                    print("Tecla 'q' pressionada. Encerrando...")
                    break

        finally:
            print("Encerrando aplicação...")
            self.cap.release()

            if hasattr(self, 'arduino'):
                try:
                    self.arduino.write(b'0')
                    time.sleep(0.5)
                    self.arduino.close()
                    print("Arduino desconectado.")
                except Exception as e:
                    print(f"Erro ao fechar conexão com Arduino: {e}")

            cv2.destroyAllWindows()
            print("Aplicação encerrada.")


if __name__ == '__main__':
    app = PPEApp()
    app.run()