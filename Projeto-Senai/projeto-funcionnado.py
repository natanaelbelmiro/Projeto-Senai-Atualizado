import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

class PPEApp:
    def __init__(self):
        # Modelo YOLO
        self.model = YOLO("C:/Users/Natan/Desktop/TRABALHO ATUALIADO/Projeto-Senai/bestn.pt")

        self.confidence_threshold = 0.8
        
        # Status anterior (para evitar envios repetidos)
        self.previous_detection = None

        # Webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Erro ao acessar a webcam.")
            exit()

        # Janela
        cv2.namedWindow("PPE Detection", cv2.WINDOW_NORMAL)
        
        # Arduino setup com comunicação serial
        try:
            port = 'COM5'  # Altere para a porta correta do seu Arduino
            print(f"Conectando ao Arduino na porta {port}...")
            self.arduino = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Aguarde a inicialização da comunicação serial
            
            # Limpar o buffer de entrada para remover quaisquer bytes iniciais
            self.arduino.reset_input_buffer()
            
            print("Arduino conectado com sucesso!")
            
        except Exception as e:
            print(f"Erro ao conectar ao Arduino: {e}")
            exit()

    def acionar_dispositivos(self, epi_detectado):
        try:
            # Só envia comando se o status mudou
            if self.previous_detection != epi_detectado:
                if epi_detectado:
                    print("EPI DETECTADO ✅ - Enviando comando '1'")
                    self.arduino.write(b'1')  # Envia comando para girar servo continuamente
                else:
                    print("EPI NÃO DETECTADO ❌ - Enviando comando '0'")
                    self.arduino.write(b'0')  # Envia comando para parar servo e tocar buzzer
                
                self.arduino.flush()  # Garante que o comando é enviado
                self.previous_detection = epi_detectado
                
                # Pequena pausa para processar o comando
                time.sleep(0.1)
                
        except Exception as e:
            print(f"Erro ao comunicar com Arduino: {e}")

    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Erro ao capturar o frame.")
            return False

        results = self.model(frame, conf=self.confidence_threshold, iou=0.3)

        # Contagem de hardhats
        hardhat_count = 0
        if len(results[0].boxes) > 0:
            detections = results[0].boxes.cls
            for det in detections:
                label = results[0].names[int(det)]
                if label == 'hardhat':
                    hardhat_count += 1

        # Acionamento dos dispositivos
        epi_detectado = hardhat_count > 0
        self.acionar_dispositivos(epi_detectado)

        # Exibição do frame anotado
        annotated_frame = results[0].plot()

        # Anotações visuais
        cv2.putText(annotated_frame, f"Hardhat: {'Sim' if hardhat_count else 'Não'}", 
                   (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                   (0,255,0) if hardhat_count else (0,0,255), 2)

        cv2.imshow("PPE Detection", annotated_frame)
        return True

    def run(self):
        try:
            print("Iniciando detecção de EPI. Pressione 'q' para sair.")
            while True:
                if not self.update_frame():
                    break
                    
                # Verificar se a tecla 'q' foi pressionada
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Tecla 'q' pressionada. Encerrando o sistema...")
                    # Enviar comando para parar todos os dispositivos
                    self.arduino.write(b'2')  # Novo comando para encerramento total
                    self.arduino.flush()
                    break
                    
        finally:
            print("Encerrando aplicação...")
            self.cap.release()
            if hasattr(self, 'arduino'):
                try:
                    # Garante que todos os dispositivos estão desligados
                    self.arduino.write(b'2')  # Comando de encerramento
                    time.sleep(0.5)  # Espera processar
                    self.arduino.close()
                except:
                    pass  # Ignora erros ao fechar
            cv2.destroyAllWindows()
            print("Aplicação encerrada.")

if __name__ == '__main__':
    app = PPEApp()
    app.run()