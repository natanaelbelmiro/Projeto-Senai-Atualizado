# from pymata4 import pymata4
# import time

# # Inicializa a comunicação com a placa
# board = pymata4.Pymata4()

# # Define os pinos
# SERVO_PIN = 9
# BUZZER_PIN = 8

# # Configura os pinos
# board.set_pin_mode_servo(SERVO_PIN)
# board.set_pin_mode_digital_output(BUZZER_PIN)

# def acionar_dispositivos(epi_detectado):
#     if epi_detectado:
#         print("EPI DETECTADO ✅")
#         # Gira o servo para 90 graus
#         board.servo_write(SERVO_PIN, 90)
#         # Desliga o buzzer
#         board.digital_write(BUZZER_PIN, 0)
#     else:
#         print("EPI NÃO DETECTADO ❌")
#         # Para o servo (posição neutra, ex: 0)
#         board.servo_write(SERVO_PIN, 0)
#         # Liga o buzzer
#         board.digital_write(BUZZER_PIN, 1)

# # Exemplo de uso:
# try:
#     while True:
#         # Aqui você integraria com o YOLO/OpenCV para detectar o EPI
#         # Substitua isso pela lógica real:
#         epi_detectado = True  # ou False dependendo da detecção

#         acionar_dispositivos(epi_detectado)
#         time.sleep(1)

# except KeyboardInterrupt:
#     print("Encerrando...")
#     board.shutdown()
