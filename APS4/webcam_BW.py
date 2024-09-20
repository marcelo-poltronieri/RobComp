import cv2
import numpy as np

class ProcessImage:
    def __init__(self):
        self.bgr = None
        self.gray = None
        self.bw = None

    def run_image(self, image):
        # Salva a imagem recebida em self.bgr
        self.bgr = image

        # Converte a imagem para tons de cinza
        self.gray = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2GRAY)

        # Processa a imagem para preto e branco
        _, self.bw = cv2.threshold(self.gray, 128, 255, cv2.THRESH_BINARY)

    def show_image(self):
        # Exibe a imagem processada
        if self.bw is not None:
            cv2.imshow('Imagem Original', self.bgr)
            cv2.imshow('Imagem em Preto e Branco', self.bw)
            cv2.waitKey(1)
        else:
            print("Nenhuma imagem processada disponível.")

def main():
    # Inicializa a captura de vídeo da webcam
    cap = cv2.VideoCapture(0)

    # Verifica se a webcam foi aberta corretamente
    if not cap.isOpened():
        print("Erro ao abrir a webcam.")
        return

    # Cria um objeto da classe ProcessImage
    img_processor = ProcessImage()

    while True:
        # Captura uma imagem da webcam
        ret, frame = cap.read()

        if not ret:
            print("Erro ao capturar a imagem.")
            break

        # Processa a imagem capturada
        img_processor.run_image(frame)

        # Exibe a imagem processada
        img_processor.show_image()

        # Pressione 'q' para sair do loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera a captura e fecha as janelas
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()