import cv2
import numpy as np

class ProcessImage:
    def __init__(self):
        self.bgr = None
        self.hsv = None
        self.mask_green = None
        self.mask_blue = None
        self.mask_red = None
        self.mask = None

    def run_image(self, image):
        # Salva a imagem recebida em self.bgr
        self.bgr = image

        # Converte a imagem para o espaço de cor HSV
        self.hsv = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2HSV)

        # Define os limites para a cor verde dos creepers
        lower_green = np.array([65, 50, 10])
        upper_green = np.array([90, 250, 255])

        # Define os limites para a cor azul dos creepers
        lower_blue = np.array([100, 150, 10])
        upper_blue = np.array([140, 255, 255])

        # Define os limites para a cor vermelha dos creepers
        lower_red = np.array([170, 180, 10])
        upper_red = np.array([180, 255, 255])

        lower_red1 = np.array([0, 180, 50]) 
        upper_red1 = np.array([10, 255, 255])

        # Cria máscaras para as cores verde, azul e vermelha
        self.mask_green = cv2.inRange(self.hsv, lower_green, upper_green)
        self.mask_blue = cv2.inRange(self.hsv, lower_blue, upper_blue)
        self.mask_red = cv2.inRange(self.hsv, lower_red, upper_red) | cv2.inRange(self.hsv, lower_red1, upper_red1)

        # Soma as máscaras
        self.mask = self.mask_green | self.mask_blue | self.mask_red

    def show_image(self):
        # Exibe a imagem original e a máscara combinada em janelas separadas
        if self.bgr is not None and self.mask is not None:
            cv2.imshow('Imagem Original', self.bgr)
            #cv2.imshow('azul', self.mask_blue)
            #cv2.imshow('verde', self.mask_green)
            #cv2.imshow('vermelho', self.mask_red)
            cv2.imshow('Máscara', self.mask)
            cv2.waitKey(1)  # Aguarda brevemente para permitir a exibição da imagem em tempo real
        else:
            print("Nenhuma imagem processada ou máscara não disponível.")

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
