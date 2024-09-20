import cv2

class ProcessImage:
    def __init__(self):
        self.bgr = None

    def run_image(self, image):
        # Salva a imagem em self.bgr
        self.bgr = image

        # Converte de BGR para RGB
        rgb_image = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2RGB)

        # Realiza a transposição da imagem (espelhamento)
        self.bgr = cv2.transpose(rgb_image)

    def show_image(self):
        # Exibe a imagem processada
        if self.bgr is not None:
            cv2.imshow('Imagem original', cv2.transpose(cv2.cvtColor(self.bgr, cv2.COLOR_RGB2BGR)))
            cv2.imshow('Processed Image', self.bgr)
            cv2.waitKey(1)  # Aguarda 1 ms para atualizar a imagem
        else:
            print("Nenhuma imagem processada.")

def main():
    # Inicializa a captura de vídeo da webcam
    cap = cv2.VideoCapture(0)
    img_processor = ProcessImage()

    while True:
        # Captura uma imagem da webcam
        ret, frame = cap.read()

        if not ret:
            print("Falha na captura de imagem da webcam.")
            break

        # Processa a imagem capturada
        img_processor.run_image(frame)

        # Exibe a imagem processada
        img_processor.show_image()

        # Sai do loop ao pressionar a tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera a captura de vídeo e fecha todas as janelas abertas
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
