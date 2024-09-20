import cv2
import numpy as np

class ProcessImage:
    def __init__(self):
        self.bgr = None

    def run_image(self, image):
        # Salva a imagem recebida em self.bgr
        self.bgr = image

        # Obtenha as dimensões da imagem
        height, width = self.bgr.shape[:2]

        # Define as dimensões de cada quadrante
        quadrante_height = height // 2
        quadrante_width = width // 3

        # Cria uma máscara preta
        mask = np.zeros((height, width), dtype=np.uint8)

        # Define as áreas brancas (que serão mantidas) na máscara

        # Quadrante superior esquerdo
        mask[0:quadrante_height, 0:quadrante_width] = 255

        # Quadrante superior direito
        mask[0:quadrante_height, 2*quadrante_width:] = 255

        # Quadrante inferior central
        mask[quadrante_height:, quadrante_width:2*quadrante_width] = 255
    

        # Aplica a máscara à imagem original
        self.bgr = cv2.bitwise_and(self.bgr, self.bgr, mask=mask)

    def show_image(self):
        # Exibe a imagem processada
        if self.bgr is not None:
            cv2.imshow('Imagem Cortada', self.bgr)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("Nenhuma imagem processada.")

def main():
    # Cria um objeto da classe ProcessImage
    img_processor = ProcessImage()

    # Carrega a imagem localizada em img/arara.jpg
    image = cv2.imread('img/arara.jpg')

    if image is None:
        print("Erro ao carregar a imagem. Verifique o caminho do arquivo.")
        return

    # Processa a imagem
    img_processor.run_image(image)

    # Exibe a imagem processada
    img_processor.show_image()

if __name__ == "__main__":
    main()
