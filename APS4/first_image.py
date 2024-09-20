import cv2

class ProcessImage:
    def __init__(self):
        self.bgr = None

    def load_image(self, image_path):
        image_path = 'img/ararap1.jpeg'
        # Carrega a imagem e salva em self.bgr
        self.bgr = cv2.imread(image_path)

    def show_image(self):
        # Exibe a imagem carregada
        if self.bgr is not None:
            cv2.imshow('Image', self.bgr)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("Nenhuma imagem carregada.")

    def show_channels(self):
        # Exibe os canais B, G e R em janelas separadas
        if self.bgr is not None:
            b, g, r = cv2.split(self.bgr)
            cv2.imshow('IMAGEM ORIGINAL', self.bgr)
            cv2.imshow('AZUL', b)
            cv2.imshow('VERDE', g)
            cv2.imshow('VERMELHO', r)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("Nenhuma imagem carregada.")

def main():
    # Cria um objeto da classe ProcessImage
    img_processor = ProcessImage()

    # Carrega a imagem
    img_processor.load_image('img/ararap1.jpeg')

    # Exibe os canais de cor da imagem
    img_processor.show_channels()

if __name__ == "__main__":
    main()
