import cv2
import numpy as np

class ProcessImage:
    def __init__(self):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        self.bgr = None
        
    def filter_bw(self,img,menor_bw,maior_bw):
        latinhas_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        latinhas_gray = cv2.inRange(latinhas_gray, menor_bw, maior_bw)

        # realiza a abertura
        mask = cv2.morphologyEx(latinhas_gray, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contornos, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    
        return contornos
    
    def filter_hsv(self,img,menor_hsv,maior_hsv):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_hsv = cv2.inRange(img_hsv, menor_hsv, maior_hsv)

        mask = cv2.morphologyEx(img_hsv, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contornos, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
        return contornos
    
    def get_center(self,contornos):
        
        centros = []
        for contour in contornos:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centros.append((cx, cy))
        return centros


    def find_latinhas(self, img, contornos):
        contours = sorted(contornos, key=cv2.contourArea, reverse=True)[:5]
        # Desenha os contornos das latinhas na imagem
        img_with_contours = img.copy()
        cv2.drawContours(img_with_contours, contours, -1, (0, 255, 0), 2)
        return img_with_contours, contours
    def find_latinha_life(self, img, latinhas_contours):
        latinhas_centers = self.get_center(latinhas_contours)
        
        # Faixa de cor verde da Coca-Cola Life em HSV
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        # Filtra a parte verde da latinha usando o método filter_hsv
        green_contours = self.filter_hsv(img, lower_green, upper_green)

        # Seleciona o maior contorno da parte verde
        green_contour = max(green_contours, key=cv2.contourArea)
        # Obtém o centro de massa do contorno verde
        green_center = self.get_center([green_contour])[0]
        
        # Compara as distâncias entre o centro da parte verde e as latinhas
        distancia_min = 1000
        closest_latinha_idx = 0
        for i,latinha_center in enumerate(latinhas_centers):
            dist = np.sqrt((green_center[0] - latinha_center[0]) * 2 + (green_center[1] - latinha_center[1]) * 2)
            if dist < distancia_min:
                distancia_min = dist
                closest_latinha_idx = i
        min_dist = float('inf')
        closest_latinha_idx = -1
        for i, latinha_center in enumerate(latinhas_centers):
            dist = np.sqrt((green_center[0] - latinha_center[0]) * 2 + (green_center[1] - latinha_center[1]) * 2)
            if dist < min_dist:
                min_dist = dist
                closest_latinha_idx = i

        # Desenha o contorno da Coca-Cola Life
        img_with_life = img.copy()
        cv2.drawContours(img_with_life, [latinhas_contours[closest_latinha_idx]], -1, (0, 0, 255), 2)
        return img_with_life
    def run_image(self,img):
        
        menor_bw = 180
        maior_bw = 255
        # Encontra contornos das latinhas
        contours = self.filter_bw(img, menor_bw, maior_bw)
        # Encontra e desenha as latinhas
        img_with_latinhas, latinhas_contours = self.find_latinhas(img, contours)
        # Encontra a latinha da Coca-Cola Life
        final_img = self.find_latinha_life(img_with_latinhas, latinhas_contours)
        self.bgr = final_img
    def show_image(self):
        cv2.imshow("Imagem Processada", self.bgr)
        cv2.waitKey(0)  
        cv2.destroyAllWindows()

def main():
    img_path = '/home/borg/colcon_ws/src/robcomp-24b-aps-5-quis/entregavel_5/img/coke-cans.png'
    img_path2 = '/home/borg/colcon_ws/src/robcomp-24b-aps-5-quis/entregavel_5/img/coke-cans2.png'
    img_path3 = '/home/borg/colcon_ws/src/robcomp-24b-aps-5-quis/entregavel_5/img/coke-cans3.png'
    # img = cv2.imread(img_path)
    # img = cv2.imread(img_path2)
    img = cv2.imread(img_path3)
    processor = ProcessImage()
    processor.run_image(img)
    processor.show_image()

if __name__ == '__main__':
    main()