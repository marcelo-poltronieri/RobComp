import time
from util import Mapa

class Control(Mapa): # Herdando de Mapa
    def __init__(self):
        # Inicializa a classe Pai
        super().__init__() 
        self.robot_state = 'inicio'
        self.state_machine = {
            'inicio': self.inicio,
            'forward': self.forward,
            'left': self.left,
            'right': self.right,
            'decisao': self.decisao,
            'stop': self.stop,
        }

    def inicio(self):
        self.robot_state = 'forward'
    
    def forward(self) -> None:
        # Move subtraindo 1 uma linha
        # Atualiza a posição
        posicao = (self.posicao[0] - 1, self.posicao[1])    
        self.atualizar_posicao(posicao)


    def left(self) -> None:
        # Move subtraindo 1 uma coluna
        # Atualiza a posição
        posicao  = (self.posicao[0], self.posicao[1] - 1)

        self.atualizar_posicao(posicao)

    def right(self) -> None:
        # Move somando 1 uma coluna
        # Atualiza a posição
        posicao  = (self.posicao[0], self.posicao[1] + 1)
        self.atualizar_posicao(posicao)
         
    
    def stop(self) -> None:
        # Não faz nada
        pass

    def decisao(self) -> None:
        # Não faz nada
        if self.grade[self.posicao[0] - 1, self.posicao[1]] == 0:
            self.robot_state = 'forward'
        if self.grade[self.posicao[0] - 1, self.posicao[1]] == 2:
                if self.grade[self.posicao[0], self.posicao[1] + 1] == 0:
                    self.robot_state = 'right'
                else:
                    if self.grade[self.posicao[0], self.posicao[1] - 1] == 0:
                        self.robot_state = 'left'

    def control(self) -> None:
        # Verifique se a posição acima está livre, se sim, mova para cima.
        # Se não, verifique se a posição à esquerda ou à direita está livre, se sim, mova para um dos lados.
        # Pare quando estiver na primeira linha.
        
        # Chamada do método de movimento a partir do dicionário
        # self.state_machine...

        # Mostre a grade atual
        self.state_machine[self.robot_state]()
        self.mostrar()
        print(self.robot_state)
        self.decisao()
        if self.posicao[0] == 0:
            self.robot_state = 'stop'
               
        
def main():
    control = Control()
    control.mostrar()

    i = 40
    
    while not control.robot_state == 'stop' and i > 0:
        control.control()
        time.sleep(1)
        i -= 1

if __name__=="__main__":
    main()
