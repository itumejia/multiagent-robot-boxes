from math import nan
from mesa import Agent, Model
from mesa.space import MultiGrid
from mesa.time import RandomActivation
from mesa.visualization.modules import CanvasGrid, TextElement
from mesa.visualization.ModularVisualization import ModularServer
from numpy import random as npRandom
from pathfinding.core.grid import Grid as AStarGrid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

class Pared(Agent):
  def __init__(self, model, pos):
    super().__init__(model.next_id(), model)
    self.pos = pos

class Pila(Agent):
    def __init__(self, model, pos, nCajas):
        super().__init__(model.next_id(), model)
        self.cargada = False
        self.pos = pos
        self.nCajas = nCajas
        self.nCajasEsperadas = nCajas #Cajas actualmente en la pila + el numero de cajas que vienen en camino

class Robot(Agent):
    def __init__(self, model, pos):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.destino = None
        self.path = None
        self.pasoPath = 0
        self.cajaCargada = None
        self.pilaDestino = None

    def step(self):
        #No esta cargando una caja, busca una de manera random
        if(self.path == None):
            # Recorrido aleatorio (toma de decisión aleatoria)
            next_moves = self.model.grid.get_neighborhood(self.pos, moore=True)
            # Escoge una casilla al azar de las disponibles
            next_move = self.random.choice(next_moves)
            # Verifica que la casilla esté disponible y realiza el step.
            if(self.model.matrix[next_move[1]][next_move[0]]==1):
                self.model.grid.move_agent(self, next_move)
                self.model.movimientos += 1
                celda = self.model.grid.get_cell_list_contents([self.pos])

                #Revisa si llego a una caja que se encuentra sola
                if len(celda) > 0 and isinstance(celda[0], Pila) and celda[0].nCajasEsperadas == 1 and celda[0].cargada == False:
                    celda[0].cargada = True
                    self.pilaDestino = self.model.pilaMasCercana(celda[0])
                    #Ya no hay pilas por apilar
                    if(self.pilaDestino == None):
                        return
                    self.pilaDestino.nCajasEsperadas += 1
                    self.destino = self.pilaDestino.pos
                    self.cajaCargada = celda[0]
                    self.path = self.getAStarPath(self.destino)
        
        #Se encuentra cargando una caja y va hacia un destino
        else:
            if(self.pasoPath < len(self.path) - 1):
                self.model.movimientos += 1
                self.model.grid.move_agent(self, self.path[self.pasoPath])                
                self.model.grid.move_agent(self.cajaCargada, self.path[self.pasoPath])  

                self.pasoPath+=1
            
            #Ya se encuentra junto a la pila para apilar
            else:
                #Se elimina la pila con la caja individual, y se agrega una caja a la pila destino
                self.pilaDestino.nCajas += 1
                self.model.kill_agents.append(self.cajaCargada)

                #Reestablecer valores del recorrido del robot
                self.destino = None
                self.path = None
                self.pasoPath = 0
                self.cajaCargada = None
                self.pilaDestino = None

    
    def getAStarPath(self, destino):
        # Definimos el punto inicial y final del recorrido
        start = self.model.gridPathFinder.node(self.pos[0], self.pos[1])
        end = self.model.gridPathFinder.node(destino[0], destino[1])
        # Se encuentra el camino más óptimo
        finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        # Se almacenan las celdas a seguir
        path, _ = finder.find_path(start, end, self.model.gridPathFinder)
        self.model.gridPathFinder=AStarGrid(matrix=self.model.matrix)
        return path

class Almacen(Model):
 
    def __init__(self, nRobots=5, nCajas = 20):
        super().__init__()
    
        self.schedule = RandomActivation(self)
        self.movimientos = 0
        self.kill_agents = []
        self.steps = 0
        self.nRobots = nRobots
        self.nCajas = nCajas

        #Crear mesa con los tamaños dados
        self.grid = MultiGrid(10, 10, torus=False)
        self.matrix = [
            [1,1,1,1,0,0,1,1,1,1],
            [1,1,1,1,0,0,1,1,1,1],
            [1,1,1,1,1,1,1,1,1,1],
            [1,1,1,1,1,1,1,1,1,1],
            [0,0,1,1,0,0,1,1,0,0],
            [0,0,1,1,0,0,1,1,0,0],
            [1,1,1,1,1,1,1,1,1,1],
            [1,1,1,1,1,1,1,1,1,1],
            [1,1,1,1,0,0,1,1,1,1],
            [1,1,1,1,0,0,1,1,1,1]
            ]

        self.gridPathFinder=AStarGrid(matrix=self.matrix)

        #Realizar un muestreo para posicionar a todos los agentes
        celdasDisponibles = 80
        prob = 1 / celdasDisponibles
        probArray = []
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[i])):
                if(self.matrix[i][j] == 1):
                    probArray.append(prob)
                else:
                    probArray.append(0)


        npSample = npRandom.choice(100, nCajas + nRobots, replace= False, p= probArray)
        sample = npSample.tolist()

        # Se posicionan los agentes de acuerdo al muestreo en donde corresponden
        index_muestreo = 0

        #Se posicionan robots
        for i in range(nRobots):
            robot = Robot(self, (sample[index_muestreo] // 10, sample[index_muestreo] % 10))
            self.grid.place_agent(robot, robot.pos)
            self.schedule.add(robot)
            index_muestreo += 1

        #Se posicionan cajas
        for i in range(nCajas):
            caja = Pila(self, (sample[index_muestreo] // 10, sample[index_muestreo] % 10), 1)
            self.grid.place_agent(caja, caja.pos)
            self.schedule.add(caja)
            index_muestreo += 1

        #Se posicionan obstaculos
        for _,x,y in self.grid.coord_iter():
            if self.matrix[y][x] == 0:
                block = Pared(self, (x, y))
                self.grid.place_agent(block, block.pos)

    def step(self):

        for agent in self.kill_agents:
            print(agent)
            print(agent.pos)
            self.grid.remove_agent(agent)
            self.schedule.remove(agent)
            self.kill_agents.remove(agent)

        terminado = self.revisarTerminado()
        print(terminado)
        if not terminado:
            self.schedule.step()
            self.steps += 1
        else:
            print("Movimientos: " + str(self.movimientos))
            print("Tiempo transcurrido: " + str(self.steps))

    def revisarTerminado(self):
        nCajasUnicas = 0
        nPilasNoLlenas = 0
        for agent in self.schedule.agents:
            if(isinstance(agent, Pila)):
                if(agent.nCajas == 1):
                    nCajasUnicas += 1
                if(agent.nCajas < 5):
                    nPilasNoLlenas += 1
        print("Pilas no llenas")
        print(nPilasNoLlenas)
        return (nCajasUnicas <= 1 and nPilasNoLlenas < 2) or nPilasNoLlenas == 0


    def pilaMasCercana(self, cajaActual):
        cajaUnica = None
        for agent in self.schedule.agents:
            if(isinstance(agent, Pila) and agent != cajaActual):
                if(agent.nCajasEsperadas < 5 and agent.nCajasEsperadas > 1):
                    return agent
                elif agent.nCajasEsperadas == 1:
                    cajaUnica = agent

        return cajaUnica #Regesa una pila de caja unica en caso de no haber encontrado pilas mas grandes
                        #Si regresa None, ya no hay mas cajas por apilar



class ResultsElement(TextElement):
    def __init__(self):
        pass

    def render(self, model):
        return "Movimientos: " + str(model.movimientos) + ", Tiempo transcurrido: " + str(model.steps)


def agent_portrayal(agent):
  # Dependiendo del tipo de agente se genera el formato correspondiente
    if(type(agent) is Robot):
    # Se le da formato al agente Aspiradora
        return {"Shape": "robot.png", "Layer": 0}
    # Se le da formato a la caja
    elif(type(agent) is Pila):
        if agent.nCajas == 1:
            return {"Shape": "1boxes.png", "Layer": 0}
        if agent.nCajas == 2:
            return {"Shape": "2boxes.png", "Layer": 0}
        if agent.nCajas == 3:
            return {"Shape": "3boxes.png", "Layer": 0}
        if agent.nCajas == 4:
            return {"Shape": "4boxes.png", "Layer": 0}
        if agent.nCajas == 5:
            return {"Shape": "5boxes.png", "Layer": 0}
    elif(type(agent) is Pared):
        return {"Shape": "rect",  "w": 1, "h": 1, "Filled": "true", "Color": "Gray", "Layer": 0}

    
grid = CanvasGrid(agent_portrayal, 10, 10, 450, 450)
results_element = ResultsElement()

server = ModularServer(Almacen, [grid, results_element], "Limpieza", {})
server.port = 8522
#server.launch()