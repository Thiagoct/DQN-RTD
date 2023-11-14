import numpy as np
import random
import time
from coppeliasim_zmqremoteapi_client import *
import pandas as pd

class robot_goal:
    def __init__(self):
        #Definição de constantes
        self.D = 195e-3
        self.R = self.D/2
        self.L = 381e-3
        self.gamma = 0.5
        self.k = 0.5
        self.h = 1 
        #Inicialização do simulador e objetos
        self.__initialize_simulation()
        #Inicializar ambiente
        self.id = 0

    def __initialize_simulation(self):
        self.client = RemoteAPIClient()
        self.sim    = self.client.getObject('sim')
        self.defaultIdleFps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.dt = self.sim.getSimulationTimeStep()
        self.objectName = '/PioneerP3DX'
        self.rightMotor = self.sim.getObject('/PioneerP3DX/rightMotor')
        self.leftMotor = self.sim.getObject('/PioneerP3DX/leftMotor')
        self.PioneerP3DX = self.sim.getObject('/PioneerP3DX')
        self.Goal = self.sim.getObject('/Goal')
        self.ultrasonicSensor_list = [self.sim.getObject('/PioneerP3DX/ultrasonicSensor['+str(i)+']') for i in range(16)]
        self.objectPosition = self.sim.getObjectPosition(self.PioneerP3DX,-1)
        self.objectOrientation = self.sim.getObjectOrientation(self.PioneerP3DX,-1)
        self.objectQuaternion = self.sim.getObjectQuaternion(self.PioneerP3DX,-1)
        self.client.setStepping(True)

    def __setvariables(self):
        num_point = 200
        self.tf = num_point * self.dt
        self.t  = np.zeros(num_point+1)
        self.xp = np.zeros(num_point+1)
        self.yp = np.zeros(num_point+1)
        self.fp = np.zeros(num_point+1)
        self.up = np.zeros(num_point+1)
        self.om = np.zeros(num_point+1)
    
    def __setp3dXpostion(self):
        pass

    def __setgoalposition(self):
        pass

    def getobservacao(self):
        pass         

    def reset(self):
        self.id = 0
        self.__setvariables()
        self.__setp3dXpostion() #Falta Criar
        self.__setgoalposition() #Falta Criar


    def __drive_p3dx(self, v,w):
        vR=(2*v+w*self.L)/2/self.R
        vL=(2*v-w*self.L)/2/self.R
        return vR,vL    
    
    def __getLyapunov(self, e, alpha, theta):
        v = self.gamma * np.cos(alpha) * e
        omega = self.k * alpha + self.gamma * (np.cos(alpha) * np.sin(alpha) / alpha) * (alpha + self.h * theta)  
        return v, omega 
    
    def minimo(self,a,b):
        if a<b:
            aux = a
        else:
            aux = b

        if aux == 0:
            return 1
        else:
            return aux   
    
    def __state_creator(self, ultrasonicSensor_value, e, alpha):
        state = [] 
        
        sensor = [self.minimo(ultrasonicSensor_value[3], ultrasonicSensor_value[4]),  #N
        self.minimo(ultrasonicSensor_value[12], ultrasonicSensor_value[11]), #S
        self.minimo(ultrasonicSensor_value[7], ultrasonicSensor_value[8]),   #L
        self.minimo(ultrasonicSensor_value[0], ultrasonicSensor_value[15]),  #O
        self.minimo(ultrasonicSensor_value[1], ultrasonicSensor_value[2]),   #NO
        self.minimo(ultrasonicSensor_value[5], ultrasonicSensor_value[6]),   #NE 
        self.minimo(ultrasonicSensor_value[13], ultrasonicSensor_value[14]), #SO
        self.minimo(ultrasonicSensor_value[10], ultrasonicSensor_value[9])]  #SE
        state.append(sensor)
        state.append(e)
        state.append(alpha)
        return state   
    
    def __calcular_recompensa(self, state, alcançou_objetivo):
    # Parâmetros ajustáveis
        peso_posicao = 1.0
        peso_orientacao = 0.5
        peso_colisao = -0.1
        peso_conclusao = 10.0
        distancia_segura = 0.2  # Ajuste conforme necessário

        # Componente de recompensa com base no erro de posição
        recompensa_posicao = 1.0 / (1.0 + state[1])

        # Componente de recompensa com base no erro de orientação
        recompensa_orientacao = 1.0 / (1.0 + state[2])

        # Componente de penalidade por colisão
        recompensa_colisao = 0.0
        for leitura_sensor in state[0]:
            if leitura_sensor < distancia_segura:
                recompensa_colisao += peso_colisao

        # Componente de recompensa pela conclusão da tarefa
        if alcançou_objetivo:
            recompensa_conclusao = peso_conclusao
        else:
            recompensa_conclusao = 0.0

        # Combinação de todas as componentes de recompensa
        recompensa_total = (
            peso_posicao * recompensa_posicao +
            peso_orientacao * recompensa_orientacao +
            peso_colisao * recompensa_colisao +
            peso_conclusao * recompensa_conclusao
        )
        return recompensa_total
    
    def step(self, action):
        done = False
        # Passagem de tempo
        ts = self.sim.getSimulationTime()
        self.id = self.id + 1
        self.t[self.id]=ts
        # Obtenção de posições e valores dos sensores
        P3DXPos = self.sim.getObjectPosition(self.PioneerP3DX,-1)
        P3DXOri = self.sim.getObjectOrientation(self.PioneerP3DX,-1)
        ultrasonicSensor_value = np.zeros(16, dtype=np.float32)
        for i in range(16):
            ultrasonicSensor_value[i] = self.sim.readProximitySensor(self.ultrasonicSensor_list[i],-1)[1]
        self.xp[self.id]=P3DXPos[0]
        self.yp[self.id]=P3DXPos[1]
        self.fp[self.id]=P3DXOri[2]
        GoalPos=self.sim.getObjectPosition(self.Goal,-1)
        GoalOri=self.sim.getObjectOrientation(self.Goal,-1)
        goal_x = GoalPos[0]
        goal_y = GoalPos[1]
        goal_orientation = GoalOri[2]
        # Cálculo de e, alpha e theta
        e_global = np.sqrt((self.xp[self.id] - goal_x)**2 + (self.yp[self.id] - goal_y)**2)
        alpha_global = np.arctan2(goal_y - self.yp[self.id], goal_x - self.xp[self.id]) - self.fp[self.id]
        theta_global = self.fp[self.id] - goal_orientation
        # Criação de estado e cálculo da ação de controle
        observation = self.__state_creator(ultrasonicSensor_value, e_global, alpha_global)
        # Cálculo de velocidades linear e angular
        v, omega = self.__getLyapunov(action[0], action[1], theta_global)
        # Aplicação de velocidades aos motores
        vR,vL = self.__drive_p3dx(v,omega)
        self.sim.setJointTargetVelocity(self.leftMotor,vL)
        self.sim.setJointTargetVelocity(self.rightMotor,vR)
        self.client.step()
        # Verificação de condição de término
        if ts >= self.tf or (e_global < 0.1 and alpha_global<0.9):
            done = True  
        #Calculo de recompensa 
        reward = self.__calcular_recompensa(observation, done)        
        return observation, reward, done