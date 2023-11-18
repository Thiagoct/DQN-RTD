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
        self.collision_distance = 0.1
        #Inicialização do simulador e objetos
        self.__initialize_simulation()
        #Inicializar ambiente
        self.id = 0
#Conexeção Coppelia------------------------------------------------------
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
#Controle do estado do Ambiente------------------------------------------
    def __setvariables(self):
        num_point = 3600
        self.tf = num_point * self.dt
        self.t  = np.zeros(num_point+1)
        self.xp = np.zeros(num_point+1)
        self.yp = np.zeros(num_point+1)
        self.fp = np.zeros(num_point+1)
        self.up = np.zeros(num_point+1)
        self.om = np.zeros(num_point+1)

    def __getSensorvalues_and_positions(self):
        P3DXPos = self.sim.getObjectPosition(self.PioneerP3DX,-1)
        P3DXOri = self.sim.getObjectOrientation(self.PioneerP3DX,-1)
        self.ultrasonicSensor_value = np.zeros(16, dtype=np.float32)
        for i in range(16):
            self.ultrasonicSensor_value[i] = self.sim.readProximitySensor(self.ultrasonicSensor_list[i],-1)[1]
        self.xp[self.id]=P3DXPos[0]
        self.yp[self.id]=P3DXPos[1]
        self.fp[self.id]=P3DXOri[2]
        GoalPos=self.sim.getObjectPosition(self.Goal,-1)
        GoalOri=self.sim.getObjectOrientation(self.Goal,-1)
        self.goal_x = GoalPos[0]
        self.goal_y = GoalPos[1]
        self.goal_orientation = GoalOri[2]
    
    def __minimo(self,a,b):
        if a<b:
            aux = a
        else:
            aux = b
        if aux == 0:
            return 1
        else:
            return aux   
        
    def __getErros(self):
        self.e_global = np.sqrt((self.xp[self.id] - self.goal_x)**2 + (self.yp[self.id] - self.goal_y)**2)
        self.alpha_global = np.arctan2(self.goal_y - self.yp[self.id], self.goal_x - self.xp[self.id]) - self.fp[self.id]
        self.theta_global = self.fp[self.id] - self.goal_orientation

    def __state_creator(self):
        # Obtenção de posições e valores dos sensores
        self.__getSensorvalues_and_positions()
        # Cálculo de e, alpha e theta
        self.__getErros()
        state = [] 
        sensor = [self.__minimo(self.ultrasonicSensor_value[3], self.ultrasonicSensor_value[4]),  #N
        self.__minimo(self.ultrasonicSensor_value[12], self.ultrasonicSensor_value[11]),          #S
        self.__minimo(self.ultrasonicSensor_value[7], self.ultrasonicSensor_value[8]),            #L
        self.__minimo(self.ultrasonicSensor_value[0], self.ultrasonicSensor_value[15]),           #O
        self.__minimo(self.ultrasonicSensor_value[1], self.ultrasonicSensor_value[2]),            #NO
        self.__minimo(self.ultrasonicSensor_value[5], self.ultrasonicSensor_value[6]),            #NE 
        self.__minimo(self.ultrasonicSensor_value[13], self.ultrasonicSensor_value[14]),          #SO
        self.__minimo(self.ultrasonicSensor_value[10], self.ultrasonicSensor_value[9])]           #SE
        for data in sensor:
            state.append(data)
        state.append(self.e_global)
        state.append(self.alpha_global)
        return np.array(state)  
        
    def __setp3dXpostion(self):
        pass

    def __setgoalposition(self):
        pass

    def reset(self):
        self.id = 0
        self.__setvariables()
        self.__setp3dXpostion() #Falta Criar
        self.__setgoalposition() #Falta Criar
        return self.__state_creator()
#Contre da ação aplicada------------------------------------------------  
    def __drive_p3dx(self, v,w):
        vR=(2*v+w*self.L)/2/self.R
        vL=(2*v-w*self.L)/2/self.R
        return vR,vL    
    
    def __getLyapunov(self, e, alpha, theta):
        v = self.gamma * np.cos(alpha) * e
        omega = self.k * alpha + self.gamma * (np.cos(alpha) * np.sin(alpha) / alpha) * (alpha + self.h * theta)  
        return v, omega 

    def __conversor_alpha(self, action):
        alpha_list = [np.pi/2, 0.00001, -np.pi/2]
        return alpha_list[action]
#Lógica de recompensa---------------------------------------------------  
    def __calcular_recompensa(self, collision, reach_goal):
        if collision:
            reward = -1
        elif reach_goal:
            reward = 1
        else:
            reward = 0.2-self.e_global*0.001         
        return reward
#Passo de simulação--------------------------------------------------------  
# Detector de colisão
    def __event_collision(self):
        sensors = self.__state_creator()[0:7]
        collision = False
        for sensor in sensors:
            if self.collision_distance > sensor:
                collision = True     
        return collision
    
    def __event_reach_goal(self):
        reach_goal = False
        if self.e_global < 0.1 and self.alpha_global<0.9:
            reach_goal = True    
        return reach_goal
    
    def __event_maximum_steps(self):
        maximum_steps = False
        if self.t[self.id] >= self.tf:
            maximum_steps = True
        return maximum_steps
# Detector de chegada ao objetivo  
    def step(self, action):
        done = False
        # Passagem de tempo
        ts = self.sim.getSimulationTime()
        self.id = self.id + 1
        self.t[self.id]=ts
        # Criação de estado e cálculo da ação de controle
        observation = self.__state_creator()
        # Cálculo de velocidades linear e angular
        v, omega = self.__getLyapunov(0.9, self.__conversor_alpha(action), self.theta_global)
        # Aplicação de velocidades aos motores
        vR,vL = self.__drive_p3dx(0.2,omega)
        self.sim.setJointTargetVelocity(self.leftMotor,vL)
        self.sim.setJointTargetVelocity(self.rightMotor,vR)
        self.client.step()
        # Verificação de condição de término
        collision     = self.__event_collision()
        reach_goal    = self.__event_reach_goal()
        maximum_steps = self.__event_maximum_steps()
        if reach_goal or maximum_steps:
            done = True  
        #Calculo de recompensa 
        reward = self.__calcular_recompensa(collision, reach_goal)        
        return observation, reward, done