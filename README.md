# DQN-RTD
 
## Descrição
O ambiente de treinamento foi projetado para capacitar um agente de aprendizado por reforço a controlar um robô de tração diferencial simulado no CoppeliaSim.

A simulação consiste em um modelo do P3DX chegar a um ponto espécifico do mapa com uma determinada orientação. Dentro do mapa pode conter obstaculos que seram detectados com os sensores ultrasonicos presentes no robô que são pré-processados 

|Nome | Valores |
|-------------|-------------|
|Espaço de Ação | [Continuo, Continuo] |
| Forma de observação  | (8, 1, 1,)  |
| Observação Alta  | [1.0, inf, Pi]  |
| Observação Baixa  | [0.0, -inf, -Pi]  |
## Espaço de ações
O espaço de ações é um np.array na forma (1, 1) no qual o primeiro index representa o erro de posição que será passado para o controlador de Lyapunov e o segundo index o erro de orientação. O intervalo de valores são [-inf, inf], [-Pi, Pi] respectivamente. 
## Espaço de observações
A observação é um np.array na forma (8, 1, 1), valores correspondentes as leituras de 16 sensores ultrasonicos pré-processadas representando a distancias de objetos mapeados usando a rosa dos ventos, além de um erro de posição e orientação a um objetivo.

| Index. | Observação | Mínimo | Máx. | Sensor |
|-------------|-------------|-------------|-------------|-------------|
| 00  | N | 0  | 1 | Sim |
| 01  | S | 0  | 1 | Sim |
| 02  | L | 0  | 1 | Sim ||
| 03  | O | 0  | 1 | Sim ||
| 04  | NO | 0  | 1 | Sim ||
| 05  | NE | 0  | 1 | Sim ||
| 06  | SO | 0  | 1 | Sim ||
| 07  | SE | 0  | 1 | Sim ||
| 1  | e_go | -inf  | inf  | Não ||
| 2  | alpha_go | -np.pi  | np.pi  | Não ||

## Recompensas
A recompensa por ação leva em consideração os seguintes criterios
* Recompensa por Alcance do Destino
* Punição por Colisões
* Recompensa por Evitar Obstáculos
* Punição por Ações Ineficazes
* Tempo de Convergência

## Estado Inicial
A posição e orientação do P3DX e objetivos são aleatorias (Evitando a posição de objetos pré-existentes).

## Fim do episodio 
O episódio termina se ocorrer alguma das seguintes situações:

* Terminação: Posição e Orientação do P3DX não se altera por mais de 5 unidades de tempo.

* Terminação: Posição e Orientação do P3DX tiverem um erro menor que 0.01.

* Truncamento: A duração do episodio for maior que 1000 unidades de tempo
