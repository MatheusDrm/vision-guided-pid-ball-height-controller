#!/usr/bin/python3

import RPi.GPIO as GPIO # Controle dos motores 
from gpiozero import Servo, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory # Permite ajustar frequễncia do ciclo de trabalho
from gpiozero import DistanceSensor
import time # Obter tempo de execução
import numpy as np # Manipular vetores em python
# import matplotlib.pyplot as plt
# from scipy.integrate import odeint # Útil para o caso de se querer modelar o sistema e precisar integrar a EDO
import sys
import threading
# import keyboard
import select

from servidorThreads import SocketServer as socket

# factory = PiGPIOFactory()
# servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

class sensorUltrassonico:
    def __init__(self):
        self.sensor_TRIGGER = 7          #Usamos o pino 7 como TRIGGER
        self.sensor_ECHO    = 13           #Usamos o pino 11 como ECHO
        GPIO.setup(self.sensor_TRIGGER,GPIO.OUT)  #Configuramos Trigger como saída
        GPIO.setup(self.sensor_ECHO,GPIO.IN)      #Configuramos Echo como entrada
        GPIO.output(self.sensor_TRIGGER,GPIO.LOW) 
        time.sleep(2)
        self.prevDist = 0
        print("Sensor configurado")

    def getDistance(self): 
        # time.sleep(0.2)
        GPIO.output(self.sensor_TRIGGER,GPIO.HIGH)   # Ativa o gatilho
        time.sleep(0.00001)              # Pausa de 10us
        GPIO.output(self.sensor_TRIGGER,GPIO.LOW)  # Desativa o gatilho

        #start = time.time()              #Guarda o tempo atual mediante time.time()
        while GPIO.input(self.sensor_ECHO)==0:  #Enquanto o sensor não receba sinal...
            start = time.time()          #Mantemos o tempo actual mediante time.time()
        while GPIO.input(self.sensor_ECHO)==1:  #Se o sensor recebe sinal...
            stop = time.time()           #Guarda o tempo actual mediante time.time() noutra variavel
        dt = stop - start             #Obtemos o tempo decorrido entre envío y receção
        distance = round((dt * 34300)/2)   #Distancia é igual ao tempo por velocidade do som partido por 2, pois conta o tempo de ida e volta 
        
        if(distance>100 or distance - self.prevDist > 50):
            distance = self.prevDist

        self.prevDist = distance
        return distance
    

class Motor:
    def __init__(self, tipoMotor):
        if tipoMotor == 'Passo':
            self.pinA = 31
            self.pinB = 33
            self.pinC = 35
            self.pinD = 37

            GPIO.setup(31,GPIO.OUT)
            GPIO.setup(33,GPIO.OUT)
            GPIO.setup(35,GPIO.OUT)
            GPIO.setup(37,GPIO.OUT)

        elif tipoMotor == 'ServoPWM':
            GPIO.setup(12,GPIO.OUT)
            self.servo = GPIO.PWM(12,40)
            self.servo.start(0)

            self.prevAngle = 0
            time.sleep(1)
        
        elif tipoMotor == 'ServoGpiozero':
            self.servo = Servo(18) # tem q ver se passa o pino físico ou o gpio -> Pino 18 (GPIO 1)
        
        elif tipoMotor == 'AngularServo':
            self.servo = AngularServo(18)

        else: 
            print("Tipo de motor inválido")

    # Servo motor
    def changeDutyCycle(self, duty):
        print("Duty: ", duty)
        self.servo.ChangeDutyCycle(duty) # duty de 0 a 20 (10 a 20 se for controlar de 90 graus até 180)
        time.sleep(0.3)
        self.servo.ChangeDutyCycle(0)
        time.sleep(0.7)

    # Servo motor
    def changeAngle(self, deltaAngle):
        angle = self.prevAngle + deltaAngle
        # print("Angle: ", angle)

        # Definindo limites para os angulos
        if (angle<45):
           angle = 45
        elif(angle>180):
           angle = 180

        duty = 2+angle/22.5 # Segundo testes = duty de 2 a 10 - 180 graus 
        self.servo.ChangeDutyCycle(duty) 
        time.sleep(0.3)
        self.servo.ChangeDutyCycle(0)
        time.sleep(0.7)        
        self.prevAngle = angle
        
        return angle
    
    def changePosition(self, val):
        self.servo.value = val # Val de -1 a 1
    
    def changeAngleGpiozero(self, angle):
        self.servo.angle = angle

    #  Motor de passo
    def giro_frente(self, dir, numPassos):
        passo_Frente = ['1001', '1100', '0110', '0011']
        passo_Tras = ['0011', '0110', '1100','1001']
        if dir == True:
            for i in range(numPassos):
                for step in passo_Frente:   
                    GPIO.output(self.pinA, int(step[0]))
                    time.sleep(0.002)
                    GPIO.output(self.pinB, int(step[1]))
                    time.sleep(0.002)
                    GPIO.output(self.pinC, int(step[2]))
                    time.sleep(0.002)
                    GPIO.output(self.pinD, int(step[3]))
                    time.sleep(0.002)


        elif dir == False:
            for i in range(numPassos):
                for step in passo_Tras:
                    GPIO.output(self.pinA, int(step[0]))
                    time.sleep(0.002)
                    GPIO.output(self.pinB, int(step[1]))
                    time.sleep(0.002)
                    GPIO.output(self.pinC, int(step[2]))
                    time.sleep(0.002)
                    GPIO.output(self.pinD, int(step[3]))
                    time.sleep(0.002)


class pidControl:
    def __init__(self, kp, ki, kd):
        # Recebe valores dos ganhos
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.kIList = [0,0,0,0,0]
        self.i = 0
        # Erro anterior
        self.e0 = 0
        # Valor inicial para termo integrativo e tempo
        self.I = 0
        self.t0 = time.time()
        # print("tempo inicial declaracao : ", self.t0)

    def update(self, ref, medicao):
        # print("Tempo inicial: ", self.t0)
        Dt = time.time() - self.t0
        # Cálculo do erro
        e = ref - medicao
        print("Erro: ", e)
        # Cálculo do termo proporcional
        P = self.Kp * e
        print("Proporcional: ", P)
        # Cálculo do termo integral
        self.kIList.append(self.Ki* e * Dt)
        self.I = sum(self.kIList[-5:])
        print("Integrativo: ", self.I)
        # Cálculo do termo derivativo
        D = self.Kd * (e - self.e0) / Dt
        print("Derivativo: ", D)

        res = P + self.I + D
        # Atualiza valor do erro salvo e tempo
        self.e0 = e
        self.t0 = time.time()

        return res



def main():

    GPIO.setmode(GPIO.BOARD) # Colocar placa em modo BOARD -> Pinos físicos

    # Configurações

    # tipoMotor = 'Passo'
    tipoMotor = 'ServoPWM'
    # tipoMotor = 'ServoGpiozero'
    motor = Motor(tipoMotor)

    # ultrassom = sensorUltrassonico()

    ref = 20  # Altura desejada em cm
    medicao = 0
    angle = 0

    # Parametros PID
    kp = 0.6
    ki = 0
    kd = 0.05

    # kp = 0.6
    # ki = 0.001
    # kd = 0.05

    # kp = 1.2
    # ki = 0
    # kd = 0.1


    pid = pidControl(kp =kp, ki= ki, kd=kd)

    
    servidor = socket()
    # servidor.start_server()
    # Startar o servidor paralelamente à execução deste código
    motor.changeDutyCycle(4)
    threading.Thread(target=servidor.start_server).start()
    time.sleep(10)
    print("Iniciando controle")
    key=[0]

    while(True):

        try:
            #Verificar se uma tecla foi apertada -> Pegar novo valor de referência para a altura
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                ref = float(input("Digite altura desejada: "))


            # Obter medicao da altura atual através do servidor (codigo de visao)
            medicao = servidor.returnMesureHeight()
            # print(f'Medicao: {medicao} Altura desejada: {ref}')

            # 2 Opção - Atualiza altura atual da bolinha -> obtenção da medicao com o sensor ultrassonico 
            # medicao = ultrassom.getDistance()
            # print("Medicao: ", medicao, "cm")
            
            # Passa a referencia e a medicao para o PID
            # Obtem uma variação no angulo do servo em função do valor retornado pelo PID
            deltaAng = pid.update(ref, medicao)
            print("DeltaAng antes da limit: ", deltaAng)

            # Limitar saída para  no máximo um incremento de 20 ou -20 graus para evitar movimentos bruscos
            deltaAng = max(-10, min(10, deltaAng))

            # Aplicar variacao do angulo para o motor
            motor.changeAngle(deltaAng)

            time.sleep(0.2)

        except KeyboardInterrupt:
            print("Código encerrado - voltando servo para posicao inicial")
            motor.changeDutyCycle(2)
            time.sleep(100)



    GPIO.cleanup()                       #Limpamos os pinos GPIO e saimos



main()
