#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from people_msgs.msg import PositionMeasurementArray
from sensor_msgs.msg import LaserScan
import numpy as np
import math


list_persons = []
list_means = []
list_priorities = []
D_F = 0
vel = Twist()


# Funcion que determina si una persona esta en movimiento o estatica
def esta_en_movimiento(list_means, num, umbral = 0.1):
    # Si la lista de medias tiene un suficientes datos
    if(len(list_means[num])>50):
        # Calculamos la desviacion tipica de los ultimos 50 valores medidos
        std_d = np.std(list_means[num][-49:-1])
    # Si no hay suficientes datos para determinar el resultado
    else:
        return False

    # Comparamos el valor de destiacion tipica con el umbral definido para determinar si:
    # Se encuentra estatica
    if(std_d < umbral):
        # Disminuimos en 1 la prioridad de la persona por cada deteccion estatica
        list_priorities[num] = list_priorities[num]-1
        return False
    # Se encuentra en movimiento
    else:
        # Aumentamos en 1 la prioridad de la persona por cada deteccion en movimiento
        list_priorities[num] = list_priorities[num]+1
        return True

# Funcion para calcular el angulo de la persona
def angulo(pos_x, pos_y):
    if (pos_x == 0 and pos_y > 0):
        return 0
    elif (pos_x == 0 and pos_y < 0):
        return 180
    elif (pos_x > 0 and pos_y == 0):
        return 90
    elif (pos_x < 0 and pos_y == 0):
        return 270
    else:
        # Angulo en grados
        angulo = math.degrees(math.atan2(pos_y, pos_x))
        if angulo < 0:
            return angulo+360
        else:
            return angulo

# Funcion que determina el movimiento del turtlebot segun el angulo dado
def logic(angulo):
    if(D_F > 0.75):
        if (340 <= angulo or angulo <= 20):
            vel.linear.x = 0.4
            vel.angular.z = 0.0
        elif (180 < angulo < 340):
            vel.linear.x = 0.25
            vel.angular.z = -1.5
        elif (20 < angulo < 180):
            vel.linear.x = 0.25
            vel.angular.z = 1.5

    else:
        vel.linear.x = 0.0
        vel.angular.z = 0.0

    pub_robot.publish(vel)

# Callback de leg_detector
def position_measurement_array_cb(data):
    global list_priorities

    # Recorremos la lista de personas detectadas
    for position_measurement in data.people:
        # Obtenemos el nombre de la persona y sus posiciones x e y
        nombre = position_measurement.name
        posicion_x = position_measurement.pos.x
        posicion_y = position_measurement.pos.y

        # Agregamos el nombre a la lista de nombres si no esta en ella
        if(nombre not in list_persons):
            list_persons.append(nombre)

        # Recorremos la lista de personas almacenada
        for person in list_persons:
            # Para la persona detectada...
            if nombre==person:
                # Obtenemos el numero identificativo de esa persona
                numero=int(person[-1])

                # Si el numero de la persona es mayor que la longitud
                # de la lista de medias, rellenamos con 0s hasta su posicion
                if numero+1>len(list_means):
                    for i in range(numero-len(list_means)+1):
                        list_means.append(list())
                        list_priorities.append(0)

                # Agregamos la media de las coordenadas X e Y a la lista de medias de esa persona
                # Trabajamos con la media de las dos coordenadas en lugar de las coordenadas en si,
                # porque se puede dar el caso de que la persona se mueva completamente en horizontal o vertical
                list_means[numero].append((posicion_x+posicion_y)/2)

                # Llamamos a la funcion que determina si la persona esta en movimiento a partir de
                # la lista de medias y su numero identificativo
                if (esta_en_movimiento(list_means, numero)):
                    # Si la persona detectada en movimiento tiene una prioridad mayor al umbral
                    if (list_priorities[numero]>10):
                        # Llamamos a la funcion que realiza el movimiento del turtlebot segun
                        # el angulo calculado mediante las coordenadas de la persona detectada
                        logic(angulo(posicion_x, posicion_y))
                        print('Follow {}?: {} -> Angle: {} degrees with priority: {}'.format(person, True, angulo(posicion_x, posicion_y), list_priorities[numero]))
                # La persona no se encuentra en movimiento
                else:
                    print('Follow {}?: {}'.format(person, False))
        
    print("****************************************")

# Callback del scan
def scan_cb(data):
    global D_F
    # Dividimos la circunferencia de medidas obtenidas por el LiDAR en 16 segmentos iguales
    # de los cuales seleccionaremos los 3 frontales, obteniendo asi un arco frontal en el que
    # detectaremos la distancia minima medida.
    D_F = min(data.ranges[len(data.ranges)/16*7:len(data.ranges)/16*9])


if __name__ == '__main__':
    rospy.init_node('leg_filter_node')
    pub_robot = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, position_measurement_array_cb)
    rospy.Subscriber('/scan', LaserScan, scan_cb)
    rospy.spin()