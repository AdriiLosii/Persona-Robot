#!/usr/bin/env python

# Importacion de librerias
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

# Variables globales para almacenar distancias de los sensores
D_FL = 0
D_F = 0
D_FR = 0

# Callback para el escaneo laser
def scan_cb(data):
    global D_FL, D_F, D_FR

    # Actualizacion de las variables globales con las distancias minimas en diferentes secciones del escaneo
    D_FL = min(data.ranges[len(data.ranges) * 5 // 8:len(data.ranges) * 7 // 8])
    D_F = min(data.ranges[len(data.ranges) * 3 // 8:len(data.ranges) * 5 // 8])
    D_FR = min(data.ranges[len(data.ranges) * 1 // 8:len(data.ranges) * 3 // 8])

# Funcion principal para evitar obstaculos
def dodge():
    rospy.init_node('nodo_obstacle_avoider')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_cb)
    vel = Twist()
    rate = rospy.Rate(4)

    THRESHOLD_F = 0.75  # Umbral para la distancia frontal
    THRESHOLD_T = 0.5   # Umbral para la diferencia entre distancias laterales

    while not rospy.is_shutdown():
        # Condicion de avance:
        if D_F > THRESHOLD_F:
            vel.linear.x = 0.2  # Velocidad lineal hacia adelante
        else:
            vel.linear.x = 0.0  # Detiene el movimiento

        # Condicion de giro:
        if D_FL > D_FR and abs(D_FL - D_FR) > THRESHOLD_T:  # Gira hacia la izquierda si hay mas espacio
            vel.angular.z = -0.3  # Velocidad angular en sentido antihorario
        elif D_FL < D_FR and abs(D_FL - D_FR) > THRESHOLD_T:  # Gira hacia la derecha si hay mas espacio
            vel.angular.z = 0.3  # Velocidad angular en sentido horario
        else:
            vel.angular.z = 0.0  # Detiene el giro

        pub.publish(vel)  # Publica los comandos de velocidad

if __name__ == '__main__':
    dodge()  # Llama a la funcion principal para evitar obstaculos