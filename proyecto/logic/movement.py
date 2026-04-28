import math
from geometry_msgs.msg import Twist

def calcular_rotacion(theta_actual, theta_objetivo, vel_angular_max=0.5, tolerancia=0.05):
    cmd = Twist()
    error = theta_objetivo - theta_actual
    error = math.atan2(math.sin(error), math.cos(error))

    if abs(error) <= tolerancia:
        return cmd, True

    # Proporcional puro — sin piso mínimo para evitar oscilación
    factor = min(abs(error) / 0.5, 1.0)
    vel = vel_angular_max * factor

    cmd.angular.z = -(vel if error > 0 else -vel)
    return cmd, False

def calcular_movimiento_relativo(tiempo_transcurrido, dist_x, dist_y, distancias_direccion, dist_segura=0.3, vel_lineal=0.4):
    cmd = Twist()
    
    # 1. Distancia total geométrica que queremos recorrer
    dist_total = math.sqrt(dist_x**2 + dist_y**2)
    
    # Si nos piden movernos 0, terminamos inmediatamente
    if dist_total < 0.001:
        return cmd, 'COMPLETADO'

    # 2. Cinemática de tiempo: ¿Cuántos segundos tardamos a vel_lineal constante?
    tiempo_necesario = dist_total / vel_lineal

    # 3. Condición de llegada por cronómetro
    if tiempo_transcurrido >= tiempo_necesario:
        return cmd, 'COMPLETADO'

    # ==========================
    # SISTEMA ANTICHOQUES DIRECCIONAL
    # ==========================
    # Ahora 'distancias_direccion' solo contiene los datos del Lidar hacia donde nos movemos
    min_dist = min(distancias_direccion) if distancias_direccion else float('inf')
    if min_dist <= dist_segura:
        return cmd, 'BLOQUEADO'
    
    # 4. Velocidad constante apuntando a la dirección correcta
    cmd.linear.x = (dist_x / dist_total) * vel_lineal
    cmd.linear.y = (dist_y / dist_total) * vel_lineal
    
    return cmd, 'EN_RUTA'