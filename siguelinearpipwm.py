#Pruebas:
#Calibración: Es posible que necesites ajustar el valor de pixels_per_cm para que coincida correctamente con el ancho de la línea en tu entorno físico.
#Acciones de los Motores: Verifica que los motores respondan como se espera según la dirección de la línea.
#Control de Velocidad: Observa si la corrección de velocidad es suave y precisa o si requiere ajustes en correction_factor.

from picamera2 import Picamera2
import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep

# Configuración de pines para los motores
motor1_forward = 12
#motor1_backward = 22
motor2_forward = 13
#motor2_backward = 24

# Configuración de GPIO
GPIO.setmode(GPIO.BCM)
#GPIO.setup([motor1_forward, motor1_backward, motor2_forward, motor2_backward], GPIO.OUT)
GPIO.setup([motor1_forward, motor2_forward], GPIO.OUT)
# Configuración de PWM
pwm_freq = 50  # Frecuencia del PWM en Hz
pwm_motor1_forward = GPIO.PWM(motor1_forward, pwm_freq)
#pwm_motor1_backward = GPIO.PWM(motor1_backward, pwm_freq)
pwm_motor2_forward = GPIO.PWM(motor2_forward, pwm_freq)
#pwm_motor2_backward = GPIO.PWM(motor2_backward, pwm_freq)

# Inicia PWM con duty cycle de 0%
pwm_motor1_forward.start(0)
#pwm_motor1_backward.start(0)
pwm_motor2_forward.start(0)
#pwm_motor2_backward.start(0)

# Configuración de la cámara con Picamera2
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 240)})
picam2.configure(config)
picam2.start()

# Velocidad base y factor de corrección
base_speed = 13
correction_factor = 22

# Relación píxeles-centímetros, necesitarás calibrarlo
known_width_cm = 3
pixels_per_cm = 100

def set_motor_speeds(left_speed, right_speed):
    """Configura la velocidad de los motores utilizando PWM."""
    left_duty_cycle = max(0, min(100, left_speed))  # Limita el rango de 0 a 100%
    right_duty_cycle = max(0, min(100, right_speed))

    print("Velocidad: Derecha {} Izquierda {}".format(left_duty_cycle,right_duty_cycle))
    if left_speed > 0:
        pwm_motor1_forward.ChangeDutyCycle(left_duty_cycle)
 #       pwm_motor1_backward.ChangeDutyCycle(0)
    else:
        pwm_motor1_forward.ChangeDutyCycle(0)
#        pwm_motor1_backward.ChangeDutyCycle(left_duty_cycle)

    if right_speed > 0:
        pwm_motor2_forward.ChangeDutyCycle(right_duty_cycle)
 #       pwm_motor2_backward.ChangeDutyCycle(0)
    else:
        pwm_motor2_forward.ChangeDutyCycle(0)
  #      pwm_motor2_backward.ChangeDutyCycle(right_duty_cycle)
        
def process_image(image):
    """Procesa la imagen capturada para detectar la línea."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        line = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(line)
        
        global pixels_per_cm
        if pixels_per_cm is None:
            pixels_per_cm = w / known_width_cm  # Calibración inicial
        
        image_center = 320 / 2
        line_center = x + (w / 2)
        deviation = line_center - image_center
        deviation_cm = deviation / pixels_per_cm
        
        return deviation_cm
    else:
        return 0


def print_bar(deviation_cm):
    """Imprime una barra que representa la posición de la línea en la terminal."""
    if deviation_cm is None:
      devi = 0
    else:
      devi = deviation_cm
    bar_length = 50  # Longitud de la barra en caracteres
    center_pos = int((devi + 3) / 6 * bar_length)  # Ajusta 3 cm a la mitad

    bar = [' '] * bar_length
    bar[center_pos] = '|'
    print(''.join(bar))

def main():
    """Función principal para capturar imágenes y ajustar los motores."""
    try:
        while True:
            image = picam2.capture_array()
            print('tomo imagen')
            deviation_cm = process_image(image)
            #Dentro del bucle principal, en lugar o además de `imshow`, imprime la barra:
            print_bar(deviation_cm)
            print("corrijo desviacion ",deviation_cm)
            if deviation_cm is not None:
                if deviation_cm > 0:
                    motor_left_speed = base_speed + correction_factor * deviation_cm
                    motor_right_speed = base_speed - correction_factor * deviation_cm
                    print("giro derecha")
                elif deviation_cm < 0:
                    motor_left_speed = base_speed - correction_factor * abs(deviation_cm)
                    motor_right_speed = base_speed + correction_factor * abs(deviation_cm)
                    print("giro izquierda")
                else:
                    motor_left_speed = base_speed
                    motor_right_speed = base_speed
                    print("mantengo direccion")
                
                set_motor_speeds(motor_left_speed, motor_right_speed)
                print("corrijo velocidad")
            else:
                set_motor_speeds(0, 0)
                print("me detengo")
            
            sleep(0.1)

    except KeyboardInterrupt:
        set_motor_speeds(0, 0)
        pwm_motor1_forward.stop()
   #     pwm_motor1_backward.stop()
        pwm_motor2_forward.stop()
    #    pwm_motor2_backward.stop()
        GPIO.cleanup()
        picam2.close()
        print("Cámara cerrada y GPIO limpiado correctamente.")

if __name__ == "__main__":
    main()
