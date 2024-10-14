import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from time import sleep

# Configuración de pines para los motores (solo avance)
motor1_forward = 12
motor2_forward = 13

# Configuración de GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([motor1_forward, motor2_forward], GPIO.OUT)

# Configuración de PWM solo para avanzar
pwm_freq = 100  # Frecuencia del PWM en Hz
pwm_motor1_forward = GPIO.PWM(motor1_forward, pwm_freq)
pwm_motor2_forward = GPIO.PWM(motor2_forward, pwm_freq)

pwm_motor1_forward.start(0)
pwm_motor2_forward.start(0)

# Configuración de la cámara con Picamera2
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 240)})
picam2.configure(config)
picam2.start()

# Velocidad base y factor de corrección
base_speed = 20  # Velocidad base en % del duty cycle (0-100)
correction_factor = 2  # Factor de corrección para ajuste fino

# Relación píxeles-centímetros, necesitarás calibrarlo
known_width_cm = 3
pixels_per_cm = None

def set_motor_speeds(left_speed, right_speed):
    """Configura la velocidad de los motores utilizando PWM solo en avance."""
    left_duty_cycle = max(0, min(100, left_speed))
    right_duty_cycle = max(0, min(100, right_speed))

    pwm_motor1_forward.ChangeDutyCycle(left_duty_cycle)
    pwm_motor2_forward.ChangeDutyCycle(right_duty_cycle)

def process_image(image):
    """Procesa la imagen capturada para detectar la línea."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Umbral para detectar la línea negra
    _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
    
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        line = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(line)
        
        global pixels_per_cm
        if pixels_per_cm is None:
            pixels_per_cm = w / known_width_cm
        
        image_center = 320 / 2
        line_center = x + (w / 2)
        deviation = line_center - image_center
        deviation_cm = deviation / pixels_per_cm
        
        return deviation_cm, image
    else:
        return 0, image

def main():
    """Función principal para capturar imágenes y ajustar los motores."""
    try:
        while True:
            image = picam2.capture_array()

            deviation_cm, annotated_image = process_image(image)
            
            if deviation_cm is not None:
                if deviation_cm > 0:
                    motor_left_speed = base_speed + correction_factor * deviation_cm
                    motor_right_speed = base_speed - correction_factor * deviation_cm
                elif deviation_cm < 0:
                    motor_left_speed = base_speed - correction_factor * abs(deviation_cm)
                    motor_right_speed = base_speed + correction_factor * abs(deviation_cm)
                else:
                    motor_left_speed = base_speed
                    motor_right_speed = base_speed
                
                set_motor_speeds(motor_left_speed, motor_right_speed)
            else:
                set_motor_speeds(0, 0)

            # En lugar de cv2.imshow, aquí va el código para la representación de terminal
            print(f"Desviación: {deviation_cm:.2f} cm")
            
            sleep(0.1)

    except KeyboardInterrupt:
        pass

    finally:
        pwm_motor1_forward.stop()
        pwm_motor2_forward.stop()
        GPIO.cleanup()
        picam2.close()
        print("Cámara cerrada y GPIO limpiado correctamente.")

if __name__ == "__main__":
    main()
