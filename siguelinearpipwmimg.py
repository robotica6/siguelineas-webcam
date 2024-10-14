import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from time import sleep

# Configuración de pines para los motores
motor1_forward = 12
#motor1_backward = 12
motor2_forward = 13
#motor2_backward = 13

# Configuración de GPI
GPIO.setmode(GPIO.BCM)
#GPIO.setup([motor1_forward, motor1_backward, motor2_forward, motor2_backward], GPIO.OUT)
GPIO.setup([motor1_forward, motor2_forward], GPIO.OUT)

# Configuración de PWM
pwm_freq = 50  # Frecuencia del PWM en Hz
pwm_motor1_forward = GPIO.PWM(motor1_forward, pwm_freq)
#pwm_motor1_backward = GPIO.PWM(motor1_backward, pwm_freq)
pwm_motor2_forward = GPIO.PWM(motor2_forward, pwm_freq)
#pwm_motor2_backward = GPIO.PWM(motor2_backward, pwm_freq)

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
pixels_per_cm = None

def set_motor_speeds(left_speed, right_speed):
    """Configura la velocidad de los motores utilizando PWM."""
    left_duty_cycle = max(0, min(100, left_speed))
    right_duty_cycle = max(0, min(100, right_speed))

    print("Velocidad: Derecha {} Izquierda {}".format(left_duty_cycle,right_duty_cycle))
    if left_speed > 0:
        pwm_motor1_forward.ChangeDutyCycle(left_duty_cycle)
     #   pwm_motor1_backward.ChangeDutyCycle(0)
    else:
        pwm_motor1_forward.ChangeDutyCycle(0)
  #      pwm_motor1_backward.ChangeDutyCycle(left_duty_cycle)

    if right_speed > 0:
        pwm_motor2_forward.ChangeDutyCycle(right_duty_cycle)
 #       pwm_motor2_backward.ChangeDutyCycle(0)
    else:
        pwm_motor2_forward.ChangeDutyCycle(0)
      #  pwm_motor2_backward.ChangeDutyCycle(right_duty_cycle)
        

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
        
        # Convertir las coordenadas a enteros antes de dibujar la línea
        line_center_int = int(line_center)
        y_int = int(y)
        h_int = int(h)

        # Dibuja la línea detectada en la imagen
        cv2.line(image, (line_center_int, y_int), (line_center_int, y_int + h_int), (0, 255, 0), 2)
        return deviation_cm, image
    else:
        return 0, image



def main():
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

            # Muestra la imagen en una ventana
            cv2.imshow("Camera View", annotated_image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            sleep(0.1)

    except KeyboardInterrupt:
        pass

    finally:
        set_motor_speeds(0, 0)
            
        pwm_motor1_forward.stop()
        #pwm_motor1_backward.stop()
        pwm_motor2_forward.stop()
       # pwm_motor2_backward.stop()
        GPIO.cleanup()
        picam2.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
