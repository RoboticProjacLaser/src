# commandeMoteur.py

import serial

# Definition classe commandeMoteur
class cmdMoteur:

   # definition de la methode speciale __init__ (constructeur)
   def __init__(self):
	self.ser = serial.Serial("/dev/ttyAMA0")

   # definition de la methode miseEnForme()
   def Publish(self, Vx, Omega):

	 #Cast des valeurs
        if Vx > 127:
            Vx = 127
        elif Vx < (-127):
            Vx = -127
        if Omega > 63:
            Omega = 63
        elif Omega <(-63):
            Omega = -63

        octet0 = str(chr(255))
	octet1 = str(chr(Vx)) #On impose nos valeur de 0 a 255
	octet2 = str(chr(Omega))#On impose nos valeur de 0 a 128
	octet3 = str(chr((Vx+Omega)&127))
	cmd = octet0 + octet1 + octet2 + octet3

	self.ser.write(cmd)
