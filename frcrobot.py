import wpilib
from wpilib.drive import MecanumDrive

class MyRobot(wpilib.TimedRobot):
    # Portlar tanımlandı (Portları örnek yazdım,değişmeli)
    solOnMotorPort = 1
    solArkaMotorPort = 3
    sagOnMotorPort = 2
    sagArkaMotorPort = 4
    joystickPort = 0
    mecanumMotorSpeedSol = 0.3
    mecanumMotorSpeedSag = -0.3

    def robotInit(self):

        #Joystick tanımlandı
        self.stick = wpilib.Joystick(self.joystickPort)
        #Motorlar tanımlandı
        self.solOnMotor = wpilib.PWMVictorSPX(self.solOnMotorPort)
        self.solArkaMotor = wpilib.PWMVictorSPX(self.solArkaMotorPort)
        self.sagOnMotor = wpilib.PWMVictorSPX(self.sagOnMotorPort)
        self.sagArkaMotor = wpilib.PWMVictorSPX(self.sagArkaMotorPort)
        #Motor Hız Tanımlamaları
        self.sagArkaMotor.set(0.1)
        self.sagOnMotor.set(0.1)
        self.solArkaMotor.set(0.1)
        self.solOnMotor.set(0.1)

        
        #wpilib kütüphanesi kullanılarak motorlar gruplanıyor
        self.drive = MecanumDrive(
            self.solOnMotor,
            self.solArkaMotor,
            self.sagOnMotor,
            self.sagArkaMotor,
        )


        self.drive.setExpiration(0.01)

    def crabwalk(self):
        if self.stick.getPOV(0) == 0:
            self.solArkaMotor.set(self.mecanumMotorSpeedSol)
            self.solOnMotor.set(self.mecanumMotorSpeedSol)
            self.sagArkaMotor.set(self.mecanumMotorSpeedSag)
            self.sagOnMotor.set(self.mecanumMotorSpeedSag)

        elif self.stick.getPOV(0) == 180:
            self.solArkaMotor.set(-self.mecanumMotorSpeedSol)
            self.solOnMotor.set(-self.mecanumMotorSpeedSol)
            self.sagArkaMotor.set(-self.mecanumMotorSpeedSag)
            self.sagOnMotor.set(-self.mecanumMotorSpeedSag)

        elif self.stick.getPOV(0) == 90:
            self.solArkaMotor.set(-self.mecanumMotorSpeedSol)
            self.solOnMotor.set(self.mecanumMotorSpeedSol)
            self.sagArkaMotor.set(self.mecanumMotorSpeedSag)
            self.sagOnMotor.set(-self.mecanumMotorSpeedSag)

        elif self.stick.getPOV(0) == 270:
            self.solArkaMotor.set(self.mecanumMotorSpeedSol)
            self.solOnMotor.set(-self.mecanumMotorSpeedSol)
            self.sagArkaMotor.set(-self.mecanumMotorSpeedSag)
            self.sagOnMotor.set(self.mecanumMotorSpeedSag)

        elif self.stick.getPOV(0) == 45:
            self.solOnMotor.set(self.mecanumMotorSpeedSol)
            self.sagArkaMotor.set(self.mecanumMotorSpeedSag)

        elif self.stick.getPOV(0) == 135:
            self.sagOnMotor.set(-self.mecanumMotorSpeedSag)
            self.solArkaMotor.set(-self.mecanumMotorSpeedSol)

        elif self.stick.getPOV(0) == 225:
            self.solOnMotor.set(-self.mecanumMotorSpeedSol)
            self.sagArkaMotor.set(-self.mecanumMotorSpeedSag)

        elif self.stick.getPOV(0) == 315:
            self.solArkaMotor.set(self.mecanumMotorSpeedSol)
            self.sagOnMotor.set(self.mecanumMotorSpeedSag)


    def teleopInit(self):
        self.drive.setSafetyEnabled(False)

    def teleopPeriodic(self):
        self.drive.driveCartesian(
            self.stick.getX(), (-self.stick.getY()), self.stick.getZ(), 0
        )
        self.crabwalk()

if __name__ == "__main__":
    wpilib.run(MyRobot)
