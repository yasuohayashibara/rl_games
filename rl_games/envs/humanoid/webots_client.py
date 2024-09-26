import socket
import sys
sys.path.append('./envs')
import messages_pb2

class Webots():
    def __init__(self, host="192.168.123.100", port=10021):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        data = self.sock.recv(8)
        if data == b'Welcome\x00':
            print("Connected to "+host+":"+str(port))
        self.sensorMeasurements = messages_pb2.SensorMeasurements()

    def initializeSensors(self):
        actuatorRequests = messages_pb2.ActuatorRequests()
        sensor_time_step = messages_pb2.SensorTimeStep()
        sensor_time_step.name = "accelerometer"
        sensor_time_step.timeStep = 8
        actuatorRequests.sensor_time_steps.append(sensor_time_step) 
        sensor_time_step.name = "gyro"
        sensor_time_step.timeStep = 8
        actuatorRequests.sensor_time_steps.append(sensor_time_step)
        data = actuatorRequests.SerializeToString()
        length = len(data)
        byte_data = length.to_bytes(4, byteorder='big')
        self.sock.sendall(byte_data+data)

    def measureSensors(self):
        data = self.sock.recv(4)
        length = int.from_bytes(data, byteorder='big')
        data = self.sock.recv(length)
        try:
            self.sensorMeasurements.ParseFromString(data)
        except Exception as e:
            print("Error: ", e)

    def setAngle(self, angles):
        joint_name = ["right_ankle_roll_joint", "right_ankle_pitch_joint", "right_knee_pitch_joint", "right_waist_pitch_joint", "right_waist_roll_joint [hip]", "right_waist_yaw_joint", "right_shoulder_roll_joint", "right_shoulder_pitch_joint [shoulder]", "right_elbow_pitch_joint", "left_ankle_roll_joint", "left_ankle_pitch_joint", "left_knee_pitch_joint", "left_waist_pitch_joint", "left_waist_roll_joint [hip]", "left_waist_yaw_joint", "left_shoulder_pitch_joint [shoulder]", "left_shoulder_roll_joint", "left_elbow_pitch_joint", "head_yaw_joint"]
        actuatorRequests = messages_pb2.ActuatorRequests()
        motor_position = messages_pb2.MotorPosition()
        for name, angle in zip(joint_name, angles):
            motor_position.name = name
            motor_position.position = angle
            actuatorRequests.motor_positions.append(motor_position)
        data = actuatorRequests.SerializeToString()
        length = len(data)
        byte_data = length.to_bytes(4, byteorder='big')
        self.sock.sendall(byte_data+data)

if __name__ == "__main__":
    webots = Webots()
    webots.initializeSensors()
    angles = [1.57]*19
    while True:
        webots.measureSensors()
        print(webots.sensorMeasurements.time)
        print(webots.sensorMeasurements.accelerometers)
        print(webots.sensorMeasurements.gyros)
        print(webots.sensorMeasurements.object_positions)
        webots.setAngle(angles)
