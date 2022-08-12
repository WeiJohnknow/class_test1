
from PyQt5.QtCore import QThread, Qt
from PyQt5 import QtWidgets
import time
from dynamixel_sdk import *
import numpy as np




#import keyboard

def data2byte(data):
    return [DXL_LOBYTE(data), DXL_HIBYTE(data)]

def data4byte(data):
    return [DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data))]

def data6byte(data,data1,data2):
    return data2byte(data) + data2byte(data2) + data2byte(data3)

def data8byte(data,data2):
    return data4byte(data) + data4byte(data2)

class Motor(QThread):
    def __init__(self, ID, angle):
        super().__init__()
        self.ID = ID
        self.vels = 20
        self.Now_vels = []
        self.poss = 2048
        self.Now_poss = []
        self.PWM = 885
        self.P = 850
        self.I = 0
        self.D = 0
        self.angle_list = []
    def Set_P(self, Pch):
        self.P = P
    def Set_I(self, I):
        self.I = I
    def Set_D(self, D):
        self.D = D
    def Set_PWM(self, PWM):
        self.PWM = PWM
    def Set_angle(self, angle):
        self.angle_list.append(angle)
    def Read_vels(self):
        return  self.Now_vels
    def Read_poss(self):
        return  self.Now_poss
    
class All_Motor_body(QThread):
    def __init__(self, ch):
        super().__init__()
        self.ch = ch
        self.Data_cur_value = []
        self.Data_vel_value = []
        self.Data_pos_value = []
        self.Data_cur ={}
        self.Data_vel ={}
        self.Data_pos ={}
        self.Data_P_value = []
        self.Data_I_value = []
        self.Data_D_value = []
        self.Data_P ={}
        self.Data_I ={}
        self.Data_D ={}
        self.Data_PWM_value = []
        self.Data_PWM ={}
        self.Data_Unit_cur ={}
        self.Data_Unit_vel ={}
        address_TORQUE_ENABLE = 64
        TORQUE_ENABLE = 1
        TORQUE_DISABLE = 0
        PROTOCOL_VERSION      = 2.0
        BAUDRATE              = 3000000             
        DEVICENAME            = 'COM9'    
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        # Open port
        print(self.portHandler.openPort())
        # Set port baudrate
        self.portHandler.setBaudRate(BAUDRATE)
    
    def WriteVP(self,Write_inds,Goal_vels,Goal_poss, address_w,byte_w):
        
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, address_w, byte_w)
        if byte_w == 2:
            for i in range(len(Write_inds)):
                vp = data2byte(PWM)
                groupSyncWrite.addParam(Write_inds[i],vp)
        elif byte_w == 6:
            for i in range(len(Write_inds)):
                vp = data2byte(D)+data2byte(I)+data2byte(P)
                groupSyncWrite.addParam(Write_inds[i],vp)   
        elif byte_w == 8:
            for i in range(len(Write_inds)):
                vp = data8byte(int(Goal_vels[Write_inds[i]]), int(Goal_poss[Write_inds[i]]))
                groupSyncWrite.addParam(Write_inds[i],vp)
        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()
        print('Write')
        
    def ReadVP(self, Read_inds, address_r, byte_r):
        groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, address_r, byte_r)
        for i in range(len(Read_inds)):
            groupSyncRead.addParam(Read_inds[i])
        if byte_r == 2:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                pwm_read= groupSyncRead.getData(Read_inds[i], 124, 2)
                self.Data_PWM_value.append(pwm_read)
                if len(self.Data_PWM_value) >= 6:
                    Data_PWM_value.pop(0)
                self.Data_PWM['ID'+str(Read_inds[i])+' : ']=Data_PWM_value[-1]
        elif byte_r == 6:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                D_read = groupSyncRead.getData(Read_inds[i], 80, 2)
                I_read = groupSyncRead.getData(Read_inds[i], 82, 2)
                p_read = groupSyncRead.getData(Read_inds[i], 84, 2)
                self.Data_P_value.append(p_read)
                self.Data_I_value.append(I_read) 
                self.Data_D_value.append(D_read) 
                if len(self.Data_P_value) >= 6 and len(self.Data_I_value) >= 6 and len(self.Data_D_value) >= 6:
                    self.Data_P_value.pop(0)
                    self.Data_I_value.pop(0)
                    self.Data_D_value.pop(0)
                self.Data_P['ID'+str(Read_inds[i])+' : ']=Data_P_value[-1]
                self.Data_v['ID'+str(Read_inds[i])+' : ']=Data_I_value[-1]
                self.Data_p['ID'+str(Read_inds[i])+' : ']=Data_D_value[-1]
        elif byte_r == 10:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                present_cur = groupSyncRead.getData(Read_inds[i], 126, 2)
                present_vel = groupSyncRead.getData(Read_inds[i], 128, 4)
                present_pos = groupSyncRead.getData(Read_inds[i], 132, 4)
                #value
                self.Data_cur_value.append(present_cur)
                self.Data_vel_value.append(present_vel)
                self.Data_pos_value.append(present_pos)
                if len(self.Data_cur_value) >= 6 and len(self.Data_vel_value) >= 6 and len(self.Data_pos_value) >= 6:
                    self.Data_cur_value.pop(0)
                    self.Data_vel_value.pop(0)
                    self.Data_pos_value.pop(0)
                #key
                self.Data_cur['ID'+str(Read_inds[i])+' : ']=self.Data_cur_value[-1]
                self.Data_vel['ID'+str(Read_inds[i])+' : ']=self.Data_vel_value[-1]
                self.Data_pos['ID'+str(Read_inds[i])+' : ']=self.Data_pos_value[-1]  
            self.Data_Unit_cur = self.Unit_cur(Read_inds, self.Data_cur_value)
            self.Data_Unit_vel = self.Unit_vel(Read_inds, self.Data_vel_value)
            
    def Unit_cur(self, Read_inds, Data_cur_value):
        Data_ch_cur =[]
        Data_Unit_cur = {}
        for i in range(len(Data_cur_value)):
            if (Data_cur_value[i]&(1<<15))>>15 == 1 :
                unit_cur=(65535-(Data_cur_value[i]-1))*0.00336
            elif (Data_cur_value[i]&(1<<15))>>15 == 0 :        
                unit_cur=Data_cur_value[i]*0.00336  
            Data_ch_cur.append(np.round(unit_cur,2))
            if len(Data_ch_cur) >= 6 :
                Data_ch_cur.pop(0)

        for i in range(len(Read_inds)):    
            Data_Unit_cur['ID'+str(Read_inds[i])+' : ']=Data_ch_cur
        return Data_Unit_cur
        
        
    def Unit_vel(self, Read_inds, Data_vel_value):
        Data_ch_vel =[]
        Data_Unit_vel = {}
        for i in range(len(Data_vel_value)):
            if (Data_vel_value[i]&(1<<31))>>31 == 1 :
                Unit_vel=(4294967296-(Data_vel_value[i]-1))                   
            elif (Data_vel_value[i]&(1<<31))>>31 == 0 :
                Unit_vel=Data_vel_value[i]               
            Data_ch_vel.append(np.round(Unit_vel,2))
            if len(Data_ch_vel) >= 6 :
                Data_ch_vel.pop(0)
        for i in range(len(Read_inds)):   
            Data_Unit_vel['ID'+str(Read_inds[i])+' : ']=Data_ch_vel           
        return Data_Unit_vel
    
    def TORQUE_ON(self, Write_inds):
        for i in range(len(Write_inds)):
            self.packetHandler.write1ByteTxRx(self.portHandler, Write_inds[i], 64, 1)
            time.sleep(0.5)
            self.packetHandler.read1ByteRx(self.portHandler, Write_inds[i])
            time.sleep(0.5)
    def TORQUE_OFF(self, Write_inds):    
        for i in range(len(Write_inds)):
            self.packetHandler.write1ByteTxRx(self.portHandler, Write_inds[i], 64, 0)
    def run(self):
        
        Read_inds = np.arange(2, 4)
        Write_inds = np.arange(2, 4)
        Goal_vels = np.ones(22)*20
        Goal_poss = np.ones(22)*2048
        address_r = 126
        byte_r = 10
        address_w = 112
        byte_w = 8
        a = 2048 
        self.TORQUE_ON(Write_inds)
        while(True):
            # read motor data
            self.ReadVP( Read_inds, address_r, byte_r)
            print(self.Data_pos)
            # send command to motor
            self.WriteVP(Write_inds,Goal_vels,Goal_poss, address_w,byte_w)
            if a >= 3000:
                a = 0
            else:
                a+=100
                
            
            Goal_poss = np.ones(22)*a
            # schedule

            time.sleep(0.1)





if __name__=="__main__":
    Read_test1 =All_Motor_body(0) 
    
    app = QtWidgets.QApplication(sys.argv)
    Read_test1.start()
    
    while True:
        
        time.sleep(0.5)



        
