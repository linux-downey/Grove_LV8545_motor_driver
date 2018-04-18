LV8548MC_Grove motor driver
=================================================

A arduino example for LV8548_Grove motor driver control.  
------------------------------------------------------------------  

Introduction of motor driver  
----------------------  
The LV8548MC is a 2-channellow saturation voltage forward/reverse motor driver IC.It is optimal for motor druve in 12V system products and can drive either two DC motors,one DC motor using parallel connection,or it can drive a stepper motor in Full-step and Half-step.  

***
Introduction of library
-----------------------
This library contains two firewarm.
**serial_control_example** is just a independent skecth.
**module_fireware** is a firewarm source code for MEGA328U4 MUC on module.
***

usage:
===========  
**1、At first,What you need to know is the LV8548MC_Grove motor driver sensor module contains a LV8548 IC and a atmel-MEGA-32U4 MCU.  In fact, Arduino sends the protocol data to the arduino MCU on the module and controls the motor drive through the MCU on the module.The process is arduino-baseboard->arduino-MCU on module->LV8548.**
2、Download all the source files and open serial_control_example/serial_control_example.ino in arduino IDE,This is a sketch.
3、Then compile and download.
4、Connect the corresponding motor to the module, and then connect the module to the arduino-baseboard with the Grove connection cable to control the module.The routine implements some simple control instructions  

**The following two points are not necessary,Because the MCU on the module already has the default, available firmware. but I suggest you do so to achieve better motor control** 
5、If you need to understand the underlying principles of the driver chip, or to achieve more customized, more flexible control, you can Read all the code in module_fireware/,The excel file motor_protocol.xlsx is the communication protocol between two board.
6、Modify the module firmware and write it according to your own needs.This may cost you some time, but it's worth it. It is worth noting that you should select Genuino Micro board as develop board in arduino IDE.  

Notice:
--------------
**1  The defualt angle(The angle at which the motor runs) value is 7.5 at the part of STEP motor,Every motor has a fixed value.You should ensure the angle per step of the motor you use,and set it. Otherwise,controls for precise corners will be inaccurate.**  

**2  The minimum interval time between two control protocols is 80ms,Don't send data continuously,A small delay is needed.Otherwise,the module will not recognize the protocol data.If you don't like this form,you can also change it by modify the protocol between arduino-baseboard and module firewarm,Or modify the receiving method.However, it is necessary to modify the arduino-baseboard firmware and the firmware of the module at the same time**

***
LV8548 reference manual:
------------------------
[LV8548 reference manuals website](http://www.onsemi.cn/PowerSolutions/evalBoard.do?id=LV8548MCSLDGEVK)




***
This software is written by downey  for seeed studio<br>
Email:dao.huang@seeed.cc
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check License.txt for more information.<br>

Contributing to this software is warmly welcomed. You can do this basically by<br>
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above<br>
for operating guide). Adding change log and your contact into file header is encouraged.<br>
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China. <br>
Benefiting from local manufacture power and convenient global logistic system, <br>
we integrate resources to serve new era of innovation. Seeed also works with <br>
global distributors and partners to push open hardware movement.<br>
