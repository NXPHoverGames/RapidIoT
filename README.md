# RapidIoT
## Steps to include the MAVlink demo into the Rapid IoT “Hello World” demo:
###	Install SDK for Rapid IoT:
1.	Download SDK from here: https://www.nxp.com/webapp/Download?colCode=SLN-RPK-NODE-SDK&appType=license 
2.	Install the SDK per Drag&Drop into the “Installed SDKs” Window of MCUXpresso
###	Import the “rapid_iot_hello_world” project into your workspace
1.	Open the Workspace in which you want to work.
2.	Select “File->New->Project”
3.	A wizard for a new Project opens
4.	Select “MCUXpresso IDE -> Import SDK Examples” and press next
5.	Select the “rapid_iot_k64f” board (you can find it after selecting “K6x” in the window “SDK MCUs”) and press next
6.	Create a project name (prefix) and select the “hello world” project under the item “rapid_iot_apps”, then press next
7.	From the project Advanced Settings page, uncheck the two options Redirect SDK “PRINTF” to C library “printf” and Include semihost Hardfault handler and check the option Redlib: Use floating point version of printf, then press Finish
###	Include the Mavlink Demo into the “Hello World” example
1.	Copy the folder “mavlink” into the project (select the project header by right click and paste the folder there)
2.	Delete the “hello_world.c” in the folder “source”
3.	Copy the file “hg_mavlink_demo.c” in the folder “source”
4.	Go to the “Peripherals". To do that select Window->Perspective->Open Perspective->Peripherals
5.	Select your project and the functional group “BOARD_Init_TERMIANL_UART” and activate the checkbox of UART2 and name this Peripheral “UART_2”
6.	Now click on “Update Code”
7.	Build the project
###	Flashing the code on the Rapid IoT
1.	Select the “GUI Flash Tool” on the top and flash your code to the Rapid IoT
2.	Therefore you have to connect your device to the PC via JLink. Follow the steps in this link till connection of the adapter board with the J-Link: https://nxp.gitbook.io/hovergames/developerguide/program-software-using-debugger after that connect the JST-GH wire to the debug port of the HDIB abapter board (port at the bottom). 
