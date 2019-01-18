# invoke SourceDir generated makefile for main.pem4f
main.pem4f: .libraries,main.pem4f
.libraries,main.pem4f: package/cfg/main_pem4f.xdl
	$(MAKE) -f C:\Github_Sync\ARM_TI_MSP432P401R\CCS\rtos\motor_control_rtos/src/makefile.libs

clean::
	$(MAKE) -f C:\Github_Sync\ARM_TI_MSP432P401R\CCS\rtos\motor_control_rtos/src/makefile.libs clean

