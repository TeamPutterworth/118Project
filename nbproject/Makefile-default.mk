#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=Level1HSM/src/TopLevelHSM.c Level2HSM/src/AmmoSearchSubHSM.c Drivers/src/motor.c Drivers/src/sensors.c C:/CMPE118/src/ES_Timers.c C:/CMPE118/src/ES_TattleTale.c C:/CMPE118/src/pwm.c C:/CMPE118/src/ES_PostList.c C:/CMPE118/src/AD.c C:/CMPE118/src/ES_Framework.c C:/CMPE118/src/IO_Ports.c C:/CMPE118/src/ES_CheckEvents.c C:/CMPE118/src/BOARD.c C:/CMPE118/src/ES_Queue.c C:/CMPE118/src/LED.c C:/CMPE118/src/ES_KeyboardInput.c C:/CMPE118/src/serial.c EventCheckers/src/EventChecker.c Services/src/SyncSampling.c main.c Level2HSM/src/AmmoLoadSubHSM.c Level2HSM/src/FirstTargetSearchHSM.c Level2HSM/src/FirstTargetUnloadSubHSM.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o ${OBJECTDIR}/Drivers/src/motor.o ${OBJECTDIR}/Drivers/src/sensors.o ${OBJECTDIR}/_ext/331920610/ES_Timers.o ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o ${OBJECTDIR}/_ext/331920610/pwm.o ${OBJECTDIR}/_ext/331920610/ES_PostList.o ${OBJECTDIR}/_ext/331920610/AD.o ${OBJECTDIR}/_ext/331920610/ES_Framework.o ${OBJECTDIR}/_ext/331920610/IO_Ports.o ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o ${OBJECTDIR}/_ext/331920610/BOARD.o ${OBJECTDIR}/_ext/331920610/ES_Queue.o ${OBJECTDIR}/_ext/331920610/LED.o ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o ${OBJECTDIR}/_ext/331920610/serial.o ${OBJECTDIR}/EventCheckers/src/EventChecker.o ${OBJECTDIR}/Services/src/SyncSampling.o ${OBJECTDIR}/main.o ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o
POSSIBLE_DEPFILES=${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o.d ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o.d ${OBJECTDIR}/Drivers/src/motor.o.d ${OBJECTDIR}/Drivers/src/sensors.o.d ${OBJECTDIR}/_ext/331920610/ES_Timers.o.d ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o.d ${OBJECTDIR}/_ext/331920610/pwm.o.d ${OBJECTDIR}/_ext/331920610/ES_PostList.o.d ${OBJECTDIR}/_ext/331920610/AD.o.d ${OBJECTDIR}/_ext/331920610/ES_Framework.o.d ${OBJECTDIR}/_ext/331920610/IO_Ports.o.d ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o.d ${OBJECTDIR}/_ext/331920610/BOARD.o.d ${OBJECTDIR}/_ext/331920610/ES_Queue.o.d ${OBJECTDIR}/_ext/331920610/LED.o.d ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o.d ${OBJECTDIR}/_ext/331920610/serial.o.d ${OBJECTDIR}/EventCheckers/src/EventChecker.o.d ${OBJECTDIR}/Services/src/SyncSampling.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o.d ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o.d ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o ${OBJECTDIR}/Drivers/src/motor.o ${OBJECTDIR}/Drivers/src/sensors.o ${OBJECTDIR}/_ext/331920610/ES_Timers.o ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o ${OBJECTDIR}/_ext/331920610/pwm.o ${OBJECTDIR}/_ext/331920610/ES_PostList.o ${OBJECTDIR}/_ext/331920610/AD.o ${OBJECTDIR}/_ext/331920610/ES_Framework.o ${OBJECTDIR}/_ext/331920610/IO_Ports.o ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o ${OBJECTDIR}/_ext/331920610/BOARD.o ${OBJECTDIR}/_ext/331920610/ES_Queue.o ${OBJECTDIR}/_ext/331920610/LED.o ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o ${OBJECTDIR}/_ext/331920610/serial.o ${OBJECTDIR}/EventCheckers/src/EventChecker.o ${OBJECTDIR}/Services/src/SyncSampling.o ${OBJECTDIR}/main.o ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o

# Source Files
SOURCEFILES=Level1HSM/src/TopLevelHSM.c Level2HSM/src/AmmoSearchSubHSM.c Drivers/src/motor.c Drivers/src/sensors.c C:/CMPE118/src/ES_Timers.c C:/CMPE118/src/ES_TattleTale.c C:/CMPE118/src/pwm.c C:/CMPE118/src/ES_PostList.c C:/CMPE118/src/AD.c C:/CMPE118/src/ES_Framework.c C:/CMPE118/src/IO_Ports.c C:/CMPE118/src/ES_CheckEvents.c C:/CMPE118/src/BOARD.c C:/CMPE118/src/ES_Queue.c C:/CMPE118/src/LED.c C:/CMPE118/src/ES_KeyboardInput.c C:/CMPE118/src/serial.c EventCheckers/src/EventChecker.c Services/src/SyncSampling.c main.c Level2HSM/src/AmmoLoadSubHSM.c Level2HSM/src/FirstTargetSearchHSM.c Level2HSM/src/FirstTargetUnloadSubHSM.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

# The following macros may be used in the pre and post step lines
Device=PIC32MX320F128H
ProjectDir="C:\Users\jcrowley\Desktop\118Project_jdgrant_rlhill_jcrowley"
ConfName=default
ImagePath="dist\default\${IMAGE_TYPE}\118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX}"
ImageDir="dist\default\${IMAGE_TYPE}"
ImageName="118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX}"
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IsDebug="true"
else
IsDebug="false"
endif

.build-conf:  .pre ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX320F128H
MP_LINKER_FILE_OPTION=,--script="C:\CMPE118\bootloader320.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o: Level1HSM/src/TopLevelHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level1HSM/src" 
	@${RM} ${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o.d 
	@${RM} ${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o.d" -o ${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o Level1HSM/src/TopLevelHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o: Level2HSM/src/AmmoSearchSubHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o Level2HSM/src/AmmoSearchSubHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Drivers/src/motor.o: Drivers/src/motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Drivers/src" 
	@${RM} ${OBJECTDIR}/Drivers/src/motor.o.d 
	@${RM} ${OBJECTDIR}/Drivers/src/motor.o 
	@${FIXDEPS} "${OBJECTDIR}/Drivers/src/motor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Drivers/src/motor.o.d" -o ${OBJECTDIR}/Drivers/src/motor.o Drivers/src/motor.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Drivers/src/sensors.o: Drivers/src/sensors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Drivers/src" 
	@${RM} ${OBJECTDIR}/Drivers/src/sensors.o.d 
	@${RM} ${OBJECTDIR}/Drivers/src/sensors.o 
	@${FIXDEPS} "${OBJECTDIR}/Drivers/src/sensors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Drivers/src/sensors.o.d" -o ${OBJECTDIR}/Drivers/src/sensors.o Drivers/src/sensors.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_Timers.o: C:/CMPE118/src/ES_Timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_Timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_Timers.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_Timers.o C:/CMPE118/src/ES_Timers.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_TattleTale.o: C:/CMPE118/src/ES_TattleTale.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_TattleTale.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_TattleTale.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o C:/CMPE118/src/ES_TattleTale.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/pwm.o: C:/CMPE118/src/pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/pwm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/pwm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/pwm.o.d" -o ${OBJECTDIR}/_ext/331920610/pwm.o C:/CMPE118/src/pwm.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_PostList.o: C:/CMPE118/src/ES_PostList.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_PostList.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_PostList.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_PostList.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_PostList.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_PostList.o C:/CMPE118/src/ES_PostList.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/AD.o: C:/CMPE118/src/AD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/AD.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/AD.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/AD.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/AD.o.d" -o ${OBJECTDIR}/_ext/331920610/AD.o C:/CMPE118/src/AD.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_Framework.o: C:/CMPE118/src/ES_Framework.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Framework.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Framework.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_Framework.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_Framework.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_Framework.o C:/CMPE118/src/ES_Framework.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/IO_Ports.o: C:/CMPE118/src/IO_Ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/IO_Ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/IO_Ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/IO_Ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/IO_Ports.o.d" -o ${OBJECTDIR}/_ext/331920610/IO_Ports.o C:/CMPE118/src/IO_Ports.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o: C:/CMPE118/src/ES_CheckEvents.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o C:/CMPE118/src/ES_CheckEvents.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/BOARD.o: C:/CMPE118/src/BOARD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/BOARD.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/BOARD.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/BOARD.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/BOARD.o.d" -o ${OBJECTDIR}/_ext/331920610/BOARD.o C:/CMPE118/src/BOARD.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_Queue.o: C:/CMPE118/src/ES_Queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_Queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_Queue.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_Queue.o C:/CMPE118/src/ES_Queue.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/LED.o: C:/CMPE118/src/LED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/LED.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/LED.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/LED.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/LED.o.d" -o ${OBJECTDIR}/_ext/331920610/LED.o C:/CMPE118/src/LED.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o: C:/CMPE118/src/ES_KeyboardInput.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o C:/CMPE118/src/ES_KeyboardInput.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/serial.o: C:/CMPE118/src/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/serial.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/serial.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/serial.o.d" -o ${OBJECTDIR}/_ext/331920610/serial.o C:/CMPE118/src/serial.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/EventCheckers/src/EventChecker.o: EventCheckers/src/EventChecker.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/EventCheckers/src" 
	@${RM} ${OBJECTDIR}/EventCheckers/src/EventChecker.o.d 
	@${RM} ${OBJECTDIR}/EventCheckers/src/EventChecker.o 
	@${FIXDEPS} "${OBJECTDIR}/EventCheckers/src/EventChecker.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/EventCheckers/src/EventChecker.o.d" -o ${OBJECTDIR}/EventCheckers/src/EventChecker.o EventCheckers/src/EventChecker.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Services/src/SyncSampling.o: Services/src/SyncSampling.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Services/src" 
	@${RM} ${OBJECTDIR}/Services/src/SyncSampling.o.d 
	@${RM} ${OBJECTDIR}/Services/src/SyncSampling.o 
	@${FIXDEPS} "${OBJECTDIR}/Services/src/SyncSampling.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Services/src/SyncSampling.o.d" -o ${OBJECTDIR}/Services/src/SyncSampling.o Services/src/SyncSampling.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o: Level2HSM/src/AmmoLoadSubHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o Level2HSM/src/AmmoLoadSubHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o: Level2HSM/src/FirstTargetSearchHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o Level2HSM/src/FirstTargetSearchHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o: Level2HSM/src/FirstTargetUnloadSubHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o Level2HSM/src/FirstTargetUnloadSubHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o: Level1HSM/src/TopLevelHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level1HSM/src" 
	@${RM} ${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o.d 
	@${RM} ${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o.d" -o ${OBJECTDIR}/Level1HSM/src/TopLevelHSM.o Level1HSM/src/TopLevelHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o: Level2HSM/src/AmmoSearchSubHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/AmmoSearchSubHSM.o Level2HSM/src/AmmoSearchSubHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Drivers/src/motor.o: Drivers/src/motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Drivers/src" 
	@${RM} ${OBJECTDIR}/Drivers/src/motor.o.d 
	@${RM} ${OBJECTDIR}/Drivers/src/motor.o 
	@${FIXDEPS} "${OBJECTDIR}/Drivers/src/motor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Drivers/src/motor.o.d" -o ${OBJECTDIR}/Drivers/src/motor.o Drivers/src/motor.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Drivers/src/sensors.o: Drivers/src/sensors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Drivers/src" 
	@${RM} ${OBJECTDIR}/Drivers/src/sensors.o.d 
	@${RM} ${OBJECTDIR}/Drivers/src/sensors.o 
	@${FIXDEPS} "${OBJECTDIR}/Drivers/src/sensors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Drivers/src/sensors.o.d" -o ${OBJECTDIR}/Drivers/src/sensors.o Drivers/src/sensors.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_Timers.o: C:/CMPE118/src/ES_Timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_Timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_Timers.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_Timers.o C:/CMPE118/src/ES_Timers.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_TattleTale.o: C:/CMPE118/src/ES_TattleTale.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_TattleTale.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_TattleTale.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_TattleTale.o C:/CMPE118/src/ES_TattleTale.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/pwm.o: C:/CMPE118/src/pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/pwm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/pwm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/pwm.o.d" -o ${OBJECTDIR}/_ext/331920610/pwm.o C:/CMPE118/src/pwm.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_PostList.o: C:/CMPE118/src/ES_PostList.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_PostList.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_PostList.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_PostList.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_PostList.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_PostList.o C:/CMPE118/src/ES_PostList.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/AD.o: C:/CMPE118/src/AD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/AD.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/AD.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/AD.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/AD.o.d" -o ${OBJECTDIR}/_ext/331920610/AD.o C:/CMPE118/src/AD.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_Framework.o: C:/CMPE118/src/ES_Framework.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Framework.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Framework.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_Framework.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_Framework.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_Framework.o C:/CMPE118/src/ES_Framework.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/IO_Ports.o: C:/CMPE118/src/IO_Ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/IO_Ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/IO_Ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/IO_Ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/IO_Ports.o.d" -o ${OBJECTDIR}/_ext/331920610/IO_Ports.o C:/CMPE118/src/IO_Ports.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o: C:/CMPE118/src/ES_CheckEvents.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_CheckEvents.o C:/CMPE118/src/ES_CheckEvents.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/BOARD.o: C:/CMPE118/src/BOARD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/BOARD.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/BOARD.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/BOARD.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/BOARD.o.d" -o ${OBJECTDIR}/_ext/331920610/BOARD.o C:/CMPE118/src/BOARD.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_Queue.o: C:/CMPE118/src/ES_Queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_Queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_Queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_Queue.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_Queue.o C:/CMPE118/src/ES_Queue.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/LED.o: C:/CMPE118/src/LED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/LED.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/LED.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/LED.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/LED.o.d" -o ${OBJECTDIR}/_ext/331920610/LED.o C:/CMPE118/src/LED.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o: C:/CMPE118/src/ES_KeyboardInput.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o.d" -o ${OBJECTDIR}/_ext/331920610/ES_KeyboardInput.o C:/CMPE118/src/ES_KeyboardInput.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/331920610/serial.o: C:/CMPE118/src/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/331920610" 
	@${RM} ${OBJECTDIR}/_ext/331920610/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/331920610/serial.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/331920610/serial.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/_ext/331920610/serial.o.d" -o ${OBJECTDIR}/_ext/331920610/serial.o C:/CMPE118/src/serial.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/EventCheckers/src/EventChecker.o: EventCheckers/src/EventChecker.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/EventCheckers/src" 
	@${RM} ${OBJECTDIR}/EventCheckers/src/EventChecker.o.d 
	@${RM} ${OBJECTDIR}/EventCheckers/src/EventChecker.o 
	@${FIXDEPS} "${OBJECTDIR}/EventCheckers/src/EventChecker.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/EventCheckers/src/EventChecker.o.d" -o ${OBJECTDIR}/EventCheckers/src/EventChecker.o EventCheckers/src/EventChecker.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Services/src/SyncSampling.o: Services/src/SyncSampling.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Services/src" 
	@${RM} ${OBJECTDIR}/Services/src/SyncSampling.o.d 
	@${RM} ${OBJECTDIR}/Services/src/SyncSampling.o 
	@${FIXDEPS} "${OBJECTDIR}/Services/src/SyncSampling.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Services/src/SyncSampling.o.d" -o ${OBJECTDIR}/Services/src/SyncSampling.o Services/src/SyncSampling.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o: Level2HSM/src/AmmoLoadSubHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/AmmoLoadSubHSM.o Level2HSM/src/AmmoLoadSubHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o: Level2HSM/src/FirstTargetSearchHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/FirstTargetSearchHSM.o Level2HSM/src/FirstTargetSearchHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o: Level2HSM/src/FirstTargetUnloadSubHSM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Level2HSM/src" 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o.d 
	@${RM} ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o 
	@${FIXDEPS} "${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"Drivers/inc" -I"C:/CMPE118/include" -I"Services/inc" -I"EventCheckers/inc" -I"." -I"Level1HSM/inc" -I"Level2HSM/inc" -MMD -MF "${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o.d" -o ${OBJECTDIR}/Level2HSM/src/FirstTargetUnloadSubHSM.o Level2HSM/src/FirstTargetUnloadSubHSM.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    C:/CMPE118/bootloader320.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)    -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=_min_heap_size=10,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   C:/CMPE118/bootloader320.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=10,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/118Project_jdgrant_rlhill_jcrowley.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif

.pre:
	@echo "--------------------------------------"
	@echo "User defined pre-build step: [python C:\CMPE118\scripts\Enum_To_String.py]"
	@python C:\CMPE118\scripts\Enum_To_String.py
	@echo "--------------------------------------"

# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
