################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../Headers


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../adc_driver.c \
../hardware.c \
../interrupt.c \
../mechanism.c \
../port_driver.c \
../TC_driver.c \
../funkcije.c \
../usart_driver.c \
../main.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
adc_driver.o \
hardware.o \
interrupt.o \
mechanism.o \
port_driver.o \
TC_driver.o \
funkcije.o \
usart_driver.o \
main.o

OBJS_AS_ARGS +=  \
adc_driver.o \
hardware.o \
interrupt.o \
mechanism.o \
port_driver.o \
TC_driver.o \
funkcije.o \
usart_driver.o \
main.o

C_DEPS +=  \
adc_driver.d \
hardware.d \
interrupt.d \
mechanism.d \
port_driver.d \
TC_driver.d \
funkcije.d \
usart_driver.d \
main.d

C_DEPS_AS_ARGS +=  \
adc_driver.d \
hardware.d \
interrupt.d \
mechanism.d \
port_driver.d \
TC_driver.d \
funkcije.d \
usart_driver.d \
main.d

OUTPUT_FILE_PATH +=AXIS_XDRIVE_MALIROB.elf

OUTPUT_FILE_PATH_AS_ARGS +=AXIS_XDRIVE_MALIROB.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler



















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 3.4.2
	$(QUOTE)D:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -funsigned-char -funsigned-bitfields -DDEBUG  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atxmega128a1 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<"
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 3.4.2
	$(QUOTE)D:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="AXIS_XDRIVE_MALIROB.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mrelax -mmcu=atxmega128a1 
	@echo Finished building target: $@
	"D:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "AXIS_XDRIVE_MALIROB.elf" "AXIS_XDRIVE_MALIROB.hex"
	"D:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "AXIS_XDRIVE_MALIROB.elf" "AXIS_XDRIVE_MALIROB.eep" || exit 0
	"D:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "AXIS_XDRIVE_MALIROB.elf" > "AXIS_XDRIVE_MALIROB.lss"
	"D:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "AXIS_XDRIVE_MALIROB.elf" "AXIS_XDRIVE_MALIROB.srec"
	"D:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-size.exe" "AXIS_XDRIVE_MALIROB.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "AXIS_XDRIVE_MALIROB.elf" "AXIS_XDRIVE_MALIROB.a" "AXIS_XDRIVE_MALIROB.hex" "AXIS_XDRIVE_MALIROB.lss" "AXIS_XDRIVE_MALIROB.eep" "AXIS_XDRIVE_MALIROB.map" "AXIS_XDRIVE_MALIROB.srec"
	