# List of the ChibiOS generic K20x startup and CMSIS files.
STARTUPSRC = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/crt1.c \
             $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/vectors.c

STARTUPASM = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S

STARTUPINC = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC \
             $(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/devices/K22x \
             $(CHIBIOS)/os/common/ext/CMSIS/include \
             $(CHIBIOS_CONTRIB)/os/common/ext/CMSIS/KINETIS

# STARTUPLD  = $(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/compilers/GCC/ld

# STARTUPSRC = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/crt1.c \
#              $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/vectors.c

# STARTUPASM = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/crt0_v7m.s

# STARTUPINC = $(CHIBIOS)/os/common/ports/ARMCMx/devices/K22x \
#              $(CHIBIOS)/os/ext/CMSIS/include \
#              $(CHIBIOS)/os/ext/CMSIS/KINETIS

# STARTUPLD  = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/ld
