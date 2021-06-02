BLACKMAGICSRC =	blackmagic/src/platforms/common/swdptap.c \
				blackmagic/src/timing.c \
				blackmagic/src/target/target.c \
				blackmagic/src/target/adiv5_swdp.c \
				blackmagic/src/target/adiv5.c \
				blackmagic/src/target/cortexm.c \
				blackmagic/src/target/nrf51.c \
				blackmagic/src/target/stm32f1.c \
				blackmagic/src/target/stm32f4.c \
				blackmagic/src/target/stm32h7.c \
				blackmagic/src/target/stm32l0.c \
				blackmagic/src/target/stm32l4.c \
				blackmagic-integration/bm_if.c \
				blackmagic-integration/glue.c \
				blackmagic-integration/exception.c \
				blackmagic-integration/platform.c \

BLACKMAGICINC =	blackmagic/src/include \
				blackmagic/src/target \
				blackmagic-integration \

BLACKMAGICDEF =	-DPC_HOSTED=0 \
				-DNO_LIBOPENCM3=1 \
