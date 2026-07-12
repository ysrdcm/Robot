#!/bin/bash

stedgeai generate --no-inputs-allocation --no-outputs-allocation --model tiny_yolo_v2_224_int8.tflite --target stm32n6 --st-neural-art default@user_neuralart.json
cp st_ai_output/network_ecblobs.h .
cp st_ai_output/network.c .
cp st_ai_output/network_atonbuf.xSPI2.raw network_data.xSPI2.bin
arm-none-eabi-objcopy -I binary network_data.xSPI2.bin --change-addresses 0x70200000 -O ihex ../Binary/network-data.hex