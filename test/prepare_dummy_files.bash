#!/bin/bash -xve

if [[ $(whoami) = "root" ]]; then SUDO=""; else SUDO="sudo"; fi

${SUDO} touch /dev/rt{buzzer,motor,motoren,motor_raw_{l,r}}0
${SUDO} chmod 666 /dev/rt{buzzer,motor,motoren,motor_raw_{l,r}}0
echo "0 0 0 0" | ${SUDO} tee /dev/rtlightsensor0
${SUDO} chmod 666 /dev/rtlightsensor0
echo "0" | ${SUDO} tee /dev/rtswitch{0,1,2}
${SUDO} chmod 666 /dev/rtswitch{0,1,2}
