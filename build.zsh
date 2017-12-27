
source ./config.zsh

killn -f miniterm

arm-none-eabi-gcc  --version

make APPBAUD=$BAUD &&
  . ./upload.zsh   &&
  . ./console.zsh
