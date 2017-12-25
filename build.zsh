
source ./config.zsh

make APPBAUD=$BAUD &&
  . ./upload.zsh   &&
  . ./console.zsh
