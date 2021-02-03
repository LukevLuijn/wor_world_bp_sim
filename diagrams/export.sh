#!/bin/sh

# converts all .puml files to svg images.

# text colors
OK_COLOR='\033[0;32m'
BAD_COLOR='\033[0;33m'
NC='\033[0m'

SET_BOLD='\e[1m'
SET_NO_BOLD='\e[21m'

#printf "89%%"; sleep 1; printf "\r90%%\n"
#

printf "${NC}[${BAD_COLOR}not found${NC}] searching for $2 directory.."

if [ -d "${2}" ]
then
    printf "\r"
    printf "${NC}[${OK_COLOR}found${NC}] searching for $2 directory..    \n"
else
    printf "\r"
    printf "${NC}[${BAD_COLOR}not found${NC}] created new directory: $2    \n"
    mkdir $2
    chmod 777 $2
fi


for FILE in $1/*.puml; do

  NAME=$(basename -- "$FILE")
  EXT="${NAME##*.}"
  printf "${NC}[${BAD_COLOR}converting${NC}] ${NAME}.. "
  NAME="${NAME%.*}"

#  printf "${NC}[${BAD_COLOR}converting${NC}] ${NAME}.. "

  NAME="${NAME}.svg"
  cat $FILE | sudo docker run --rm -i think/plantuml > $NAME

  SPACES='     '

  printf "\r"
  printf "${NC}[${OK_COLOR}complete${NC}] ${NAME}${SPACES}\n"
#  printf "${OK_COLOR}${SET_BOLD}complete\n${SET_NO_BOLD}${NC}"

  chmod 777 $NAME
  mv $NAME $2/$NAME
done



