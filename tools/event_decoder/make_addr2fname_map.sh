#!/bin/bash

# Tool for creating maping:
# - from address of string created by using __func__ special identifier of C language
# - to content of the string

if [ "$1" == "" ]; then
	echo "Usage:"
	echo "make_addr2fname_map.sh <input_elf_file>"
	echo ""
	echo "input_elf_file    The name to elf file to parse"
	echo
	exit 1
fi

INPUT_ELF_FILE=$1

if [ ! -f "$INPUT_ELF_FILE" ]; then
    echo "'$INPUT_ELF_FILE' does not exist"
	exit 1
fi

# Prepare command script for gdb (initial part)
gdb_script='make_addr2fname_map_script.gdb'
echo > "${gdb_script}"
echo set pagination off >> "${gdb_script}"
echo set height unlimited >> "${gdb_script}"

# Steps (according to pipe)
# 1. Using readelf read symbol table of input elf file
#
# 2. Take those symbols containing __func__ string. This relies on compiler used to produce such symbols
#    whenever special identifier __func__ of the C language was used.
#    Example lines of of readelf output
#       249: 00009f39    16 OBJECT  LOCAL  DEFAULT    1 __func__.11003
#       250: 00009f49    34 OBJECT  LOCAL  DEFAULT    1 __func__.11079
#
# 3. With awk we produce commands for gdb to read memory at addresses related with these symbols as strings
arm-none-eabi-readelf --symbols $INPUT_ELF_FILE | grep "__func__" | awk '{l="x/s 0x"$2; print l}' >> "${gdb_script}"

# Now using gdb we read the content of input elf file
# Steps (according to pipe)
# 1. Run gdb with generated script
# 2. Allow only lines with __func__ (gdb emits extra diagnostic lines we want to discard)
# 3. With awk we take the address and string that has been read at given address
# 4. With sed we discard quotes "" from the string
# 5. Leave result at stdout
arm-none-eabi-gdb -q -ex "source ${gdb_script}" -ex quit $INPUT_ELF_FILE | grep "__func__" | awk '{print $1, $3}' | sed -e s/\"//g
