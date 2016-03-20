#*複数のcソースがある場合、本来Makefileにソース名を記入しますが
#　これは記入せずとも同じ階層のソースファイル(**.c)は全てビルドするようにしています。
#　ただ、ソースを削除する場合、「一緒にゴミ箱にいれる」にしないとソース自体は残ったままになり
#　整合が取れないので注意。


#================= 必須設定 ============================

MCU = atmega88p
#MCU = atmega328p
F_CPU = 16000000

#================= 適宜変更箇所 =========================

OPT = s
#CINCS =
MATH_LIB = -lm 

#================= avrdude ライター関係 =================

AVRDUDE_PROGRAMMER	= usbasp
AVRDUDE_PORT		= /debv/cu.usbserial

# AVR_FUSE_L	= 0xE2	# 8 MHz
# AVR_FUSE_L	= 0x62	# 1 MHz
# AVR_FUSE_L	= 0x64	# old 1 MHz setting
# AVR_FUSE_H	= 0xdf
# AVR_FUSE_E	= 0xff


AVRDUDE_BITCLOCK_DELAY	= 1

AVRDUDE_BITBANG_DELAY	= 1

#AVRDUDE_VERBOSE = -v -v

#========================================================

#	説明

#================= 必須設定 ===========================

#	MCU:	ターゲットAVRの種類
#			atmega128 , attiny48等
#------------------------------------------------------
#	F_CPU ターゲットAVRのクロック
#	      1000000 , 800000 等


#================= 適宜変更箇所 ========================

#　OPT:		コンパイラの最適化設定
#　			0:最適化無し　1,2,3 s:コードサイズ優先 
#------------------------------------------------------
#　MATH_LIB:	float型やmath.hなどにある関数を使用する場合
#　			このライブラリをリンクさせる
#　			コメントを外すとリンクする
#------------------------------------------------------
# CINCS:	インクルードファイルが別フォルダにある場合
#			そのパスを記入

#================= avrdude ライター関係 =================
#
#AVRDUDE_PROGRAMMER:	使用するライターを記入
#						ライター名は
#						http://www.nongnu.org/avrdude/user-manual/avrdude_4.html#SEC4
#						の -cオプション欄で確認のこと
#						(usbasp,dragon_isp,stk500v2　・・・)
#------------------------------------------------------
#AVRDUDE_PORT:			ライターが接続されているポート
#						USBシリアル変換であれば /dev/tty.~~~~
#							ターミナルで　"ls /dev"で確認のこと
#						USBaspは　/debv/cu.usbserialでOKだった。
#						AVRDragonは　usb でOK。
#						その他は未確認
#						(/debv/cu.usbserial , usb , /dev/tty.~~~~ ・・)
#------------------------------------------------------
#AVR_FUSE_L,H,E:		AVRのヒューズ設定。16進数で記入
#						H,Eは無い場合もあります。その場合は未記入で。
#						こちらで計算できるので便利です。
#						http://palmavr.sourceforge.net/cgi-bin/fc.cgi
#						( 0xFF, 0x65 , 0xA5 ・・・)
#------------------------------------------------------
#AVRDUDE_BITCLOCK_DELAY:どちらもAVRが低速設定時(1MHzなど）に書き込めるように
#AVRDUDE_BITBANG_DELAY	ISPクロックを遅くする設定です。
#						JTAG ICE,AVR Dragonでは BITCLOCK_DELAYを
#						bit-bangタイプのプログラマーではBITBANG_DELAYを記入
#						設定値は AVR Clockが1MHzのとき　3 (us) 以上で焼くことができました
#						およそCPUクロックの1/4以下(1MHz: 250kHz）のISPクロックが推奨されてるようです。
#						3us=333kHzで範囲外ですので4以上が無難？
#						いまのところAVR Dragonでしか試していません。
#						( 1,2,3,4,5・・・)
#
#						ただ、一度書き込めるようになると設定せずとも書けたり・・
#						ちょっとよくわかりません。。。
#------------------------------------------------------
#AVRDUDE_VERBOSE		動作状況を詳細に表示するモードです。
#						ただエラーがおきたときにしかXCodeは表示してくれません。
#						コメントを外すと有効
#------------------------------------------------------
#
#	以上


#
# modified recotana 2008/6/9
#
# thanks OSX-AVR http://www.digithink.com/OSX-AVR/Introduction/Main
# thanks AVR MacPack http://www.obdev.at/products/avrmacpack/index.htm

# Hey Emacs, this is a -*- makefile -*-
#----------------------------------------------------------------------------
# WinAVR Makefile Template written by Eric B. Weddington, Jˆrg Wunsch, et al.
#
# Released to the Public Domain
#
# Additional material for this makefile was written by:
# Peter Fleury
# Tim Henigan
# Colin O'Flynn
# Reiner Patommel
# Markus Pfaff
# Sander Pool
# Frederik Rouleau 
#
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF.
#
# make extcoff = Convert ELF to AVR Extended COFF.
#
# make program = Download the hex file to the device, using avrdude.
#                Please customize the avrdude settings below first!
#
# make debug = Start either simulavr or avarice as specified for debugging, 
#              with avr-gdb or avr-insight as the front end for debugging.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------


# Output format. (can be srec, ihex, binary)
FORMAT = ihex


# Target file name (without extension).
TARGET = main

# AVRlib LIBSRC = $(AVRLIB)**1.c $(AVRLIB)**2.c
AVRLIB_SRC = 

# List C source files here. (C dependencies are automatically generated.)
#SRC += ADC.c 

SRC = $(wildcard *.c)



# List Assembler source files here.
#     Make them always end in a capital .S.  Files ending in a lowercase .s
#     will not be considered source files but generated files (assembler
#     output from the compiler), and will be deleted upon "make clean"!
#     Even though the DOS/Win* filesystem matches both .s and .S the same,
#     it will preserve the spelling of the filenames, and gcc itself does
#     care about how the name is spelled on its command-line.
ASRC = 


# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
#OPT = s


# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = stabs


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRAINCDIRS = $(AVRLIB)


# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99


# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)UL 


# Place -I options here
#CINCS =



#---------------- Compiler Options ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -Wall -Wstrict-prototypes
CFLAGS += -Wa,-adhlns=$(<:.c=.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)


#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 


#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
PRINTF_LIB = 
#PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_FLOAT)


# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB = 
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)

#MATH_LIB +=




#---------------- External Memory Options ----------------

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS = 



#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)



#---------------- Programming Options (avrdude) ----------------

# Programming hardware: alf avr910 avrisp bascom bsd 
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
#AVRDUDE_PROGRAMMER = avrdude

# com1 = serial port. Use lpt1 to connect to parallel port.
#AVRDUDE_PORT = /dev/cu.usbserial    # programmer connected to serial device

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep


# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v





AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
AVRDUDE_FLAGS += $(AVRDUDE_BITCLOCK)
AVRDUDE_FLAGS += $(AVRDUDE_BITBANG)

ifneq ($(AVRDUDE_BITCLOCK_DELAY),)
	AVRDUDE_FLAGS +=-B $(AVRDUDE_BITCLOCK_DELAY)
endif

ifneq ($(AVRDUDE_BITBANG_DELAY),)
	AVRDUDE_FLAGS += -i $(AVRDUDE_BITBANG_DELAY)
endif



ifneq ($(AVR_FUSE_E),)
	AVRDUDE_FUSES = -U efuse:w:$(AVR_FUSE_E):m
endif

ifneq ($(AVR_FUSE_H),)
	AVRDUDE_FUSES += -U hfuse:w:$(AVR_FUSE_H):m
endif

ifneq ($(AVR_FUSE_L),)
	AVRDUDE_FUSES += -U lfuse:w:$(AVR_FUSE_L):m
else
	AVRDUDE_FUSES =
endif






#---------------- Debugging Options ----------------

# For simulavr only - target MCU frequency.
DEBUG_MFREQ = $(F_CPU)

# Set the DEBUG_UI to either gdb or insight.
 DEBUG_UI = gdb
#DEBUG_UI = insight

# Set the debugging back-end to either avarice, simulavr.
DEBUG_BACKEND = avarice
#DEBUG_BACKEND = simulavr

# GDB Init Filename.
GDBINIT_FILE = __avr_gdbinit

# When using avarice settings for the JTAG
#JTAG_DEV = /dev/cu.usbserial
JTAG_DEV = usb
#JTAG_OPTS = --erase

# Debugging port used to communicate between GDB / avarice / simulavr.
DEBUG_PORT = 4242

# Debugging host used to communicate between GDB / avarice / simulavr, normally
#     just set to localhost unless doing some sort of crazy debugging when 
#     avarice is running on a different computer.
DEBUG_HOST = localhost



#============================================================================


# Define programs and commands.
SHELL = sh
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
COPY = cp
WINSHELL = cmd


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:




# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o) 

# Define all listing files.
LST = $(SRC:.c=.lst) $(ASRC:.S=.lst) 


# Compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)



 

# Default target.
all: begin clean gccversion sizebefore build sizeafter end

all-prog: begin all program end

build: elf hex eep lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym



# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	echo
	echo $(MSG_BEGIN)

#
# a couple of hacks were added here by NAL to help with Xcode debugging
# ie give some kind of valid executable file.
end:
#	chmod 755 $(TARGET).elf
	echo $(MSG_END)
	echo
	

# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -A $(TARGET).elf
AVRMEM = avr-mem.sh $(TARGET).elf $(MCU)

sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	$(AVRMEM) 2>/dev/null; echo; fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	$(AVRMEM) 2>/dev/null; echo; fi



# Display compiler version information.
gccversion : 
	@$(CC) --version

# Program the device.  
program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)
	
# write fuse setting
flash-fuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_FUSES)


# Generate avr-gdb config/init file which does the following:
#     define the reset signal, load the target file, connect to target, and set 
#     a breakpoint at main().
gdb-config: 
	$(REMOVE) $(GDBINIT_FILE)
	echo define reset >> $(GDBINIT_FILE)
	echo SIGNAL SIGHUP >> $(GDBINIT_FILE)
	echo end >> $(GDBINIT_FILE)
	echo file $(TARGET).elf >> $(GDBINIT_FILE)
	echo target remote $(DEBUG_HOST):$(DEBUG_PORT)  >> $(GDBINIT_FILE)
ifeq ($(DEBUG_BACKEND),simulavr)
	echo load  >> $(GDBINIT_FILE)
endif	
	echo break main >> $(GDBINIT_FILE)
	
debug: gdb-config $(TARGET).elf
ifeq ($(DEBUG_BACKEND), avarice)
	echo Starting AVaRICE - Press enter when "waiting to connect" message displays.
#	@$(WINSHELL) /c start avarice --jtag $(JTAG_DEV) --erase --program --file \
	$(TARGET).elf $(DEBUG_HOST):$(DEBUG_PORT)
#	@$(WINSHELL) /c pause
	avarice --jtag $(JTAG_DEV) $(JTAG_OPTS) --program --file \
	$(TARGET).elf $(DEBUG_HOST):$(DEBUG_PORT) &
	
else
#	@$(WINSHELL) /c start simulavr --gdbserver --device $(MCU) --clock-freq \

	simulavr --gdbserver --device $(MCU) --clock-freq \
	$(DEBUG_MFREQ) --port $(DEBUG_PORT)
endif
#	@$(WINSHELL) /c start 
	sleep 10
	avr-$(DEBUG_UI) --command=$(GDBINIT_FILE)
	



# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 


coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof


extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof



# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $^ --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@ 


# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).eep
	$(REMOVE) $(TARGET).cof
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) .dep/*



# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program debug gdb-config



