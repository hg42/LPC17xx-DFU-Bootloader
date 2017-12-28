#
#
#

APPBAUD  = 115200
#APPBAUD  = 1000000

PROJECT  = DFU-Bootloader

CONSOLE  = /dev/arduino

CSRC     = $(wildcard *.c)
CXXSRC   = $(wildcard *.cpp)
ASRC     = $(wildcard *.S)

SUBDIRS  = Drivers Core

INC      = . $(shell find */ -type d)

LIBRARIES =

OUTDIR   = build

OSRC     =

NXPSRC   = $(shell find CMSISv2p00_LPC17xx/ LPC17xxLib/ -name '*.c')
NXPO     = $(patsubst %.c,$(OUTDIR)/%.o,$(notdir $(NXPSRC))) $(OUTDIR)/system_LPC17xx.o

FATFSSRC = $(shell find fatfs/ -name '*.c')
FATFSO   = $(patsubst %.c,$(OUTDIR)/%.o,$(notdir $(FATFSSRC)))

CHIP     = lpc1769
MCU      = cortex-m3

ARCH     = arm-none-eabi
PREFIX   = $(ARCH)-

CC       = $(PREFIX)gcc
# CXX      = $(PREFIX)g++
OBJCOPY  = $(PREFIX)objcopy
OBJDUMP  = $(PREFIX)objdump
AR       = $(PREFIX)ar
SIZE     = $(PREFIX)size
READELF  = $(PREFIX)readelf

# You MUST link with G++ if you have even one C++ source file in the project
# If you have no C++, then feel free to link with gcc which gives a significant reduction in included library code
LINK     = $(PREFIX)gcc

MKDIR    = mkdir
RMDIR    = rmdir
RM       = rm -f

OPTIMIZE = s

#DEBUG_MESSAGES
CDEFS    = MAX_URI_LENGTH=512 __LPC17XX__ USB_DEVICE_ONLY APPBAUD=$(APPBAUD) DEBUG_MESSAGES

FLAGS    = -mcpu=$(MCU)
FLAGS   += -O$(OPTIMIZE)
FLAGS   += -mthumb -mthumb-interwork
FLAGS   += -mlong-calls
FLAGS   += -ffunction-sections -fdata-sections
FLAGS   += -Wall
FLAGS   += -g
FLAGS   += -funsigned-char -funsigned-bitfields -fshort-enums
FLAGS   += -fpack-struct
FLAGS   += $(patsubst %,-I%,$(INC))
FLAGS   += $(patsubst %,-D%,$(CDEFS))
#FLAGS   += -fno-aggressive-loop-optimizations
#FLAGS   += -fno-dce                       # Use the RTL dead code elimination pass.
#FLAGS   += -fno-gcse                      # Perform global common subexpression elimination.
#FLAGS   += -fno-gcse-after-reload         # Perform global common subexpression elimination after register allocation has finished.
#FLAGS   += -fno-gcse-las                  # Perform redundant load after store elimination in global common subexpression elimination.
#FLAGS   += -fno-gcse-lm                   # Perform enhanced load motion during global common subexpression elimination.
#FLAGS   += -fno-gcse-sm                   # Perform store motion after global common subexpression elimination.
#FLAGS   += -fno-ipa-cp                    # Perform interprocedural constant propagation.
#FLAGS   += -fno-ipa-cp-alignment          # Perform alignment discovery and propagation to make Interprocedural constant propagation stronger.
#FLAGS   += -fno-ipa-cp-clone              # Perform cloning to make Interprocedural constant propagation stronger.
#FLAGS   += -fno-ipa-icf                   # Perform Identical Code Folding for functions and read-only variables.
#FLAGS   += -fno-ipa-icf-functions         # Perform Identical Code Folding for functions.
#FLAGS   += -fno-ipa-profile               # Perform interprocedural profile propagation.
#FLAGS   += -fno-ipa-pta                   # Perform interprocedural points-to analysis.
#FLAGS   += -fno-ipa-pure-const            # Discover pure and const functions.
#FLAGS   += -fno-ipa-ra                    # Use caller save register across calls if possible.
#FLAGS   += -fno-ipa-reference             # Discover readonly and non addressable static variables.
#FLAGS   += -fno-ipa-sra                   # Perform interprocedural reduction of aggregates.
#FLAGS   += -fno-ira-hoist-pressure        # Use IRA based register pressure calculation in RTL hoist optimizations.
#FLAGS   += -fno-ira-loop-pressure         # Use IRA based register pressure calculation in RTL loop optimizations.
#FLAGS   += -fno-ira-share-save-slots      # Share slots for saving different hard registers.
#FLAGS   += -fno-ira-share-spill-slots     # Share stack slots for spilled pseudo-registers.
#FLAGS   += -fno-loop-nest-optimize
#FLAGS   += -fno-loop-parallelize-all
#FLAGS   += -fno-move-loop-invariants
#FLAGS   += -fno-peel-loops
#FLAGS   += -fno-prefetch-loop-arrays
#FLAGS   += -fno-reorder-blocks
#FLAGS   += -fno-tree-dse
#FLAGS   += -fno-tree-loop-distribute-patterns # Enable loop distribution for patterns transformed into a library call.
#FLAGS   += -fno-tree-loop-distribution    # Enable loop distribution on trees.
#FLAGS   += -fno-tree-loop-if-convert      # Convert conditional jumps in innermost loops to branchless equivalents.
#FLAGS   += -fno-tree-loop-if-convert-stores # Also if-convert conditional jumps containing memory writes.
#FLAGS   += -fno-tree-loop-im              # Enable loop invariant motion on trees.
#FLAGS   += -fno-tree-loop-ivcanon         # Create canonical induction variables in loops.
#FLAGS   += -fno-tree-loop-optimize
#FLAGS   += -fno-tree-loop-vectorize
#FLAGS   += -fno-unsafe-loop-optimizations
#FLAGS   += -fno-variable-expansion-in-unroller
#FLAGS   += -fstrict-volatile-bitfields

CFLAGS   = $(FLAGS) -std=gnu99 -pipe -fno-builtin-printf -fno-builtin-fprintf -fno-builtin-vfprintf -fno-builtin-puts
ASFLAGS  = $(FLAGS)
CXXFLAGS = $(FLAGS) -fno-rtti -fno-exceptions -std=gnu++0x

LDFLAGS  = $(FLAGS) -Wl,--as-needed,--gc-sections,-e,__cs3_reset_cortex_m,-T,$(CHIP).ld
LDFLAGS += $(patsubst %,-L%,$(LIBRARIES)) -lc
LDFLAGS += -Wl,-Map=$(OUTDIR)/$(PROJECT).map

OBJ      = $(patsubst %,$(OUTDIR)/%,$(notdir $(CSRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o)))

VPATH    = . $(patsubst %/inc,%/src,$(INC)) $(dir $(NXPSRC)) $(dir $(USBSRC)) $(dir $(UIPSRC)) $(dir $(LWIPSRC))

.PHONY: all clean program upload size functions functionsizes

.PRECIOUS: $(OBJ)

all: $(OUTDIR) $(OUTDIR)/nxp.ar $(OUTDIR)/fatfs.ar $(OUTDIR)/$(PROJECT).elf $(OUTDIR)/$(PROJECT).bin $(OUTDIR)/$(PROJECT).hex size

clean:
	@echo "  RM    " ".o"
	@$(RM) $(OBJ) $(OBJ:%.o=%.lst)

	@echo "  RM    " "nxp"
	@$(RM) $(NXPO) $(NXPO:%.o=%.lst) $(OUTDIR)/nxp.ar

	@echo "  RM    " "fatfs"
	@$(RM) $(FATFSO) $(FATFSO:%.o=%.lst) $(OUTDIR)/fatfs.ar

	@echo "  RM    " "build/"$(PROJECT)".*"
	@$(RM) $(OUTDIR)/$(PROJECT).bin $(OUTDIR)/$(PROJECT).hex $(OUTDIR)/$(PROJECT).elf $(OUTDIR)/$(PROJECT).map

	@echo "  RM    " "build/"
	@$(RMDIR) $(OUTDIR); true

program: $(OUTDIR)/$(PROJECT).hex
	lpc21isp $^ $(CONSOLE) 115200 12000

upload: program

console:
	@stty raw ignbrk -echo $(APPBAUD) < $(CONSOLE)
	@echo "Press ctrl+D to exit"
	@( cat <&3 & cat >&3 ; kill %% ) 3<>$(CONSOLE)


# size: $(OUTDIR)/$(PROJECT).elf
# 	@$(SIZE) $<
size: $(OUTDIR)/$(PROJECT).elf
	@echo
	@echo $$'           \033[1;4m  SIZE        LPC1769         (bootloader)\033[0m'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(text|rodata)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "  FLASH    %6d bytes  %2d%% of %3dkb    %2d%% of %3dkb\n", $$a, ceil($$a * 100 / (512 * 1024)), 512, ceil($$a * 100 / (16 * 1024)), 16 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(data|bss)\s+([0-9a-f]+)/    && do { $$a += eval "0x$$2" }; END { printf "  RAM      %6d bytes  %2d%% of %3dkb\n", $$a, ceil($$a * 100 / ( 16 * 1024)),  16 }'

functions: $(OUTDIR)/$(PROJECT).elf
	@$(READELF) -s $^ | perl -e 'for (<>) { /^\s+(\d+):\s*([0-9A-F]+)\s+(\d+)/i && do { s/^\s+//; push @symbols, [ split /\s+/, $$_ ]; }; }; for (sort { hex($$a->[1]) <=> hex($$b->[1]); } @symbols) { printf "0x%08s: [%4d] %7s %s\n", $$_->[1], $$_->[2], $$_->[3], $$_->[7] if ($$_->[2]) && (hex($$_->[1]) < 0x10000000); }'

functionsizes: $(OUTDIR)/$(PROJECT).elf
	@$(READELF) -s $^ | perl -e 'for (<>) { /^\s+(\d+):\s*([0-9A-F]+)\s+(\d+)/i && do { s/^\s+//; push @symbols, [ split /\s+/, $$_ ]; }; }; for (sort { $$a->[2] <=> $$b->[2]; } @symbols) { printf "0x%08s: [%4d] %7s %s\n", $$_->[1], $$_->[2], $$_->[3], $$_->[7] if ($$_->[2]) && (hex($$_->[1]) < 0x10000000); }'

$(OUTDIR):
	@$(MKDIR) $(OUTDIR)

$(OUTDIR)/%.bin: $(OUTDIR)/%.elf
	@echo "  COPY  " $@
	@$(OBJCOPY) -O binary $< $@

$(OUTDIR)/%.hex: $(OUTDIR)/%.elf
	@echo "  COPY  " $@
	@$(OBJCOPY) -O ihex $< $@

$(OUTDIR)/%.sym: $(OUTDIR)/%.elf
	@echo "  SYM   " $@
	@$(OBJDUMP) -t $< | perl -ne 'BEGIN { printf "%6s  %-40s %s\n", "ADDR","NAME","SIZE"; } /([0-9a-f]+)\s+(\w+)\s+O\s+\.(bss|data)\s+([0-9a-f]+)\s+(\w+)/ && printf "0x%04x  %-40s +%d\n", eval("0x$$1") & 0xFFFF, $$5, eval("0x$$4")' | sort -k1 > $@

$(OUTDIR)/%.elf: $(OBJ) $(OUTDIR)/nxp.ar $(OUTDIR)/fatfs.ar
	@echo "  LINK  " $@
	@$(LINK) $(OSRC) -Wl,-Map=$(@:.elf=.map) -o $@ $^ $(LDFLAGS)

$(OUTDIR)/%.o: %.c Makefile
	@echo "  CC    " $@
	@$(CC) $(CFLAGS) -Wa,-adhlns=$(@:.o=.lst) -c -o $@ $<

# $(OUTDIR)/%.o: %.cpp
# 	@echo "  CXX   " $@
# 	@$(CXX) $(CXXFLAGS) -Wa,-adhlns=$(@:.o=.lst) -c -o $@ $<

$(OUTDIR)/%.o: %.S Makefile
	@echo "  AS    " $@
	@$(CC) $(ASFLAGS) -Wa,-adhlns=$(@:.o=.lst) -c -o $@ $<

$(OUTDIR)/nxp.ar: $(NXPO)
	@echo "  AR    " "  nxp/"$@
	@$(AR) cru $@ $^

$(OUTDIR)/fatfs.ar: $(FATFSO)
	@echo "  AR    " "fatfs/"$@
	@$(AR) cru $@ $^
