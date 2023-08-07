#####################################################################
##     Processor:           MC56F83783VLH
##     Compiler:            CodeWarrior C/C++ for DSP M56800E
##     Reference manual:    Manual version TBD
##     Version:             rev. 0.1, 2019-09-23
##     Build:               b200929
##
##     Abstract:
##         Linker file for the CodeWarrior C/C++ for DSP M56800E
##
##     Copyright 2016 Freescale Semiconductor, Inc.
##     Copyright 2016-2020 NXP
##     All rights reserved.
##
##     SPDX-License-Identifier: BSD-3-Clause
##
##     http:                 www.nxp.com
##     mail:                 support@nxp.com
##
#####################################################################

#####################################################################
##
##  MC56F83783VLH linker file - Flash mapped into data space range 0x8000~0xbfff is not used
##
##  Notes:
##  - Source codes(reside in Flash) that need to run in RAM can be put into "codesInRam.text" section
##  - Default ramCodeSize = 3KWords (0xC00) of Flash and RAM
##    are assigned to store and run codes which are meant to run in RAM by default.
##  - To change the ramCodeSize, below memory segment should be changed manually:
##      1. p_Code
##      2. p_Storage
##      3. x_Data
##      4. p_ramSpace
##      5. p_flash_ROM_data
##  - See the comments in MEMORY{} block which contains ramCodeSize, change them with the desired size,
##    and assign the calculated results to the corresponding ORIGIN and LENGTH.
##
##    ***************************************************************
##              Program_space
##              -------------
##    0x0:
##              vector table
##    0xDF:
##              -------------
##              -------------
##    0x1E0:
##              bootloader config area
##    0x1FF:
##              -------------
##              -------------
##    0x200:
##              flash config field
##    0x207:
##              -------------
##              -------------
##    0x208:
##            > Codes running in Flash
##            > Global variables' initial values, namely, "*.data" sections
##    0x1F3FF:
##              -------------
##              -------------
##    0x1F400:
##            > Codes stored in Flash, running in RAM
##    0x1FFFF:
##              -------------
##
##    ***************************************************************
##
##              Data_space
##              -------------
##    0x0:
##              > Codes running in RAM
##    0xBFF:
##              -------------
##              -------------
##    0xC00:
##              > .data sections
##              > .bss sections
##              > Heap
##              > Stack
##    0x7FFF:
##              -------------
##
#####################################################################

FLASH_PARTITION {
  # name : ORIGIN_P = addr, ORIGIN_X = addr, LENGTH = size [> filename]
  FlashImage   : ORIGIN_P = 0x0, ORIGIN_X = 0x20000, LENGTH = 0x20000 > FlashImage.bin
  # FlashPart1 : ORIGIN_P = 0x0, ORIGIN_X = 0x20000, LENGTH = 0x10000
  # FlashPart2 : ORIGIN_P = 0x10000, ORIGIN_X = 0x30000, LENGTH = 0x10000
}

MEMORY {
  # I/O registers area for on-chip peripherals
  .x_Peripherals   (RW): ORIGIN = 0xC000, LENGTH = 0x0

  .p_Interrupts   (RWX): ORIGIN = 0x0, LENGTH = 0xE0
  .p_reserved_BCA (RWX): ORIGIN = 0x1E0, LENGTH = 0x20 # Bootloader Configuration Area
  .p_reserved_FCF (RWX): ORIGIN = 0x200, LENGTH = 0x8  # Flash Configuration Area

  .p_Code         (RWX): ORIGIN = 0x208, LENGTH = 0x1BDF8 # 111KWords of Flash for codes running in Flash.
                                                          # ORIGIN fixed to 0x208, LENGTH = 0x1FDF8 - ramCodeSize(0x4000)

  .p_Storage      (RWX): ORIGIN = 0x1C000, LENGTH = 0x4000 # 16KWords of Flash for ram-code storage.
                                                          # ORIGIN = 0x20000 - ramCodeSize(0x4000), LENGTH = ramCodeSize( 0x4000)

  .x_Data         (RW) : ORIGIN = 0x4000, LENGTH = 0x4000  # 16KWords of RAM mapped in data space.
                                                          # ORIGIN = ramCodeSize( 0x4000), LENGTH = 0x8000 - ramCodeSize(0x4000)

  .p_ramSpace     (RX) : ORIGIN = 0x60000, LENGTH = 0x4000 # 16KWords of RAM mapped in program space.
                                                          # ORIGIN fixed to 0x60000, LENGTH = ramCodeSize(default 0x4000)

  # p_flash_ROM_data mirrors x_Data, mapping to origin and length
  # the "X" flag in "RX" tells the debugger flash p-memory.
  # the p-memory flash is directed to the address determined by AT
  # in the data_in_p_flash_ROM section definition
  .p_flash_ROM_data  (RX) : ORIGIN = 0x4000, LENGTH = 0x4000  # 16KWords for data with their initial value stored in Flash,
                                                             # but actual addresses are in RAM of data space.
                                                             # ORIGIN = ramCodeSize(0x4000), LENGTH = 0x8000 - ramCodeSize( 0x4000)
}

KEEP_SECTION { interrupt_vectors.text }
KEEP_SECTION { reserved_FCF.text }
KEEP_SECTION { codesInRam.text }
KEEP_SECTION { reserved_BCA.text}

SECTIONS {

  .interrupt_vectors :
  {
    F_vector_addr = .;
    # interrupt vectors
    * (interrupt_vectors.text)
    Fvba_vector_addr = (F_vector_addr >> 4);
  } > .p_Interrupts

  .reserved_FCF :
  {
    F_FCF_addr = .;
    # reserved FCF - Flash Configuration Field
    * (reserved_FCF.text)
  } > .p_reserved_FCF

  .reserved_BCA :
  {
    F_BCA_addr = .;
    # reserved BCA - Bootloader Configuration Area
    * (reserved_BCA.text)
  } > .p_reserved_BCA

  .ramFunctions : AT(ADDR(.p_Storage))
  {
    F__pRAM_code_start = .;

    OBJECT(FGFLIB_Sin_F16_FAsm,GFLIB_LDM.lib) # this is an example of how to allocate functions in library to RAM
    OBJECT(FGFLIB_AtanYX_F16_FAsm,GFLIB_LDM.lib)
    OBJECT(FGFLIB_CtrlPIpAW_F16_FAsm,GFLIB_LDM.lib)
    OBJECT(FGFLIB_Ramp_F32_FAsm,GFLIB_LDM.lib)
    OBJECT(FGFLIB_Ramp_F16_FAsm,GFLIB_LDM.lib)
    OBJECT(FGFLIB_Sqrt_F16l_FAsm,GFLIB_LDM.lib)
    OBJECT(FGFLIB_Sqrt_F16_FAsm,GFLIB_LDM.lib)
    OBJECT(FAMCLIB_PMSMBemfObsrvDQ_F16_FAsm,AMCLIB_LDM.lib)
    OBJECT(FAMCLIB_TrackObsrv_F16_FAsm,AMCLIB_LDM.lib)
    OBJECT(FGMCLIB_SvmStd_F16_FAsm,GMCLIB_LDM.lib)
    OBJECT(FGMCLIB_ElimDcBusRipFOC_F16_FAsm,GMCLIB_LDM.lib)
    * (rtlib.text) # suggest to put rtlib function into RAM if ISR performance needs to be improved
    * (codesInRam.text)
    # save address where for the data start in pROM
    . = ALIGN(2);
    F__pRAM_code_end = .;
    __ramfunctions_size = F__pRAM_code_end - F__pRAM_code_start;
  } > .p_ramSpace
    # AT sets the download address

  .ApplicationCode :
  {
    F_Pcode_start_addr =.;

    # .text sections
    * (.text)

    * (startup.text)
    * (fp_engine.text)
    * (ll_engine.text)
    * (user.text)
    * (rtlib.text)
    * (.data.pmem)
    * (.const.data.pmem)

    F_Pcode_end_addr = .;

    # save address where for the data start in pROM
    . = ALIGN(2);
    F__pROM_data_start = .;
  } > .p_Code

  .data_in_p_flash_ROM : AT(F__pROM_data_start)
  {
    # the data sections flashed to pROM
    # save data start so we can copy data later to xRAM

    __xRAM_data_start = .;

    # .data sections
    * (.const.data.char)     # used if "Emit Separate Char Data Section" enabled
    * (.const.data)

    * (fp_state.data)
    * (rtlib.data)
    * (.data.char)        # used if "Emit Separate Char Data Section" enabled
    * (.data)



    # save data end and calculate data block size
    . = ALIGN(2);
    __xRAM_data_end = .;
    __data_size = __xRAM_data_end - __xRAM_data_start;

  } > .p_flash_ROM_data          # this section is designated as p-memory
                                 # with X flag in the memory map
                                 # the start address and length map to
                                 # actual internal xRAM

  .ApplicationData :
  {
    # save space for the pROM data copy
    . = __xRAM_data_start + __data_size;

    # .bss sections
    * (rtlib.bss.lo)
    * (rtlib.bss)
    . = ALIGN(4);
    F_Xbss_start_addr = .;
    _START_BSS = .;
    * (.bss.char)         # used if "Emit Separate Char Data Section" enabled
    * (.bss)
    . = ALIGN(2);         # used to ensure proper functionality of the zeroBSS hardware loop utility
    _END_BSS   = .;
    F_Xbss_length = _END_BSS - _START_BSS;

    /* Setup the HEAP address */
    . = ALIGN(4);
    _HEAP_ADDR = .;
    _HEAP_SIZE = 0;
    _HEAP_END = _HEAP_ADDR + _HEAP_SIZE;
    . = _HEAP_END;

    /* SETUP the STACK address */
    _min_stack_size = 0x500;
    _stack_addr = _HEAP_END;
    _stack_end  = _stack_addr + _min_stack_size;
    . = _stack_end;

    /* EXPORT HEAP and STACK runtime to libraries */
    F_heap_addr   = _HEAP_ADDR;
    F_heap_end    = _HEAP_END;
    F_Lstack_addr = _HEAP_END;
    F_StackAddr = _HEAP_END;
    F_StackEndAddr = _stack_end - 1;

    # runtime code __init_sections uses these globals:

    F_Ldata_size     = __data_size;
    F_Ldata_RAM_addr = __xRAM_data_start;
    F_Ldata_ROM_addr = F__pROM_data_start;

    F_Livt_size     = __ramfunctions_size;
    F_Livt_RAM_addr = F__pRAM_code_start;
    F_Livt_ROM_addr = ADDR(.p_Storage);


    F_xROM_to_xRAM   = 0x0000;
    F_pROM_to_xRAM   = 0x0001;  # Enable loading data initial value to corresponding variables before entering main function
    F_pROM_to_pRAM   = 0x0001;  # Enable loading codes that residing in Flash but running in RAM to RAM before entering main function

    F_start_bss   = _START_BSS;
    F_end_bss     = _END_BSS;

    __DATA_END=.;
  } > .x_Data
}
