/*
*****************************************************************************
**

**  File        : LinkerScript.ld
**
**  Abstract    : Linker script for STM32L476RETx Device with
**                512KByte FLASH, 128KByte RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; COPYRIGHT(c) 2014 Ac6</center></h2>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**   1. Redistributions of source code must retain the above copyright notice,
**      this list of conditions and the following disclaimer.
**   2. Redistributions in binary form must reproduce the above copyright notice,
**      this list of conditions and the following disclaimer in the documentation
**      and/or other materials provided with the distribution.
**   3. Neither the name of Ac6 nor the names of its contributors
**      may be used to endorse or promote products derived from this software
**      without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = 0x20018000;    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x080;      /* required amount of heap  */
_Min_Stack_Size = 0x100; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 96K
RAM2 (xrw)      : ORIGIN = 0x10000000, LENGTH = 32K
FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 512K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(8);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(8);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(8);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(8);
    *(.user_process)   /* .user_process sections (code) */
    *(.user_process*)  /* .user_process* sections (code) */
    . = ALIGN(8);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(8);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(8);
  } >FLASH

  .ARM.extab   : 
  { 
  . = ALIGN(8);
  *(.ARM.extab* .gnu.linkonce.armextab.*)
  . = ALIGN(8);
  } >FLASH
  .ARM : {
	. = ALIGN(8);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
	. = ALIGN(8);
  } >FLASH

  .preinit_array     :
  {
	. = ALIGN(8);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
	. = ALIGN(8);
  } >FLASH
  
  .init_array :
  {
	. = ALIGN(8);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
	. = ALIGN(8);
  } >FLASH
  .fini_array :
  {
	. = ALIGN(8);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
	. = ALIGN(8);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(8);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
	. = ALIGN(8);
    _dev_initcall_start_0 = .;        /* create a global symbol at initcall start */
    KEEP(*(.dev_initcall_0))         		/* .dev_initcall sections */
    KEEP(*(.dev_initcall_0*))        		/* .dev_initcall* sections*/
    . = ALIGN(8);
    _dev_initcall_end_0 = .;        /* define a global symbol at initcall end */
	. = ALIGN(8);
    _dev_exitcall_start_0 = .;        /* create a global symbol at exitcall start */
    KEEP(*(.dev_exitcall_0))         		/* .dev_exitcall sections */
    KEEP(*(.dev_exitcall_0*))        		/* .dev_exitcall* sections*/
    . = ALIGN(8);
    _dev_exitcall_end_0 = .;        /* define a global symbol at exitcall end */
    
	. = ALIGN(8);
    _dev_initcall_start_1 = .;        /* create a global symbol at initcall start */
    KEEP(*(.dev_initcall_1))         		/* .dev_initcall sections */
    KEEP(*(.dev_initcall_1*))        		/* .dev_initcall* sections*/
    . = ALIGN(8);
    _dev_initcall_end_1 = .;        /* define a global symbol at initcall end */
	. = ALIGN(8);
    _dev_exitcall_start_1 = .;        /* create a global symbol at exitcall start */
    KEEP(*(.dev_exitcall_1))         		/* .dev_exitcall sections */
    KEEP(*(.dev_exitcall_1*))        		/* .dev_exitcall* sections*/
    . = ALIGN(8);
    _dev_exitcall_end_1 = .;        /* define a global symbol at exitcall end */
    
	. = ALIGN(8);
    _dev_initcall_start_2 = .;        /* create a global symbol at initcall start */
    KEEP(*(.dev_initcall_2))         		/* .dev_initcall sections */
    KEEP(*(.dev_initcall_2*))        		/* .dev_initcall* sections*/
    . = ALIGN(8);
    _dev_initcall_end_2 = .;        /* define a global symbol at initcall end */
	. = ALIGN(8);
    _dev_exitcall_start_2 = .;        /* create a global symbol at exitcall start */
    KEEP(*(.dev_exitcall_2))         		/* .dev_exitcall sections */
    KEEP(*(.dev_exitcall_2*))        		/* .dev_exitcall* sections*/
    . = ALIGN(8);
    _dev_exitcall_end_2 = .;        /* define a global symbol at exitcall end */
    
	. = ALIGN(8);
    _dev_initcall_start_3 = .;        /* create a global symbol at initcall start */
    KEEP(*(.dev_initcall_3))         		/* .dev_initcall sections */
    KEEP(*(.dev_initcall_3*))        		/* .dev_initcall* sections*/
    . = ALIGN(8);
    _dev_initcall_end_3 = .;        /* define a global symbol at initcall end */
	. = ALIGN(8);
    _dev_exitcall_start_3 = .;        /* create a global symbol at exitcall start */
    KEEP(*(.dev_exitcall_3))         		/* .dev_exitcall sections */
    KEEP(*(.dev_exitcall_3*))        		/* .dev_exitcall* sections*/
    . = ALIGN(8);
    _dev_exitcall_end_3 = .;        /* define a global symbol at exitcall end */


	. = ALIGN(8);
    _src_initcall_start = .;        /* create a global symbol at initcall start */
    KEEP(*(.src_initcall))         	/* .src_initcall sections */
    KEEP(*(.src_initcall*))        	/* .src_initcall* sections*/
    . = ALIGN(8);
    _src_initcall_end = .;       	 /* define a global symbol at initcall end */
	. = ALIGN(8);
    _src_deinitcall_start = .;       /* create a global symbol at exitcall start */
    KEEP(*(.src_deinitcall))         /* .src_deinitcall sections */
    KEEP(*(.src_deinitcall*))        /* .src_deinitcall* sections*/
    . = ALIGN(8);
    _src_deinitcall_end = .;        /* define a global symbol at exitcall end */
    
    	. = ALIGN(8);
    _alg_initcall_start = .;        /* create a global symbol at initcall start */
    KEEP(*(.alg_initcall))         	/* .alg_initcall sections */
    KEEP(*(.alg_initcall*))        	/* .alg_initcall* sections*/
    . = ALIGN(8);
    _alg_initcall_end = .;       	 /* define a global symbol at initcall end */
	. = ALIGN(8);
    _alg_deinitcall_start = .;       /* create a global symbol at exitcall start */
    KEEP(*(.alg_deinitcall))         /* .alg_deinitcall sections */
    KEEP(*(.alg_deinitcall*))        /* .alg_deinitcall* sections*/
    . = ALIGN(8);
    _alg_deinitcall_end = .;        /* define a global symbol at exitcall end */

    . = ALIGN(8);
    _process_init_start = .;        /* create a global symbol at process_initcall start */
    KEEP(*(.process_initcall))      /* .process_initcall sections */
    KEEP(*(.process_initcall*))     /* .dprocess_initcall* sections*/
    . = ALIGN(8);
    _process_init_end = .;        /* define a global symbol at process_initcall end */
    . = ALIGN(8);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

      /* Init code of device drivers */

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM
  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


