 
/*
 * Linker script meant to work with the STM32F103C8T6 and STM32F103CBT6
 */

MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 128K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

INCLUDE ./cortex-m-generic.ld
