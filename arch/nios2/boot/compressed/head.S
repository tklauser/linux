/*
 *  linux/arch/nios2/boot/compressed/head.S
 *
 */

	.text
	.set noat
#include <asm/asm-macros.h>
#include <asm/nios.h>

	/*
	 * This code can be loaded anywhere, as long as output will not
	 * overlap it.
	 *
	 */

	.global	_start
_start:
	// disable interrupt
	wrctl   status, r0
	//------------------------------------------------------
	// Zero out the .bss segment (uninitialized common data)
	//
	movia	r2,__bss_start		// presume nothing is between
	movia	r1,_end			// the .bss and _end.
1:
	stb	r0,0(r2)
	addi	r2,r2,1
	bne	r1,r2,1b
	// set up the stack pointer, some where higher than _end. The stack space must be greater than 32K for decompress.
	movia	sp, 0x10000
	add	sp,sp,r1
	// save args passed from u-boot
	addi	sp,sp,-16
	stw	r4,0(sp)
	stw	r5,4(sp)
	stw	r6,8(sp)
	stw	r7,12(sp)
/*
 * decompress the kernel
 */
	call	decompress_kernel
	
flush_cache:
	// pass saved args to kernel
	ldw	r4,0(sp)
	ldw	r5,4(sp)
	ldw	r6,8(sp)
	ldw	r7,12(sp)
	movia r1, (DDR2_TOP_BASE | KERNEL_REGION_BASE)
	jmp	r1

	.balign 512
fake_headers_as_bzImage:
	.short	0
	.ascii	"HdrS"
	.short	0x0202
	.short	0
	.short	0
	.byte	0x00, 0x10
	.short	0
	.byte	0
	.byte	1
	.byte	0x00, 0x80
	.long	0
	.long	0

