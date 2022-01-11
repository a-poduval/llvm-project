//===-- xray_trampoline_riscv64.s ----------------------------------*- ASM -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a part of XRay, a dynamic runtime instrumentation system.
//
// This implements the riscv64-specific assembler for the trampolines.
//
//===----------------------------------------------------------------------===//

	.text
	.file "xray_trampoline_riscv64.S"
	.globl __xray_FunctionEntry
	.p2align 2
	.type __xray_FunctionEntry,@function
__xray_FunctionEntry:
	.cfi_startproc
	// Push argument registers to stack
	addi	sp, sp, -136
	.cfi_def_cfa_offset 136
	sd	ra, 128(sp)
	.cfi_offset ra, -8
	sd	a7, 120(sp)
	sd	a6, 112(sp)
	sd	a5, 104(sp)
	sd	a4, 96(sp)
	sd	a3, 88(sp)
	sd	a3, 80(sp)
	sd	a2, 72(sp)
	sd	a0, 64(sp)
	sd	fa7, 56(sp)
	sd	fa6, 48(sp)
	sd	fa5, 40(sp)
	sd	fa4, 32(sp)
	sd	fa3, 24(sp)
	sd	fa2, 16(sp)
	sd	fa1, 8(sp)
	sd	fa0, 0(sp)

	// Load the handler function pointer into a2
	li	a2, _ZN6__xray19XRayPatchedFunctionE

	/* Should it be the following? I felt li would be a safer bet because
	   it would result in the 32 bit value directly being loaded in a2
	   Also, does RISCV have any standard conventions with the global pointer, like
	   MIPS does, as seen in the MIPS XRay trampoline?

	lui	a2, %hi(_ZN6__xray19XRayPatchedFunctionE)
	addi	a2, %lo(_ZN6__xray19XRayPatchedFunctionE)

	*/

	// Handler address will be null if it is not set
	beq	a2, x0, FunctionEntry_restore

	// If we reach here, we are tracing an event
	// a0 already contains function id
	// a1 = 0 means we are tracing an entry event
	mv	a1, x0
	jalr	a2

FunctionEntry_restore:
	// Restore argument registers
	ld	fa0, 0(sp)
	ld	fa1, 8(sp)
	ld	fa2, 16(sp)
	ld	fa3, 24(sp)
	ld	fa4, 32(sp)
	ld	fa5, 40(sp)
	ld	fa6, 48(sp)
	ld	fa7, 56(sp)
	ld	a0, 64(sp)
	ld	a1, 72(sp)
	ld	a2, 80(sp)
	ld	a3, 88(sp)
	ld	a4, 96(sp)
	ld	a5, 104(sp)
	ld	a6, 112(sp)
	ld	a7, 120(sp)
	ld	ra, 128(sp)
	addi	sp, sp, 136
	jr	ra

FunctionEntry_end:
	.size __xray_FunctionEntry, FunctionEntry_end-__xray_FunctionEntry
	.cfi_endproc

	.text
	.globl __xray_FunctionExit
	.p2align 2
	.type __xray_FunctionExit,@function
__xray_FunctionExit:
	.cfi_startproc
	// Push return registers to stack
	addi	sp, sp, -40
	.cfi_def_cfa_offset 40
	sd	ra, 32(sp)
	.cfi_offset ra, -8
	sd	a1, 24(sp)
	sd	a0, 16(sp)
	sd	fa1, 8(sp)
	sd	fa0, 0(sp)

	// Load the handler function pointer into a2
	li	a2, _ZN6__xray19XRayPatchedFunctionE

	/* Should it be the following? I felt li would be a safer bet because
	   it would result in the 32 bit value directly being loaded in a2
	   Also, does RISCV have any standard conventions with the global pointer, like
	   MIPS does, as seen in the MIPS XRay trampoline?

	lui	a2, %hi(_ZN6__xray19XRayPatchedFunctionE)
	addi	a2, %lo(_ZN6__xray19XRayPatchedFunctionE)

	*/

	// Handler address will be null if it is not set
	beq	a2, x0, FunctionExit_restore

	// If we reach here, we are tracing an event
	// a0 already contains function id
	// a1 = 1 means we are tracing an exit event
	addi	a1, x0, 1
	jalr	a2

FunctionExit_restore:
	// Restore return registers
	ld	fa0, 0(sp)
	ld	fa1, 8(sp)
	ld	a0, 16(sp)
	ld	a1, 24(sp)
	ld	ra, 32(sp)
	addi	sp, sp, 40
	jr	ra

FunctionExit_end:
	.size __xray_FunctionExit, FunctionExit_end-__xray_FunctionExit
	.cfi_endproc
