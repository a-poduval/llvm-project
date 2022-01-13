//===-- xray_trampoline_riscv32.s ----------------------------------*- ASM -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a part of XRay, a dynamic runtime instrumentation system.
//
// This implements the riscv32-specific assembler for the trampolines.
//
//===----------------------------------------------------------------------===//

	.text
	.file "xray_trampoline_riscv32.S"
	.globl __xray_FunctionEntry
	.p2align 2
	.type __xray_FunctionEntry,@function
__xray_FunctionEntry:
	.cfi_startproc
	// Push argument registers to stack
	addi	sp, sp, -68
	.cfi_def_cfa_offset 68
	sd	ra, 64(sp)
	.cfi_offset ra, -8
	sd	a7, 60(sp)
	sd	a6, 56(sp)
	sd	a5, 52(sp)
	sd	a4, 48(sp)
	sd	a3, 44(sp)
	sd	a3, 40(sp)
	sd	a2, 36(sp)
	sd	a0, 32(sp)
	// If this is RISCV32D, then these (faX) will still be 64 bit 
	// wide registers, so we should be incrementing by 8
	// Perhaps we can push a registers based on defines which
	// indicate if we are following RISCV32I, IF or IFD
	// We will need the same for rv64 as well
	sd	fa7, 28(sp)
	sd	fa6, 24(sp)
	sd	fa5, 20(sp)
	sd	fa4, 16(sp)
	sd	fa3, 12(sp)
	sd	fa2, 8(sp)
	sd	fa1, 4(sp)
	sd	fa0, 0(sp)

	// Load the handler function pointer into a2
	li	a2, _ZN6__xray19XRayPatchedFunctionE

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
	ld	fa1, 4(sp)
	ld	fa2, 8(sp)
	ld	fa3, 12(sp)
	ld	fa4, 16(sp)
	ld	fa5, 20(sp)
	ld	fa6, 24(sp)
	ld	fa7, 28(sp)
	ld	a0, 32(sp)
	ld	a1, 36(sp)
	ld	a2, 40(sp)
	ld	a3, 44(sp)
	ld	a4, 48(sp)
	ld	a5, 52(sp)
	ld	a6, 56(sp)
	ld	a7, 60(sp)
	ld	ra, 64(sp)
	addi	sp, sp, 68
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
	addi	sp, sp, -20
	.cfi_def_cfa_offset 20
	sd	ra, 16(sp)
	.cfi_offset ra, -8
	sd	a1, 12(sp)
	sd	a0, 8(sp)
	// If this is RISCV32D, then these (faX) will still be 64 bit 
	// wide registers, so we should be incrementing by 8
	// Perhaps we can push a registers based on defines which
	// indicate if we are following RISCV32I, IF or IFD
	// We will need the same for rv64 as well
	sd	fa1, 4(sp)
	sd	fa0, 0(sp)

	// Load the handler function pointer into a2 - similar thoughts as above
	li	a2, _ZN6__xray19XRayPatchedFunctionE

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
	ld	fa1, 4(sp)
	ld	a0, 8(sp)
	ld	a1, 12(sp)
	ld	ra, 16(sp)
	addi	sp, sp, 20
	jr	ra

FunctionExit_end:
	.size __xray_FunctionExit, FunctionExit_end-__xray_FunctionExit
	.cfi_endproc
