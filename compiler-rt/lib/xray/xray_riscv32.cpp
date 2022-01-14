//===-- xray_riscv32.cpp ----------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a part of XRay, a dynamic runtime instrumentation system.
//
// Implementation of riscv32-specific routines (64-bit).
//
//===----------------------------------------------------------------------===//
#include "sanitizer_common/sanitizer_common.h"
#include "xray_defs.h"
#include "xray_interface_internal.h"
#include <atomic>

namespace __xray {

// The machine codes for some instructions used in runtime patching.
enum PatchOpcodes : uint32_t {
  PO_ADDI = 0x00000013,   // addi rd, rs1, imm
  PO_ADD =   0x00000033,  // add rd, rs1, rs2
  PO_SW = 0x00002023,     // sw rt, base(offset)
  PO_LUI = 0x00000037,    // lui rd, imm
  PO_ORI = 0x00000013,    // ori rd, rs1, imm
  PO_OR =    0x00006033,  // or rd, rs1, rs2
  PO_SLLI = 0x00001013,   // slli rd, rs, shamt
  PO_JALR = 0x00000067,   // jalr rs
  PO_LW = 0x00002003,     // lw rd, base(offset)
  PO_B = 0x0000006f,      // jal #n_bytes
  PO_NOP = 0x00000013,    // nop - pseduo-instruction, same as addi r0, r0, 0
};

enum RegNum : uint32_t {
  RN_R0 = 0x0,
  RN_RA = 0x1,
  RN_SP = 0x2,
  RN_T0 = 0x5,
  RN_T1 = 0x6,
  RN_T2 = 0x7,
  RN_A0 = 0xa,
};

inline static uint32_t
encodeRTypeInstruction(uint32_t Opcode, uint32_t Rs1, uint32_t Rs2,
                         uint32_t Rd) XRAY_NEVER_INSTRUMENT {
  return (Rs2 << 20 | Rs1 << 15 | Rd << 7 | Opcode);
}

inline static uint32_t encodeITypeInstruction(uint32_t Opcode, uint32_t Rs1,
                                         uint32_t Rd,
                                         uint32_t Imm) XRAY_NEVER_INSTRUMENT {
  return (Imm << 20 | Rs1 << 15 | Rd << 7 | Opcode);
}

inline static uint32_t
encodeSTypeInstruction(uint32_t Opcode, uint32_t Rs1, uint32_t Rs2,
                         uint32_t Imm) XRAY_NEVER_INSTRUMENT {
  uint32_t imm_msbs = (Imm & 0xfe0) << 25;
  uint32_t imm_lsbs = (Imm & 0x01f) << 7;
  return (imm_msbs | Rs2 << 20 | Rs1 << 15 | imm_lsbs | Opcode);
}

inline static uint32_t encodeUTypeInstruction(uint32_t Opcode, 
                                         uint32_t Rd,
                                         uint32_t Imm) XRAY_NEVER_INSTRUMENT {
  return (Imm << 12 | Rd << 7 | Opcode);
  // Not sure if  an entire 32 bit value is taken and the lower 12 bits are zeroed out
  // That is, the immediate which is passed to U Type instructions is 32 bits, and it is
  // converted to a 20 bit value, in which case this function should return the following.
  // return ((Imm >> 12)<<12 | Rd << 7 | Opcode);
}

inline static uint32_t encodeJTypeInstruction(uint32_t Opcode, 
                                         uint32_t Rd,
                                         uint32_t Imm) XRAY_NEVER_INSTRUMENT {
  uint32_t imm_msb = (Imm & 0x80000) << 31;
  uint32_t imm_lsbs = (Imm & 0x003ff) << 21;
  uint32_t imm_11 = (Imm & 0x00400) << 20;
  uint32_t imm_1912 = (Imm & 0x7f800) << 12;
  return (imm_msb | imm_lsbs | imm_11 | imm_1912 | Rd << 7 | Opcode);
}

inline static bool patchSled(const bool Enable, const uint32_t FuncId,
                             const XRaySledEntry &Sled,
                             void (*TracingHook)()) XRAY_NEVER_INSTRUMENT {
  // When |Enable| == true,
  // We replace the following compile-time stub (sled):
  //
  // xray_sled_n:
  //	J .tmpN
  //	18 NOPs (72 bytes)
  //	.tmpN
  //
  // With the following runtime patch:
  //
  // xray_sled_n (64-bit):
  //    addi sp, sp, -16                                                        ;create stack frame
  //    sw ra, 12(sp)                                                           ;save return address
  //    sw t2, 8(sp)                                                            ;save register t2
  //    sw t1, 4(sp)                                                            ;save register t1
  //    sw t0, 0(sp)                                                            ;save register t0
  //    addi t0, r0, 1                                                          ;store 4096 in register t0 to handle 
  //    slli t0, t0, 12                                                         ;cases when the 12 bit value is negative
  //    lui t1, %hi(__xray_FunctionEntry/Exit)
  //    addi t1, t1, %lo(__xray_FunctionEntry/Exit)
  //    if higher was negative, i.e msb was 1 
  //    add t1, t1, t0, else nop
  //    lui a0, %hi(function_id)
  //    addi a0, a0, %lo(function_id)                                           ;pass function id
  //    if lower function id  was negative, i.e msb was 1 
  //    add a0, a0, t0, else nop
  //    jalr t1                                                                 ;call Tracing hook
  //    lw t0, 0(sp)                                                            ;restore register t0
  //    lw t1, 4(sp)                                                            ;restore register t1
  //    lw t2, 8(sp)                                                            ;restore register t2
  //    lw ra, 12(sp)                                                           ;restore return address
  //    addi sp, sp, 16                                                         ;delete stack frame
  //
  // Replacement of the first 4-byte instruction should be the last and atomic
  // operation, so that the user code which reaches the sled concurrently
  // either jumps over the whole sled, or executes the whole sled when the
  // latter is ready.
  //
  // When |Enable|==false, we set back the first instruction in the sled to be
  //   J #76 bytes

  uint32_t *Address = reinterpret_cast<uint32_t *>(Sled.address());
  if (Enable) {
    uint32_t LoTracingHookAddr =
        reinterpret_cast<int64_t>(TracingHook) & 0xfff;
    uint32_t HiTracingHookAddr =
        (reinterpret_cast<int64_t>(TracingHook) >> 12) & 0xfffff;
    uint32_t LoFunctionID = FuncId & 0xfff;
    uint32_t HiFunctionID = (FuncId >> 12) & 0xfffff;
    Address[1] = encodeSTypeInstruction(PatchOpcodes::PO_SW, RegNum::RN_SP,
                                   RegNum::RN_RA, 0x0c);
    Address[2] = encodeSTypeInstruction(PatchOpcodes::PO_SW, RegNum::RN_SP,
                                   RegNum::RN_T2, 0x08);
    Address[3] = encodeSTypeInstruction(PatchOpcodes::PO_SW, RegNum::RN_SP,
                                   RegNum::RN_T1, 0x4);
    Address[4] = encodeSTypeInstruction(PatchOpcodes::PO_SW, RegNum::RN_SP,
                                   RegNum::RN_T0, 0x0);
    Address[5] = encodeITypeInstruction(PatchOpcodes::PO_ADDI, RegNum::RN_R0,
                                   RegNum::RN_T0, 0x01);
    Address[6] = encodeITypeInstruction(PatchOpcodes::PO_SLLI, RegNum::RN_T0,
                                   RegNum::RN_T0, 0x0c);
    Address[7] = encodeUTypeInstruction(PatchOpcodes::PO_LUI, RegNum::RN_T1,
                                   HiTracingHookAddr);
    Address[8] = encodeITypeInstruction(PatchOpcodes::PO_ADDI, RegNum::RN_T1,
                                   RegNum::RN_T1, LoTracingHookAddr);
    if((LoTracingHookAddr & 0x0800) >> 11) {            // Add 4096
        Address[9] = encodeRTypeInstruction(PatchOpcodes::PO_ADD, RegNum::RN_T0, RegNum::RN_T1,
                                       RegNum::RN_T1);
    } else {                                            // NOP
        Address[9] = encodeITypeInstruction(PatchOpcodes::PO_ADDI, RegNum::RN_R0,
                                       RegNum::RN_R0, 0);
    }
    Address[10] = encodeUTypeInstruction(PatchOpcodes::PO_LUI, RegNum::RN_A0,
                                    HiFunctionID);
    Address[11] = encodeITypeInstruction(PatchOpcodes::PO_ADDI, RegNum::RN_A0,
                                    RegNum::RN_A0, LoFunctionID);
    if((LoFunctionID & 0x0800) >> 11) {                 // Add 4096
        Address[12] = encodeRTypeInstruction(PatchOpcodes::PO_ADD, RegNum::RN_T0, RegNum::RN_A0,
                                       RegNum::RN_A0);
    } else {                                            // NOP
        Address[12] = encodeITypeInstruction(PatchOpcodes::PO_ADDI, RegNum::RN_R0,
                                       RegNum::RN_R0, 0);
    }
    Address[13] = encodeITypeInstruction(PatchOpcodes::PO_JALR, RegNum::RN_T1,
                                           RegNum::RN_RA, 0x0);
    Address[14] = encodeITypeInstruction(PatchOpcodes::PO_LW, RegNum::RN_SP,
                                    RegNum::RN_T0, 0x0);
    Address[15] = encodeITypeInstruction(PatchOpcodes::PO_LW, RegNum::RN_SP,
                                    RegNum::RN_T1, 0x4);
    Address[16] = encodeITypeInstruction(PatchOpcodes::PO_LW, RegNum::RN_SP,
                                    RegNum::RN_T2, 0x08);
    Address[17] = encodeITypeInstruction(PatchOpcodes::PO_LW, RegNum::RN_SP,
                                    RegNum::RN_RA, 0x0c);
    Address[18] = encodeITypeInstruction(PatchOpcodes::PO_ADDI, RegNum::RN_SP,
                                    RegNum::RN_SP, 0x10);
    uint32_t CreateStackSpace = encodeITypeInstruction(
        PatchOpcodes::PO_ADDI, RegNum::RN_SP, RegNum::RN_SP, 0xfff0);
    std::atomic_store_explicit(
        reinterpret_cast<std::atomic<uint32_t> *>(Address), CreateStackSpace,
        std::memory_order_release);
  } else {
    uint32_t CreateBranch = encodeJTypeInstruction(
        PatchOpcodes::PO_B, RegNum::RN_R0, 0x026);				//jump encodes an offset in multiples of 2 bytes. 38*2 = 76
    std::atomic_store_explicit(
        reinterpret_cast<std::atomic<uint32_t> *>(Address), CreateBranch,
        std::memory_order_release);
  }
  return true;
}

bool patchFunctionEntry(const bool Enable, const uint32_t FuncId,
                        const XRaySledEntry &Sled,
                        void (*Trampoline)()) XRAY_NEVER_INSTRUMENT {
  return patchSled(Enable, FuncId, Sled, Trampoline);
}

bool patchFunctionExit(const bool Enable, const uint32_t FuncId,
                       const XRaySledEntry &Sled) XRAY_NEVER_INSTRUMENT {
  return patchSled(Enable, FuncId, Sled, __xray_FunctionExit);
}

bool patchFunctionTailExit(const bool Enable, const uint32_t FuncId,
                           const XRaySledEntry &Sled) XRAY_NEVER_INSTRUMENT {
  // FIXME: Implement tail exits in riscv32
  return patchSled(Enable, FuncId, Sled, __xray_FunctionExit);
  //return patchSled(Enable, FuncId, Sled, __xray_FunctionTailExit);
}

bool patchCustomEvent(const bool Enable, const uint32_t FuncId,
                      const XRaySledEntry &Sled) XRAY_NEVER_INSTRUMENT {
  // FIXME: Implement in riscv32?
  return false;
}

bool patchTypedEvent(const bool Enable, const uint32_t FuncId,
                     const XRaySledEntry &Sled) XRAY_NEVER_INSTRUMENT {
  // FIXME: Implement in riscv32?
  return false;
}
} // namespace __xray

extern "C" void __xray_ArgLoggerEntry() XRAY_NEVER_INSTRUMENT {
  // FIXME: this will have to be implemented in the trampoline assembly file
}
