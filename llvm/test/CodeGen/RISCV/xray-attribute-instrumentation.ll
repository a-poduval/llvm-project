; RUN: llc -filetype=asm -o - -mtriple=riscv32-unknown-elf < %s | FileCheck --check-prefix=CHECK %s
; RUN: llc -filetype=asm -o - -mtriple=riscv32-unknown-linux-gnu  < %s | FileCheck --check-prefix=CHECK %s
; RUN: llc -filetype=asm -o - -mtriple=riscv64-unknown-elf < %s | FileCheck --check-prefix=CHECK --check-prefix=CHECK-RISCV64 %s
; RUN: llc -filetype=asm -o - -mtriple=riscv64-unknown-linux-gnu  < %s | FileCheck --check-prefix=CHECK --check-prefix=CHECK-RISCV64 %s

define i32 @foo() nounwind noinline uwtable "function-instrument"="xray-always" {
; CHECK:		.p2align 2
; CHECK-LABEL:		.Lxray_sled_0:
; CHECK-NEXT:		j .Ltmp0
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-LABEL:		.Ltmp0:
  ret i32 0
; CHECK:		.p2align 2
; CHECK-LABEL:		.Lxray_sled_1:
; CHECK-NEXT:		j .Ltmp1
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-NEXT:  		nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-RISCV64:	nop
; CHECK-LABEL:		.Ltmp1:
; CHECK-NEXT:  		ret
}
; CHECK:		.section xray_instr_map,{{.*}}
; CHECK-LABEL:		.Lxray_sleds_start0:
; CHECK:		.Lxray_sled_0-.Ltmp{{.*}}
; CHECK:		.Lxray_sled_1-.Ltmp{{.*}}
; CHECK-LABEL:		.Lxray_sleds_end0:
