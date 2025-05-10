//===-- WDC65816MachineFunctionInfo.cpp - WDC65816 private data ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "WDC65816MachineFunction.h"

#include "WDC65816InstrInfo.h"
#include "WDC65816Subtarget.h"

#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"

using namespace llvm;

void WDC65816MachineFunctionInfo::anchor() {}

MachineFunctionInfo *WDC65816MachineFunctionInfo::clone(
    BumpPtrAllocator &Allocator, MachineFunction &DestMF,
    const DenseMap<MachineBasicBlock *, MachineBasicBlock *> &Src2DstMBB)
    const {
  return DestMF.cloneInfo<WDC65816MachineFunctionInfo>(*this);
}
