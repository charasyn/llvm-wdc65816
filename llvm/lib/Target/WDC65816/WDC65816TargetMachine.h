//===-- WDC65816TargetMachine.h - Define TargetMachine for WDC65816 -----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file declares the WDC65816 specific subclass of TargetMachine.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_WDC65816TARGETMACHINE_H
#define LLVM_LIB_TARGET_WDC65816_WDC65816TARGETMACHINE_H

#include "WDC65816Subtarget.h"
#include "MCTargetDesc/WDC65816MCTargetDesc.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"

#include <optional>

namespace llvm {
class formatted_raw_ostream;
class WDC65816RegisterInfo;

class WDC65816TargetMachine : public LLVMTargetMachine {
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  WDC65816Subtarget Subtarget;

  mutable StringMap<std::unique_ptr<WDC65816Subtarget>> SubtargetMap;

public:
  WDC65816TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                    StringRef FS, const TargetOptions &Options,
                    std::optional<Reloc::Model> RM,
                    std::optional<CodeModel::Model> CM, CodeGenOptLevel OL,
                    bool JIT);

  ~WDC65816TargetMachine() override;

  const WDC65816Subtarget *getSubtargetImpl() const { return &Subtarget; }

  const WDC65816Subtarget *getSubtargetImpl(const Function &F) const override;

  MachineFunctionInfo *
  createMachineFunctionInfo(BumpPtrAllocator &Allocator, const Function &F,
                            const TargetSubtargetInfo *STI) const override;

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }
};
} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_WDC65816TARGETMACHINE_H
