//===-- WDC65816TargetMachine.cpp - WDC65816 Target Machine -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains implementation for WDC65816 target machine.
///
//===----------------------------------------------------------------------===//

#include "WDC65816TargetMachine.h"
#include "WDC65816.h"
#include "WDC65816MachineFunction.h"
#include "WDC65816Subtarget.h"
#include "WDC65816TargetObjectFile.h"
#include "TargetInfo/WDC65816TargetInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/PassRegistry.h"
#include <memory>
#include <optional>

using namespace llvm;

#define DEBUG_TYPE "wdc65816"

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeWDC65816Target() {
  RegisterTargetMachine<WDC65816TargetMachine> X(getTheWDC65816Target());
  auto *PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializeWDC65816DAGToDAGISelLegacyPass(*PR);
  initializeWDC65816ExpandPseudoPass(*PR);
  initializeWDC65816GlobalBaseRegPass(*PR);
  initializeWDC65816CollapseMOVEMPass(*PR);
}

namespace {

std::string computeDataLayout(const Triple &TT, StringRef CPU,
                              const TargetOptions &Options) {
  std::string Ret = "";
  // WDC65816 is Little Endian
  Ret += "e";

  // Use ELF symbol mangling
  Ret += "-m:e";

  // Address space 0: $7E0000-$7EFFFF
  // We will use 16-bit pointers for the default address space 0
  // (used for global variables).
  Ret += "-p0:16:8";

  // Address space 1: $000000-$FFFFFF
  // We will use 32-bit pointers for address space 1, which covers the
  // entire address space.
  Ret += "-p1:32:8";

  // Integers don't require any alignment.
  Ret += "-i8:8-i16:8-i32:8";

  // FIXME no floats at the moment

  // Assume registers always hold 16 bits
  Ret += "-n16";

  // Aggregate types can have any alignment
  Ret += "-a:0:16";

  // Let's keep the stack 16-bit aligned
  Ret += "-S16";

  // Code goes into address space 1
  Ret += "-P1";

  return Ret;
}

Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                    std::optional<Reloc::Model> RM) {
  // If not defined we default to static
  if (!RM.has_value())
    return Reloc::Static;

  return *RM;
}

CodeModel::Model getEffectiveCodeModel(std::optional<CodeModel::Model> CM,
                                       bool JIT) {
  if (!CM) {
    return CodeModel::Small;
  } else if (CM == CodeModel::Large) {
    llvm_unreachable("Large code model is not supported");
  } else if (CM == CodeModel::Kernel) {
    llvm_unreachable("Kernel code model is not implemented yet");
  }
  return CM.value();
}
} // end anonymous namespace

WDC65816TargetMachine::WDC65816TargetMachine(const Target &T, const Triple &TT,
                                     StringRef CPU, StringRef FS,
                                     const TargetOptions &Options,
                                     std::optional<Reloc::Model> RM,
                                     std::optional<CodeModel::Model> CM,
                                     CodeGenOptLevel OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options), TT, CPU, FS,
                        Options, getEffectiveRelocModel(TT, RM),
                        ::getEffectiveCodeModel(CM, JIT), OL),
      TLOF(std::make_unique<WDC65816ELFTargetObjectFile>()),
      Subtarget(TT, CPU, FS, *this) {
  initAsmInfo();
}

WDC65816TargetMachine::~WDC65816TargetMachine() {}

const WDC65816Subtarget *
WDC65816TargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  auto CPU = CPUAttr.isValid() ? CPUAttr.getValueAsString().str() : TargetCPU;
  auto FS = FSAttr.isValid() ? FSAttr.getValueAsString().str() : TargetFS;

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = std::make_unique<WDC65816Subtarget>(TargetTriple, CPU, FS, *this);
  }
  return I.get();
}

MachineFunctionInfo *WDC65816TargetMachine::createMachineFunctionInfo(
    BumpPtrAllocator &Allocator, const Function &F,
    const TargetSubtargetInfo *STI) const {
  return WDC65816MachineFunctionInfo::create<WDC65816MachineFunctionInfo>(Allocator, F,
                                                                  STI);
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

namespace {
class WDC65816PassConfig : public TargetPassConfig {
public:
  WDC65816PassConfig(WDC65816TargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  WDC65816TargetMachine &getWDC65816TargetMachine() const {
    return getTM<WDC65816TargetMachine>();
  }

  const WDC65816Subtarget &getWDC65816Subtarget() const {
    return *getWDC65816TargetMachine().getSubtargetImpl();
  }
  void addIRPasses() override;
  bool addIRTranslator() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
  bool addInstSelector() override;
  void addPreSched2() override;
  void addPreEmitPass() override;
};
} // namespace

TargetPassConfig *WDC65816TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new WDC65816PassConfig(*this, PM);
}

void WDC65816PassConfig::addIRPasses() {
  addPass(createAtomicExpandLegacyPass());
  TargetPassConfig::addIRPasses();
}

bool WDC65816PassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createWDC65816ISelDag(getWDC65816TargetMachine()));
  addPass(createWDC65816GlobalBaseRegPass());
  return false;
}

bool WDC65816PassConfig::addIRTranslator() {
  addPass(new IRTranslator());
  return false;
}

bool WDC65816PassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool WDC65816PassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool WDC65816PassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  return false;
}

void WDC65816PassConfig::addPreSched2() { addPass(createWDC65816ExpandPseudoPass()); }

void WDC65816PassConfig::addPreEmitPass() {
  addPass(createWDC65816CollapseMOVEMPass());
}
