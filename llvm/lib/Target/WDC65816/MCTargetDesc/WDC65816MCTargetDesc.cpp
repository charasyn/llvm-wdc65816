//===-- WDC65816MCTargetDesc.cpp - WDC65816 Target Descriptions ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file provides WDC65816 target specific descriptions.
///
//===----------------------------------------------------------------------===//

#include "WDC65816MCTargetDesc.h"
#include "WDC65816InstPrinter.h"
#include "WDC65816MCAsmInfo.h"
#include "TargetInfo/WDC65816TargetInfo.h"

#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstPrinter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "WDC65816GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "WDC65816GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "WDC65816GenRegisterInfo.inc"

// TODO Implement feature set parsing logics
static std::string ParseWDC65816Triple(const Triple &TT, StringRef CPU) {
  return "";
}

static MCInstrInfo *createWDC65816MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitWDC65816MCInstrInfo(X); // defined in WDC65816GenInstrInfo.inc
  return X;
}

static MCRegisterInfo *createWDC65816MCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitWDC65816MCRegisterInfo(X, llvm::WDC65816::A0, 0, 0, llvm::WDC65816::PC);
  return X;
}

static MCSubtargetInfo *createWDC65816MCSubtargetInfo(const Triple &TT,
                                                  StringRef CPU, StringRef FS) {
  std::string ArchFS = ParseWDC65816Triple(TT, CPU);
  if (!FS.empty()) {
    if (!ArchFS.empty()) {
      ArchFS = (ArchFS + "," + FS).str();
    } else {
      ArchFS = FS.str();
    }
  }
  return createWDC65816MCSubtargetInfoImpl(TT, CPU, /*TuneCPU=*/CPU, ArchFS);
}

static MCAsmInfo *createWDC65816MCAsmInfo(const MCRegisterInfo &MRI,
                                      const Triple &TT,
                                      const MCTargetOptions &TO) {
  MCAsmInfo *MAI = new WDC65816ELFMCAsmInfo(TT);

  // Initialize initial frame state.
  // Calculate amount of bytes used for return address storing
  int StackGrowth = -4;

  // Initial state of the frame pointer is SP+StackGrowth.
  // TODO: Add tests for `cfi_*` directives
  MCCFIInstruction Inst = MCCFIInstruction::cfiDefCfa(
      nullptr, MRI.getDwarfRegNum(llvm::WDC65816::SP, true), -StackGrowth);
  MAI->addInitialFrameState(Inst);

  // Add return address to move list
  Inst = MCCFIInstruction::createOffset(
      nullptr, MRI.getDwarfRegNum(WDC65816::PC, true), StackGrowth);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCRelocationInfo *createWDC65816MCRelocationInfo(const Triple &TheTriple,
                                                    MCContext &Ctx) {
  // Default to the stock relocation info.
  return llvm::createMCRelocationInfo(TheTriple, Ctx);
}

static MCInstPrinter *createWDC65816MCInstPrinter(const Triple &T,
                                              unsigned SyntaxVariant,
                                              const MCAsmInfo &MAI,
                                              const MCInstrInfo &MII,
                                              const MCRegisterInfo &MRI) {
  return new WDC65816InstPrinter(MAI, MII, MRI);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeWDC65816TargetMC() {
  Target &T = getTheWDC65816Target();

  // Register the MC asm info.
  RegisterMCAsmInfoFn X(T, createWDC65816MCAsmInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(T, createWDC65816MCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(T, createWDC65816MCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(T, createWDC65816MCSubtargetInfo);

  // Register the code emitter.
  TargetRegistry::RegisterMCCodeEmitter(T, createWDC65816MCCodeEmitter);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(T, createWDC65816MCInstPrinter);

  // Register the MC relocation info.
  TargetRegistry::RegisterMCRelocationInfo(T, createWDC65816MCRelocationInfo);

  // Register the asm backend.
  TargetRegistry::RegisterMCAsmBackend(T, createWDC65816AsmBackend);
}
