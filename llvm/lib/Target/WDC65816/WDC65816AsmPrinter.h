//===-- WDC65816AsmPrinter.h - WDC65816 LLVM Assembly Printer -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains WDC65816 assembler printer declarations.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_WDC65816ASMPRINTER_H
#define LLVM_LIB_TARGET_WDC65816_WDC65816ASMPRINTER_H

#include "WDC65816MCInstLower.h"
#include "WDC65816TargetMachine.h"
#include "MCTargetDesc/WDC65816MemOperandPrinter.h"

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>
#include <utility>

namespace llvm {
class MCStreamer;
class MachineInstr;
class MachineBasicBlock;
class Module;
class raw_ostream;

class WDC65816Subtarget;
class WDC65816MachineFunctionInfo;

class LLVM_LIBRARY_VISIBILITY WDC65816AsmPrinter
    : public AsmPrinter,
      public WDC65816MemOperandPrinter<WDC65816AsmPrinter, MachineInstr> {

  friend class WDC65816MemOperandPrinter;

  void EmitInstrWithMacroNoAT(const MachineInstr *MI);

  void printOperand(const MachineInstr *MI, int OpNum, raw_ostream &OS);

  void printDisp(const MachineInstr *MI, unsigned OpNum, raw_ostream &OS);
  void printAbsMem(const MachineInstr *MI, unsigned OpNum, raw_ostream &OS);

public:
  const WDC65816Subtarget *Subtarget;
  const WDC65816MachineFunctionInfo *MMFI;
  std::unique_ptr<WDC65816MCInstLower> MCInstLowering;

  explicit WDC65816AsmPrinter(TargetMachine &TM,
                          std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)) {
    Subtarget = static_cast<WDC65816TargetMachine &>(TM).getSubtargetImpl();
  }

  StringRef getPassName() const override { return "WDC65816 Assembly Printer"; }

  virtual bool runOnMachineFunction(MachineFunction &MF) override;

  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                       const char *ExtraCode, raw_ostream &OS) override;
  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                             const char *ExtraCode, raw_ostream &OS) override;

  void emitInstruction(const MachineInstr *MI) override;
  void emitFunctionBodyStart() override;
  void emitFunctionBodyEnd() override;
  void emitStartOfAsmFile(Module &M) override;
  void emitEndOfAsmFile(Module &M) override;
};
} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_WDC65816ASMPRINTER_H
