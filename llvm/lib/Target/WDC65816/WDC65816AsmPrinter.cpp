//===-- WDC65816AsmPrinter.cpp - WDC65816 LLVM Assembly Printer ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains a printer that converts from our internal representation
/// of machine-dependent LLVM code to GAS-format WDC65816 assembly language.
///
//===----------------------------------------------------------------------===//

// TODO Conform to Motorola ASM syntax

#include "WDC65816AsmPrinter.h"

#include "WDC65816.h"
#include "WDC65816MachineFunction.h"
#include "MCTargetDesc/WDC65816InstPrinter.h"
#include "TargetInfo/WDC65816TargetInfo.h"

#include "llvm/MC/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "m68k-asm-printer"

bool WDC65816AsmPrinter::runOnMachineFunction(MachineFunction &MF) {
  MMFI = MF.getInfo<WDC65816MachineFunctionInfo>();
  MCInstLowering = std::make_unique<WDC65816MCInstLower>(MF, *this);
  AsmPrinter::runOnMachineFunction(MF);
  return true;
}

void WDC65816AsmPrinter::printOperand(const MachineInstr *MI, int OpNum,
                                  raw_ostream &OS) {
  const MachineOperand &MO = MI->getOperand(OpNum);
  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    OS << "%" << WDC65816InstPrinter::getRegisterName(MO.getReg());
    break;
  case MachineOperand::MO_Immediate:
    OS << '#' << MO.getImm();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MO.getMBB()->getSymbol()->print(OS, MAI);
    break;
  case MachineOperand::MO_GlobalAddress:
    PrintSymbolOperand(MO, OS);
    break;
  case MachineOperand::MO_BlockAddress:
    GetBlockAddressSymbol(MO.getBlockAddress())->print(OS, MAI);
    break;
  case MachineOperand::MO_ConstantPoolIndex: {
    const DataLayout &DL = getDataLayout();
    OS << DL.getPrivateGlobalPrefix() << "CPI" << getFunctionNumber() << '_'
       << MO.getIndex();
    break;
  }
  default:
    llvm_unreachable("not implemented");
  }
}

bool WDC65816AsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                     const char *ExtraCode, raw_ostream &OS) {
  // Print the operand if there is no operand modifier.
  if (!ExtraCode || !ExtraCode[0]) {
    printOperand(MI, OpNo, OS);
    return false;
  }

  // Fallback to the default implementation.
  return AsmPrinter::PrintAsmOperand(MI, OpNo, ExtraCode, OS);
}

void WDC65816AsmPrinter::printDisp(const MachineInstr *MI, unsigned opNum,
                               raw_ostream &O) {
  // Print immediate displacement without the '#' predix
  const MachineOperand &Op = MI->getOperand(opNum);
  if (Op.isImm()) {
    O << Op.getImm();
    return;
  }
  // Displacement is relocatable, so we're pretty permissive about what
  // can be put here.
  printOperand(MI, opNum, O);
}

void WDC65816AsmPrinter::printAbsMem(const MachineInstr *MI, unsigned OpNum,
                                 raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(OpNum);
  if (MO.isImm())
    O << format("$%0" PRIx64, (uint64_t)MO.getImm());
  else
    PrintAsmMemoryOperand(MI, OpNum, nullptr, O);
}

bool WDC65816AsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                           unsigned OpNo, const char *ExtraCode,
                                           raw_ostream &OS) {
  const MachineOperand &MO = MI->getOperand(OpNo);
  switch (MO.getType()) {
  case MachineOperand::MO_Immediate:
    // Immediate value that goes here is the addressing mode kind we set
    // in WDC65816DAGToDAGISel::SelectInlineAsmMemoryOperand.
    using namespace WDC65816;
    // Skip the addressing mode kind operand.
    ++OpNo;
    // Decode MemAddrModeKind.
    switch (static_cast<MemAddrModeKind>(MO.getImm())) {
    case MemAddrModeKind::j:
      printARIMem(MI, OpNo, OS);
      break;
    case MemAddrModeKind::o:
      printARIPIMem(MI, OpNo, OS);
      break;
    case MemAddrModeKind::e:
      printARIPDMem(MI, OpNo, OS);
      break;
    case MemAddrModeKind::p:
      printARIDMem(MI, OpNo, OS);
      break;
    case MemAddrModeKind::f:
    case MemAddrModeKind::F:
      printARIIMem(MI, OpNo, OS);
      break;
    case MemAddrModeKind::k:
      printPCIMem(MI, 0, OpNo, OS);
      break;
    case MemAddrModeKind::q:
      printPCDMem(MI, 0, OpNo, OS);
      break;
    case MemAddrModeKind::b:
      printAbsMem(MI, OpNo, OS);
      break;
    default:
      llvm_unreachable("Unrecognized memory addressing mode");
    }
    return false;
  case MachineOperand::MO_GlobalAddress:
    PrintSymbolOperand(MO, OS);
    return false;
  case MachineOperand::MO_BlockAddress:
    GetBlockAddressSymbol(MO.getBlockAddress())->print(OS, MAI);
    return false;
  case MachineOperand::MO_Register:
    // This is a special case where it is treated as a memory reference, with
    // the register holding the address value. Thus, we print it as ARI here.
    if (WDC65816II::isAddressRegister(MO.getReg())) {
      printARIMem(MI, OpNo, OS);
      return false;
    }
    break;
  default:
    break;
  }
  return AsmPrinter::PrintAsmMemoryOperand(MI, OpNo, ExtraCode, OS);
}

void WDC65816AsmPrinter::emitInstruction(const MachineInstr *MI) {
  WDC65816_MC::verifyInstructionPredicates(MI->getOpcode(),
                                       getSubtargetInfo().getFeatureBits());

  switch (MI->getOpcode()) {
  default: {
    if (MI->isPseudo()) {
      LLVM_DEBUG(dbgs() << "Pseudo opcode(" << MI->getOpcode()
                        << ") found in EmitInstruction()\n");
      llvm_unreachable("Cannot proceed");
    }
    break;
  }
  case WDC65816::TAILJMPj:
  case WDC65816::TAILJMPq:
    // Lower these as normal, but add some comments.
    OutStreamer->AddComment("TAILCALL");
    break;
  }

  MCInst TmpInst0;
  MCInstLowering->Lower(MI, TmpInst0);
  OutStreamer->emitInstruction(TmpInst0, getSubtargetInfo());
}

void WDC65816AsmPrinter::emitFunctionBodyStart() {}

void WDC65816AsmPrinter::emitFunctionBodyEnd() {}

void WDC65816AsmPrinter::emitStartOfAsmFile(Module &M) {
  OutStreamer->emitSyntaxDirective();
}

void WDC65816AsmPrinter::emitEndOfAsmFile(Module &M) {}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeWDC65816AsmPrinter() {
  RegisterAsmPrinter<WDC65816AsmPrinter> X(getTheWDC65816Target());
}
