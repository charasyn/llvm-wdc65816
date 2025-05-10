//===-- WDC65816ExpandPseudo.cpp - Expand pseudo instructions -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains a pass that expands pseudo instructions into target
/// instructions to allow proper scheduling, if-conversion, other late
/// optimizations, or simply the encoding of the instructions.
///
//===----------------------------------------------------------------------===//

#include "WDC65816.h"
#include "WDC65816FrameLowering.h"
#include "WDC65816InstrInfo.h"
#include "WDC65816MachineFunction.h"
#include "WDC65816Subtarget.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h" // For IDs of passes that are preserved.
#include "llvm/IR/EHPersonalities.h"
#include "llvm/IR/GlobalValue.h"

using namespace llvm;

#define DEBUG_TYPE "m68k-expand-pseudo"
#define PASS_NAME "WDC65816 pseudo instruction expansion pass"

namespace {
class WDC65816ExpandPseudo : public MachineFunctionPass {
public:
  static char ID;
  WDC65816ExpandPseudo() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    AU.addPreservedID(MachineLoopInfoID);
    AU.addPreservedID(MachineDominatorsID);
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  const WDC65816Subtarget *STI;
  const WDC65816InstrInfo *TII;
  const WDC65816RegisterInfo *TRI;
  const WDC65816MachineFunctionInfo *MFI;
  const WDC65816FrameLowering *FL;

  bool runOnMachineFunction(MachineFunction &Fn) override;

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().set(
        MachineFunctionProperties::Property::NoVRegs);
  }

private:
  bool ExpandMI(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI);
  bool ExpandMBB(MachineBasicBlock &MBB);
};
char WDC65816ExpandPseudo::ID = 0;
} // End anonymous namespace.

INITIALIZE_PASS(WDC65816ExpandPseudo, DEBUG_TYPE, PASS_NAME, false, false)

/// If \p MBBI is a pseudo instruction, this method expands
/// it to the corresponding (sequence of) actual instruction(s).
/// \returns true if \p MBBI has been expanded.
bool WDC65816ExpandPseudo::ExpandMI(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MBBI) {
  MachineInstr &MI = *MBBI;
  MachineInstrBuilder MIB(*MI.getParent()->getParent(), MI);
  unsigned Opcode = MI.getOpcode();
  DebugLoc DL = MBBI->getDebugLoc();
  /// TODO infer argument size to create less switch cases
  switch (Opcode) {
  default:
    return false;

  case WDC65816::MOVI8di:
    return TII->ExpandMOVI(MIB, MVT::i8);
  case WDC65816::MOVI16ri:
    return TII->ExpandMOVI(MIB, MVT::i16);
  case WDC65816::MOVI32ri:
    return TII->ExpandMOVI(MIB, MVT::i32);

  case WDC65816::MOVXd16d8:
    return TII->ExpandMOVX_RR(MIB, MVT::i16, MVT::i8);
  case WDC65816::MOVXd32d8:
    return TII->ExpandMOVX_RR(MIB, MVT::i32, MVT::i8);
  case WDC65816::MOVXd32d16:
    return TII->ExpandMOVX_RR(MIB, MVT::i32, MVT::i16);

  case WDC65816::MOVSXd16d8:
    return TII->ExpandMOVSZX_RR(MIB, true, MVT::i16, MVT::i8);
  case WDC65816::MOVSXd32d8:
    return TII->ExpandMOVSZX_RR(MIB, true, MVT::i32, MVT::i8);
  case WDC65816::MOVSXd32d16:
    return TII->ExpandMOVSZX_RR(MIB, true, MVT::i32, MVT::i16);

  case WDC65816::MOVZXd16d8:
    return TII->ExpandMOVSZX_RR(MIB, false, MVT::i16, MVT::i8);
  case WDC65816::MOVZXd32d8:
    return TII->ExpandMOVSZX_RR(MIB, false, MVT::i32, MVT::i8);
  case WDC65816::MOVZXd32d16:
    return TII->ExpandMOVSZX_RR(MIB, false, MVT::i32, MVT::i16);

  case WDC65816::MOVSXd16j8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8dj), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVSXd32j8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8dj), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVSXd32j16:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV16rj), MVT::i32,
                                MVT::i16);

  case WDC65816::MOVZXd16j8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8dj), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVZXd32j8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8dj), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVZXd32j16:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV16rj), MVT::i32,
                                MVT::i16);

  case WDC65816::MOVSXd16p8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8dp), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVSXd32p8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8dp), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVSXd32p16:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV16rp), MVT::i32,
                                MVT::i16);

  case WDC65816::MOVZXd16p8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8dp), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVZXd32p8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8dp), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVZXd32p16:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV16rp), MVT::i32,
                                MVT::i16);

  case WDC65816::MOVSXd16f8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8df), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVSXd32f8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8df), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVSXd32f16:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV16rf), MVT::i32,
                                MVT::i16);

  case WDC65816::MOVZXd16f8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8df), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVZXd32f8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8df), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVZXd32f16:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV16rf), MVT::i32,
                                MVT::i16);

  case WDC65816::MOVSXd16q8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8dq), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVSXd32q8:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV8dq), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVSXd32q16:
    return TII->ExpandMOVSZX_RM(MIB, true, TII->get(WDC65816::MOV16dq), MVT::i32,
                                MVT::i16);

  case WDC65816::MOVZXd16q8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8dq), MVT::i16,
                                MVT::i8);
  case WDC65816::MOVZXd32q8:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV8dq), MVT::i32,
                                MVT::i8);
  case WDC65816::MOVZXd32q16:
    return TII->ExpandMOVSZX_RM(MIB, false, TII->get(WDC65816::MOV16dq), MVT::i32,
                                MVT::i16);

  case WDC65816::MOV8cd:
    return TII->ExpandCCR(MIB, /*IsToCCR=*/true);
  case WDC65816::MOV8dc:
    return TII->ExpandCCR(MIB, /*IsToCCR=*/false);

  case WDC65816::MOVM8jm_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32jm), /*IsRM=*/false);
  case WDC65816::MOVM16jm_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32jm), /*IsRM=*/false);
  case WDC65816::MOVM32jm_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32jm), /*IsRM=*/false);

  case WDC65816::MOVM8pm_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32pm), /*IsRM=*/false);
  case WDC65816::MOVM16pm_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32pm), /*IsRM=*/false);
  case WDC65816::MOVM32pm_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32pm), /*IsRM=*/false);

  case WDC65816::MOVM8mj_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32mj), /*IsRM=*/true);
  case WDC65816::MOVM16mj_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32mj), /*IsRM=*/true);
  case WDC65816::MOVM32mj_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32mj), /*IsRM=*/true);

  case WDC65816::MOVM8mp_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32mp), /*IsRM=*/true);
  case WDC65816::MOVM16mp_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32mp), /*IsRM=*/true);
  case WDC65816::MOVM32mp_P:
    return TII->ExpandMOVEM(MIB, TII->get(WDC65816::MOVM32mp), /*IsRM=*/true);

  case WDC65816::TCRETURNq:
  case WDC65816::TCRETURNj: {
    MachineOperand &JumpTarget = MI.getOperand(0);
    MachineOperand &StackAdjust = MI.getOperand(1);
    assert(StackAdjust.isImm() && "Expecting immediate value.");

    // Adjust stack pointer.
    int StackAdj = StackAdjust.getImm();
    int MaxTCDelta = MFI->getTCReturnAddrDelta();
    int Offset = 0;
    assert(MaxTCDelta <= 0 && "MaxTCDelta should never be positive");

    // Incoporate the retaddr area.
    Offset = StackAdj - MaxTCDelta;
    assert(Offset >= 0 && "Offset should never be negative");

    if (Offset) {
      // Check for possible merge with preceding ADD instruction.
      Offset += FL->mergeSPUpdates(MBB, MBBI, true);
      FL->emitSPUpdate(MBB, MBBI, Offset, /*InEpilogue=*/true);
    }

    // Jump to label or value in register.
    if (Opcode == WDC65816::TCRETURNq) {
      MachineInstrBuilder MIB =
          BuildMI(MBB, MBBI, DL, TII->get(WDC65816::TAILJMPq));
      if (JumpTarget.isGlobal()) {
        MIB.addGlobalAddress(JumpTarget.getGlobal(), JumpTarget.getOffset(),
                             JumpTarget.getTargetFlags());
      } else {
        assert(JumpTarget.isSymbol());
        MIB.addExternalSymbol(JumpTarget.getSymbolName(),
                              JumpTarget.getTargetFlags());
      }
    } else {
      BuildMI(MBB, MBBI, DL, TII->get(WDC65816::TAILJMPj))
          .addReg(JumpTarget.getReg(), RegState::Kill);
    }

    MachineInstr &NewMI = *std::prev(MBBI);
    NewMI.copyImplicitOps(*MBBI->getParent()->getParent(), *MBBI);

    // Delete the pseudo instruction TCRETURN.
    MBB.erase(MBBI);

    return true;
  }
  case WDC65816::RET: {
    if (MBB.getParent()->getFunction().getCallingConv() ==
        CallingConv::WDC65816_INTR) {
      BuildMI(MBB, MBBI, DL, TII->get(WDC65816::RTE));
    } else if (int64_t StackAdj = MBBI->getOperand(0).getImm(); StackAdj == 0) {
      BuildMI(MBB, MBBI, DL, TII->get(WDC65816::RTS));
    } else {
      // Copy return address from stack to a free address(A0 or A1) register
      // TODO check if pseudo expand uses free address register
      BuildMI(MBB, MBBI, DL, TII->get(WDC65816::MOV32aj), WDC65816::A1)
          .addReg(WDC65816::SP);

      // Adjust SP
      FL->emitSPUpdate(MBB, MBBI, StackAdj, /*InEpilogue=*/true);

      // Put the return address on stack
      BuildMI(MBB, MBBI, DL, TII->get(WDC65816::MOV32ja))
          .addReg(WDC65816::SP)
          .addReg(WDC65816::A1);

      // RTS
      BuildMI(MBB, MBBI, DL, TII->get(WDC65816::RTS));
    }

    // FIXME: Can rest of the operands be ignored, if there is any?
    MBB.erase(MBBI);
    return true;
  }
  }
  llvm_unreachable("Previous switch has a fallthrough?");
}

/// Expand all pseudo instructions contained in \p MBB.
/// \returns true if any expansion occurred for \p MBB.
bool WDC65816ExpandPseudo::ExpandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  // MBBI may be invalidated by the expansion.
  MachineBasicBlock::iterator MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    MachineBasicBlock::iterator NMBBI = std::next(MBBI);
    Modified |= ExpandMI(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool WDC65816ExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  STI = &MF.getSubtarget<WDC65816Subtarget>();
  TII = STI->getInstrInfo();
  TRI = STI->getRegisterInfo();
  MFI = MF.getInfo<WDC65816MachineFunctionInfo>();
  FL = STI->getFrameLowering();

  bool Modified = false;
  for (MachineBasicBlock &MBB : MF)
    Modified |= ExpandMBB(MBB);
  return Modified;
}

/// Returns an instance of the pseudo instruction expansion pass.
FunctionPass *llvm::createWDC65816ExpandPseudoPass() {
  return new WDC65816ExpandPseudo();
}
