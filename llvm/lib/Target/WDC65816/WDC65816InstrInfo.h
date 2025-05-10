//===-- WDC65816InstrInfo.h - WDC65816 Instruction Information ----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the WDC65816 implementation of the TargetInstrInfo class.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_WDC65816INSTRINFO_H
#define LLVM_LIB_TARGET_WDC65816_WDC65816INSTRINFO_H

#include "WDC65816.h"
#include "WDC65816RegisterInfo.h"

#include "MCTargetDesc/WDC65816BaseInfo.h"

#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "WDC65816GenInstrInfo.inc"

namespace llvm {

class WDC65816Subtarget;

namespace WDC65816 {
// These MUST be kept in sync with codes definitions in WDC65816InstrInfo.td
enum CondCode {
  COND_T = 0,   // True
  COND_F = 1,   // False
  COND_HI = 2,  // High
  COND_LS = 3,  // Less or Same
  COND_CC = 4,  // Carry Clear
  COND_CS = 5,  // Carry Set
  COND_NE = 6,  // Not Equal
  COND_EQ = 7,  // Equal
  COND_VC = 8,  // Overflow Clear
  COND_VS = 9,  // Overflow Set
  COND_PL = 10, // Plus
  COND_MI = 11, // Minus
  COND_GE = 12, // Greater or Equal
  COND_LT = 13, // Less Than
  COND_GT = 14, // Greater Than
  COND_LE = 15, // Less or Equal
  LAST_VALID_COND = COND_LE,
  COND_INVALID
};

// FIXME would be nice tablegen to generate these predicates and converters
// mb tag based

static inline WDC65816::CondCode GetOppositeBranchCondition(WDC65816::CondCode CC) {
  switch (CC) {
  default:
    llvm_unreachable("Illegal condition code!");
  case WDC65816::COND_T:
    return WDC65816::COND_F;
  case WDC65816::COND_F:
    return WDC65816::COND_T;
  case WDC65816::COND_HI:
    return WDC65816::COND_LS;
  case WDC65816::COND_LS:
    return WDC65816::COND_HI;
  case WDC65816::COND_CC:
    return WDC65816::COND_CS;
  case WDC65816::COND_CS:
    return WDC65816::COND_CC;
  case WDC65816::COND_NE:
    return WDC65816::COND_EQ;
  case WDC65816::COND_EQ:
    return WDC65816::COND_NE;
  case WDC65816::COND_VC:
    return WDC65816::COND_VS;
  case WDC65816::COND_VS:
    return WDC65816::COND_VC;
  case WDC65816::COND_PL:
    return WDC65816::COND_MI;
  case WDC65816::COND_MI:
    return WDC65816::COND_PL;
  case WDC65816::COND_GE:
    return WDC65816::COND_LT;
  case WDC65816::COND_LT:
    return WDC65816::COND_GE;
  case WDC65816::COND_GT:
    return WDC65816::COND_LE;
  case WDC65816::COND_LE:
    return WDC65816::COND_GT;
  }
}

static inline unsigned GetCondBranchFromCond(WDC65816::CondCode CC) {
  switch (CC) {
  default:
    llvm_unreachable("Illegal condition code!");
  case WDC65816::COND_EQ:
    return WDC65816::Beq8;
  case WDC65816::COND_NE:
    return WDC65816::Bne8;
  case WDC65816::COND_LT:
    return WDC65816::Blt8;
  case WDC65816::COND_LE:
    return WDC65816::Ble8;
  case WDC65816::COND_GT:
    return WDC65816::Bgt8;
  case WDC65816::COND_GE:
    return WDC65816::Bge8;
  case WDC65816::COND_CS:
    return WDC65816::Bcs8;
  case WDC65816::COND_LS:
    return WDC65816::Bls8;
  case WDC65816::COND_HI:
    return WDC65816::Bhi8;
  case WDC65816::COND_CC:
    return WDC65816::Bcc8;
  case WDC65816::COND_MI:
    return WDC65816::Bmi8;
  case WDC65816::COND_PL:
    return WDC65816::Bpl8;
  case WDC65816::COND_VS:
    return WDC65816::Bvs8;
  case WDC65816::COND_VC:
    return WDC65816::Bvc8;
  }
}

static inline WDC65816::CondCode GetCondFromBranchOpc(unsigned Opcode) {
  switch (Opcode) {
  default:
    return WDC65816::COND_INVALID;
  case WDC65816::Beq8:
    return WDC65816::COND_EQ;
  case WDC65816::Bne8:
    return WDC65816::COND_NE;
  case WDC65816::Blt8:
    return WDC65816::COND_LT;
  case WDC65816::Ble8:
    return WDC65816::COND_LE;
  case WDC65816::Bgt8:
    return WDC65816::COND_GT;
  case WDC65816::Bge8:
    return WDC65816::COND_GE;
  case WDC65816::Bcs8:
    return WDC65816::COND_CS;
  case WDC65816::Bls8:
    return WDC65816::COND_LS;
  case WDC65816::Bhi8:
    return WDC65816::COND_HI;
  case WDC65816::Bcc8:
    return WDC65816::COND_CC;
  case WDC65816::Bmi8:
    return WDC65816::COND_MI;
  case WDC65816::Bpl8:
    return WDC65816::COND_PL;
  case WDC65816::Bvs8:
    return WDC65816::COND_VS;
  case WDC65816::Bvc8:
    return WDC65816::COND_VC;
  }
}

static inline unsigned IsCMP(unsigned Op) {
  switch (Op) {
  default:
    return false;
  case WDC65816::CMP8dd:
  case WDC65816::CMP8df:
  case WDC65816::CMP8di:
  case WDC65816::CMP8dj:
  case WDC65816::CMP8dp:
  case WDC65816::CMP16dr:
  case WDC65816::CMP16df:
  case WDC65816::CMP16di:
  case WDC65816::CMP16dj:
  case WDC65816::CMP16dp:
    return true;
  }
}

static inline bool IsSETCC(unsigned SETCC) {
  switch (SETCC) {
  default:
    return false;
  case WDC65816::SETd8eq:
  case WDC65816::SETd8ne:
  case WDC65816::SETd8lt:
  case WDC65816::SETd8ge:
  case WDC65816::SETd8le:
  case WDC65816::SETd8gt:
  case WDC65816::SETd8cs:
  case WDC65816::SETd8cc:
  case WDC65816::SETd8ls:
  case WDC65816::SETd8hi:
  case WDC65816::SETd8pl:
  case WDC65816::SETd8mi:
  case WDC65816::SETd8vc:
  case WDC65816::SETd8vs:
  case WDC65816::SETj8eq:
  case WDC65816::SETj8ne:
  case WDC65816::SETj8lt:
  case WDC65816::SETj8ge:
  case WDC65816::SETj8le:
  case WDC65816::SETj8gt:
  case WDC65816::SETj8cs:
  case WDC65816::SETj8cc:
  case WDC65816::SETj8ls:
  case WDC65816::SETj8hi:
  case WDC65816::SETj8pl:
  case WDC65816::SETj8mi:
  case WDC65816::SETj8vc:
  case WDC65816::SETj8vs:
  case WDC65816::SETp8eq:
  case WDC65816::SETp8ne:
  case WDC65816::SETp8lt:
  case WDC65816::SETp8ge:
  case WDC65816::SETp8le:
  case WDC65816::SETp8gt:
  case WDC65816::SETp8cs:
  case WDC65816::SETp8cc:
  case WDC65816::SETp8ls:
  case WDC65816::SETp8hi:
  case WDC65816::SETp8pl:
  case WDC65816::SETp8mi:
  case WDC65816::SETp8vc:
  case WDC65816::SETp8vs:
    return true;
  }
}

} // namespace WDC65816

class WDC65816InstrInfo : public WDC65816GenInstrInfo {
  virtual void anchor();

protected:
  const WDC65816Subtarget &Subtarget;
  const WDC65816RegisterInfo RI;

public:
  explicit WDC65816InstrInfo(const WDC65816Subtarget &STI);

  static const WDC65816InstrInfo *create(WDC65816Subtarget &STI);

  /// TargetInstrInfo is a superset of MRegister info. As such, whenever a
  /// client has an instance of instruction info, it should always be able to
  /// get register info as well (through this method).
  const WDC65816RegisterInfo &getRegisterInfo() const { return RI; };

  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                     MachineBasicBlock *&FBB,
                     SmallVectorImpl<MachineOperand> &Cond,
                     bool AllowModify) const override;

  bool AnalyzeBranchImpl(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                         MachineBasicBlock *&FBB,
                         SmallVectorImpl<MachineOperand> &Cond,
                         bool AllowModify) const;

  unsigned removeBranch(MachineBasicBlock &MBB,
                        int *BytesRemoved = nullptr) const override;

  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                        MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                        const DebugLoc &DL,
                        int *BytesAdded = nullptr) const override;

  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                   const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg,
                   bool KillSrc) const override;

  bool getStackSlotRange(const TargetRegisterClass *RC, unsigned SubIdx,
                         unsigned &Size, unsigned &Offset,
                         const MachineFunction &MF) const override;

  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI, Register SrcReg,
                           bool IsKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI,
                           Register VReg) const override;

  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MI, Register DestReg,
                            int FrameIndex, const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI,
                            Register VReg) const override;

  bool expandPostRAPseudo(MachineInstr &MI) const override;

  bool isPCRelRegisterOperandLegal(const MachineOperand &MO) const override;

  /// Add appropriate SExt nodes
  void AddSExt(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
               DebugLoc DL, unsigned Reg, MVT From, MVT To) const;

  /// Add appropriate ZExt nodes
  void AddZExt(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
               DebugLoc DL, unsigned Reg, MVT From, MVT To) const;

  /// Move immediate to register
  bool ExpandMOVI(MachineInstrBuilder &MIB, MVT MVTSize) const;

  /// Move across register classes without extension
  bool ExpandMOVX_RR(MachineInstrBuilder &MIB, MVT MVTDst, MVT MVTSrc) const;

  /// Move from register and extend
  bool ExpandMOVSZX_RR(MachineInstrBuilder &MIB, bool IsSigned, MVT MVTDst,
                       MVT MVTSrc) const;

  /// Move from memory and extend
  bool ExpandMOVSZX_RM(MachineInstrBuilder &MIB, bool IsSigned,
                       const MCInstrDesc &Desc, MVT MVTDst, MVT MVTSrc) const;

  /// Push/Pop to/from stack
  bool ExpandPUSH_POP(MachineInstrBuilder &MIB, const MCInstrDesc &Desc,
                      bool IsPush) const;

  /// Moves to/from CCR
  bool ExpandCCR(MachineInstrBuilder &MIB, bool IsToCCR) const;

  /// Expand all MOVEM pseudos into real MOVEMs
  bool ExpandMOVEM(MachineInstrBuilder &MIB, const MCInstrDesc &Desc,
                   bool IsRM) const;

  /// Return a virtual register initialized with the global base register
  /// value. Output instructions required to initialize the register in the
  /// function entry block, if necessary.
  unsigned getGlobalBaseReg(MachineFunction *MF) const;

  std::pair<unsigned, unsigned>
  decomposeMachineOperandsTargetFlags(unsigned TF) const override;

  ArrayRef<std::pair<unsigned, const char *>>
  getSerializableDirectMachineOperandTargetFlags() const override;
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_WDC65816INSTRINFO_H
