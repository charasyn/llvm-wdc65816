//===-- WDC65816InstrInfo.cpp - WDC65816 Instruction Information --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the WDC65816 declaration of the TargetInstrInfo class.
///
//===----------------------------------------------------------------------===//

#include "WDC65816InstrInfo.h"

#include "WDC65816InstrBuilder.h"
#include "WDC65816MachineFunction.h"
#include "WDC65816TargetMachine.h"
#include "MCTargetDesc/WDC65816MCCodeEmitter.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/ScopeExit.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Regex.h"

#include <functional>

using namespace llvm;

#define DEBUG_TYPE "WDC65816-instr-info"

#define GET_INSTRINFO_CTOR_DTOR
#include "WDC65816GenInstrInfo.inc"

// Pin the vtable to this file.
void WDC65816InstrInfo::anchor() {}

WDC65816InstrInfo::WDC65816InstrInfo(const WDC65816Subtarget &STI)
    : WDC65816GenInstrInfo(WDC65816::ADJCALLSTACKDOWN, WDC65816::ADJCALLSTACKUP, 0,
                       WDC65816::RET),
      Subtarget(STI), RI(STI) {}

static WDC65816::CondCode getCondFromBranchOpc(unsigned BrOpc) {
  switch (BrOpc) {
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

bool WDC65816InstrInfo::AnalyzeBranchImpl(MachineBasicBlock &MBB,
                                      MachineBasicBlock *&TBB,
                                      MachineBasicBlock *&FBB,
                                      SmallVectorImpl<MachineOperand> &Cond,
                                      bool AllowModify) const {

  auto UncondBranch =
      std::pair<MachineBasicBlock::reverse_iterator, MachineBasicBlock *>{
          MBB.rend(), nullptr};

  // Erase any instructions if allowed at the end of the scope.
  std::vector<std::reference_wrapper<llvm::MachineInstr>> EraseList;
  auto FinalizeOnReturn = llvm::make_scope_exit([&EraseList] {
    std::for_each(EraseList.begin(), EraseList.end(),
                  [](auto &ref) { ref.get().eraseFromParent(); });
  });

  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  for (auto iter = MBB.rbegin(); iter != MBB.rend(); iter = std::next(iter)) {

    unsigned Opcode = iter->getOpcode();

    if (iter->isDebugInstr())
      continue;

    // Working from the bottom, when we see a non-terminator instruction, we're
    // done.
    if (!isUnpredicatedTerminator(*iter))
      break;

    // A terminator that isn't a branch can't easily be handled by this
    // analysis.
    if (!iter->isBranch())
      return true;

    // Handle unconditional branches.
    if (Opcode == WDC65816::BRA8 || Opcode == WDC65816::BRA16) {
      if (!iter->getOperand(0).isMBB())
        return true;
      UncondBranch = {iter, iter->getOperand(0).getMBB()};

      // TBB is used to indicate the unconditional destination.
      TBB = UncondBranch.second;

      if (!AllowModify)
        continue;

      // If the block has any instructions after a JMP, erase them.
      EraseList.insert(EraseList.begin(), MBB.rbegin(), iter);

      Cond.clear();
      FBB = nullptr;

      // Erase the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(UncondBranch.second)) {
        TBB = nullptr;
        EraseList.push_back(*iter);
        UncondBranch = {MBB.rend(), nullptr};
      }

      continue;
    }

    // Handle conditional branches.
    auto BranchCode = WDC65816::GetCondFromBranchOpc(Opcode);

    // Can't handle indirect branch.
    if (BranchCode == WDC65816::COND_INVALID)
      return true;

    // In practice we should never have an undef CCR operand, if we do
    // abort here as we are not prepared to preserve the flag.
    // ??? Is this required?
    // if (iter->getOperand(1).isUndef())
    //   return true;

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      if (!iter->getOperand(0).isMBB())
        return true;
      MachineBasicBlock *CondBranchTarget = iter->getOperand(0).getMBB();

      // If we see something like this:
      //
      //     bcc l1
      //     bra l2
      //     ...
      //   l1:
      //     ...
      //   l2:
      if (UncondBranch.first != MBB.rend()) {

        assert(std::next(UncondBranch.first) == iter && "Wrong block layout.");

        // And we are allowed to modify the block and the target block of the
        // conditional branch is the direct successor of this block:
        //
        //     bcc l1
        //     bra l2
        //   l1:
        //     ...
        //   l2:
        //
        // we change it to this if allowed:
        //
        //     bncc l2
        //   l1:
        //     ...
        //   l2:
        //
        // Which is a bit more efficient.
        if (AllowModify && MBB.isLayoutSuccessor(CondBranchTarget)) {

          BranchCode = GetOppositeBranchCondition(BranchCode);
          unsigned BNCC = GetCondBranchFromCond(BranchCode);

          BuildMI(MBB, *UncondBranch.first, MBB.rfindDebugLoc(iter), get(BNCC))
              .addMBB(UncondBranch.second);

          EraseList.push_back(*iter);
          EraseList.push_back(*UncondBranch.first);

          TBB = UncondBranch.second;
          FBB = nullptr;
          Cond.push_back(MachineOperand::CreateImm(BranchCode));

          // Otherwise preserve TBB, FBB and Cond as requested
        } else {
          TBB = CondBranchTarget;
          FBB = UncondBranch.second;
          Cond.push_back(MachineOperand::CreateImm(BranchCode));
        }

        UncondBranch = {MBB.rend(), nullptr};
        continue;
      }

      TBB = CondBranchTarget;
      FBB = nullptr;
      Cond.push_back(MachineOperand::CreateImm(BranchCode));

      continue;
    }

    // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination and their condition
    // opcodes fit one of the special multi-branch idioms.
    assert(Cond.size() == 1);
    assert(TBB);

    // If the conditions are the same, we can leave them alone.
    auto OldBranchCode = static_cast<WDC65816::CondCode>(Cond[0].getImm());
    if (!iter->getOperand(0).isMBB())
      return true;
    auto NewTBB = iter->getOperand(0).getMBB();
    if (OldBranchCode == BranchCode && TBB == NewTBB)
      continue;

    // If they differ we cannot do much here.
    return true;
  }

  return false;
}

bool WDC65816InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                  MachineBasicBlock *&TBB,
                                  MachineBasicBlock *&FBB,
                                  SmallVectorImpl<MachineOperand> &Cond,
                                  bool AllowModify) const {
  return AnalyzeBranchImpl(MBB, TBB, FBB, Cond, AllowModify);
}

unsigned WDC65816InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                     int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugValue())
      continue;
    if (I->getOpcode() != WDC65816::BRA8 &&
        getCondFromBranchOpc(I->getOpcode()) == WDC65816::COND_INVALID)
      break;
    // Remove the branch.
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

unsigned WDC65816InstrInfo::insertBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB,
    ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  // Shouldn't be a fall through.
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "WDC65816 branch conditions have one component!");
  assert(!BytesAdded && "code size not handled");

  if (Cond.empty()) {
    // Unconditional branch?
    assert(!FBB && "Unconditional branch with multiple successors!");
    BuildMI(&MBB, DL, get(WDC65816::BRA8)).addMBB(TBB);
    return 1;
  }

  // If FBB is null, it is implied to be a fall-through block.
  bool FallThru = FBB == nullptr;

  // Conditional branch.
  unsigned Count = 0;
  WDC65816::CondCode CC = (WDC65816::CondCode)Cond[0].getImm();
  unsigned Opc = GetCondBranchFromCond(CC);
  BuildMI(&MBB, DL, get(Opc)).addMBB(TBB);
  ++Count;
  if (!FallThru) {
    // Two-way Conditional branch. Insert the second branch.
    BuildMI(&MBB, DL, get(WDC65816::BRA8)).addMBB(FBB);
    ++Count;
  }
  return Count;
}

void WDC65816InstrInfo::AddSExt(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator I, DebugLoc DL,
                            unsigned Reg, MVT From, MVT To) const {
  if (From == MVT::i8) {
    unsigned R = Reg;
    // EXT16 requires i16 register
    if (To == MVT::i32) {
      R = RI.getSubReg(Reg, WDC65816::MxSubRegIndex16Lo);
      assert(R && "No viable SUB register available");
    }
    BuildMI(MBB, I, DL, get(WDC65816::EXT16), R).addReg(R);
  }

  if (To == MVT::i32)
    BuildMI(MBB, I, DL, get(WDC65816::EXT32), Reg).addReg(Reg);
}

void WDC65816InstrInfo::AddZExt(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator I, DebugLoc DL,
                            unsigned Reg, MVT From, MVT To) const {

  unsigned Mask, And;
  if (From == MVT::i8)
    Mask = 0xFF;
  else
    Mask = 0xFFFF;

  if (To == MVT::i16)
    And = WDC65816::AND16di;
  else // i32
    And = WDC65816::AND32di;

  // TODO use xor r,r to decrease size
  BuildMI(MBB, I, DL, get(And), Reg).addReg(Reg).addImm(Mask);
}

// Convert MOVI to MOVQ if the target is a data register and the immediate
// fits in a sign-extended i8, otherwise emit a plain MOV.
bool WDC65816InstrInfo::ExpandMOVI(MachineInstrBuilder &MIB, MVT MVTSize) const {
  Register Reg = MIB->getOperand(0).getReg();
  int64_t Imm = MIB->getOperand(1).getImm();
  bool IsAddressReg = false;

  const auto *DR32 = RI.getRegClass(WDC65816::DR32RegClassID);
  const auto *AR32 = RI.getRegClass(WDC65816::AR32RegClassID);
  const auto *AR16 = RI.getRegClass(WDC65816::AR16RegClassID);

  if (AR16->contains(Reg) || AR32->contains(Reg))
    IsAddressReg = true;

  LLVM_DEBUG(dbgs() << "Expand " << *MIB.getInstr() << " to ");

  if (MVTSize == MVT::i8 || (!IsAddressReg && Imm >= -128 && Imm <= 127)) {
    LLVM_DEBUG(dbgs() << "MOVEQ\n");

    // We need to assign to the full register to make IV happy
    Register SReg =
        MVTSize == MVT::i32 ? Reg : Register(RI.getMatchingMegaReg(Reg, DR32));
    assert(SReg && "No viable MEGA register available");

    MIB->setDesc(get(WDC65816::MOVQ));
    MIB->getOperand(0).setReg(SReg);
  } else {
    LLVM_DEBUG(dbgs() << "MOVE\n");
    MIB->setDesc(get(MVTSize == MVT::i16 ? WDC65816::MOV16ri : WDC65816::MOV32ri));
  }

  return true;
}

bool WDC65816InstrInfo::ExpandMOVX_RR(MachineInstrBuilder &MIB, MVT MVTDst,
                                  MVT MVTSrc) const {
  unsigned Move = MVTDst == MVT::i16 ? WDC65816::MOV16rr : WDC65816::MOV32rr;
  Register Dst = MIB->getOperand(0).getReg();
  Register Src = MIB->getOperand(1).getReg();

  assert(Dst != Src && "You cannot use the same Regs with MOVX_RR");

  const auto &TRI = getRegisterInfo();

  const auto *RCDst = TRI.getMaximalPhysRegClass(Dst, MVTDst);
  const auto *RCSrc = TRI.getMaximalPhysRegClass(Src, MVTSrc);

  assert(RCDst && RCSrc && "Wrong use of MOVX_RR");
  assert(RCDst != RCSrc && "You cannot use the same Reg Classes with MOVX_RR");
  (void)RCSrc;

  // We need to find the super source register that matches the size of Dst
  unsigned SSrc = RI.getMatchingMegaReg(Src, RCDst);
  assert(SSrc && "No viable MEGA register available");

  DebugLoc DL = MIB->getDebugLoc();

  // If it happens to that super source register is the destination register
  // we do nothing
  if (Dst == SSrc) {
    LLVM_DEBUG(dbgs() << "Remove " << *MIB.getInstr() << '\n');
    MIB->eraseFromParent();
  } else { // otherwise we need to MOV
    LLVM_DEBUG(dbgs() << "Expand " << *MIB.getInstr() << " to MOV\n");
    MIB->setDesc(get(Move));
    MIB->getOperand(1).setReg(SSrc);
  }

  return true;
}

/// Expand SExt MOVE pseudos into a MOV and a EXT if the operands are two
/// different registers or just EXT if it is the same register
bool WDC65816InstrInfo::ExpandMOVSZX_RR(MachineInstrBuilder &MIB, bool IsSigned,
                                    MVT MVTDst, MVT MVTSrc) const {
  LLVM_DEBUG(dbgs() << "Expand " << *MIB.getInstr() << " to ");

  unsigned Move;

  if (MVTDst == MVT::i16)
    Move = WDC65816::MOV16rr;
  else // i32
    Move = WDC65816::MOV32rr;

  Register Dst = MIB->getOperand(0).getReg();
  Register Src = MIB->getOperand(1).getReg();

  assert(Dst != Src && "You cannot use the same Regs with MOVSX_RR");

  const auto &TRI = getRegisterInfo();

  const auto *RCDst = TRI.getMaximalPhysRegClass(Dst, MVTDst);
  const auto *RCSrc = TRI.getMaximalPhysRegClass(Src, MVTSrc);

  assert(RCDst && RCSrc && "Wrong use of MOVSX_RR");
  assert(RCDst != RCSrc && "You cannot use the same Reg Classes with MOVSX_RR");
  (void)RCSrc;

  // We need to find the super source register that matches the size of Dst
  unsigned SSrc = RI.getMatchingMegaReg(Src, RCDst);
  assert(SSrc && "No viable MEGA register available");

  MachineBasicBlock &MBB = *MIB->getParent();
  DebugLoc DL = MIB->getDebugLoc();

  if (Dst != SSrc) {
    LLVM_DEBUG(dbgs() << "Move and " << '\n');
    BuildMI(MBB, MIB.getInstr(), DL, get(Move), Dst).addReg(SSrc);
  }

  if (IsSigned) {
    LLVM_DEBUG(dbgs() << "Sign Extend" << '\n');
    AddSExt(MBB, MIB.getInstr(), DL, Dst, MVTSrc, MVTDst);
  } else {
    LLVM_DEBUG(dbgs() << "Zero Extend" << '\n');
    AddZExt(MBB, MIB.getInstr(), DL, Dst, MVTSrc, MVTDst);
  }

  MIB->eraseFromParent();

  return true;
}

bool WDC65816InstrInfo::ExpandMOVSZX_RM(MachineInstrBuilder &MIB, bool IsSigned,
                                    const MCInstrDesc &Desc, MVT MVTDst,
                                    MVT MVTSrc) const {
  LLVM_DEBUG(dbgs() << "Expand " << *MIB.getInstr() << " to LOAD and ");

  Register Dst = MIB->getOperand(0).getReg();

  // We need the subreg of Dst to make instruction verifier happy because the
  // real machine instruction consumes and produces values of the same size and
  // the registers the will be used here fall into different classes and this
  // makes IV cry. We could use a bigger operation, but this will put some
  // pressure on cache and memory, so no.
  unsigned SubDst =
      RI.getSubReg(Dst, MVTSrc == MVT::i8 ? WDC65816::MxSubRegIndex8Lo
                                          : WDC65816::MxSubRegIndex16Lo);
  assert(SubDst && "No viable SUB register available");

  // Make this a plain move
  MIB->setDesc(Desc);
  MIB->getOperand(0).setReg(SubDst);

  MachineBasicBlock::iterator I = MIB.getInstr();
  I++;
  MachineBasicBlock &MBB = *MIB->getParent();
  DebugLoc DL = MIB->getDebugLoc();

  if (IsSigned) {
    LLVM_DEBUG(dbgs() << "Sign Extend" << '\n');
    AddSExt(MBB, I, DL, Dst, MVTSrc, MVTDst);
  } else {
    LLVM_DEBUG(dbgs() << "Zero Extend" << '\n');
    AddZExt(MBB, I, DL, Dst, MVTSrc, MVTDst);
  }

  return true;
}

bool WDC65816InstrInfo::ExpandPUSH_POP(MachineInstrBuilder &MIB,
                                   const MCInstrDesc &Desc, bool IsPush) const {
  MachineBasicBlock::iterator I = MIB.getInstr();
  I++;
  MachineBasicBlock &MBB = *MIB->getParent();
  MachineOperand MO = MIB->getOperand(0);
  DebugLoc DL = MIB->getDebugLoc();
  if (IsPush)
    BuildMI(MBB, I, DL, Desc).addReg(RI.getStackRegister()).add(MO);
  else
    BuildMI(MBB, I, DL, Desc, MO.getReg()).addReg(RI.getStackRegister());

  MIB->eraseFromParent();
  return true;
}

bool WDC65816InstrInfo::ExpandCCR(MachineInstrBuilder &MIB, bool IsToCCR) const {

  // Replace the pseudo instruction with the real one
  if (IsToCCR)
    MIB->setDesc(get(WDC65816::MOV16cd));
  else
    // FIXME M68010 or later is required
    MIB->setDesc(get(WDC65816::MOV16dc));

  // Promote used register to the next class
  auto &Opd = MIB->getOperand(1);
  Opd.setReg(getRegisterInfo().getMatchingSuperReg(
      Opd.getReg(), WDC65816::MxSubRegIndex8Lo, &WDC65816::DR16RegClass));

  return true;
}

bool WDC65816InstrInfo::ExpandMOVEM(MachineInstrBuilder &MIB,
                                const MCInstrDesc &Desc, bool IsRM) const {
  int Reg = 0, Offset = 0, Base = 0;
  auto XR32 = RI.getRegClass(WDC65816::XR32RegClassID);
  auto DL = MIB->getDebugLoc();
  auto MI = MIB.getInstr();
  auto &MBB = *MIB->getParent();

  if (IsRM) {
    Reg = MIB->getOperand(0).getReg();
    Offset = MIB->getOperand(1).getImm();
    Base = MIB->getOperand(2).getReg();
  } else {
    Offset = MIB->getOperand(0).getImm();
    Base = MIB->getOperand(1).getReg();
    Reg = MIB->getOperand(2).getReg();
  }

  // If the register is not in XR32 then it is smaller than 32 bit, we
  // implicitly promote it to 32
  if (!XR32->contains(Reg)) {
    Reg = RI.getMatchingMegaReg(Reg, XR32);
    assert(Reg && "Has not meaningful MEGA register");
  }

  unsigned Mask = 1 << RI.getSpillRegisterOrder(Reg);
  if (IsRM) {
    BuildMI(MBB, MI, DL, Desc)
        .addImm(Mask)
        .addImm(Offset)
        .addReg(Base)
        .addReg(Reg, RegState::ImplicitDefine)
        .copyImplicitOps(*MIB);
  } else {
    BuildMI(MBB, MI, DL, Desc)
        .addImm(Offset)
        .addReg(Base)
        .addImm(Mask)
        .addReg(Reg, RegState::Implicit)
        .copyImplicitOps(*MIB);
  }

  MIB->eraseFromParent();

  return true;
}

/// Expand a single-def pseudo instruction to a two-addr
/// instruction with two undef reads of the register being defined.
/// This is used for mapping:
///   %d0 = SETCS_C32d
/// to:
///   %d0 = SUBX32dd %d0<undef>, %d0<undef>
///
static bool Expand2AddrUndef(MachineInstrBuilder &MIB,
                             const MCInstrDesc &Desc) {
  assert(Desc.getNumOperands() == 3 && "Expected two-addr instruction.");
  Register Reg = MIB->getOperand(0).getReg();
  MIB->setDesc(Desc);

  // MachineInstr::addOperand() will insert explicit operands before any
  // implicit operands.
  MIB.addReg(Reg, RegState::Undef).addReg(Reg, RegState::Undef);
  // But we don't trust that.
  assert(MIB->getOperand(1).getReg() == Reg &&
         MIB->getOperand(2).getReg() == Reg && "Misplaced operand");
  return true;
}

bool WDC65816InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineInstrBuilder MIB(*MI.getParent()->getParent(), MI);
  switch (MI.getOpcode()) {
  case WDC65816::PUSH8d:
    return ExpandPUSH_POP(MIB, get(WDC65816::MOV8ed), true);
  case WDC65816::PUSH16d:
    return ExpandPUSH_POP(MIB, get(WDC65816::MOV16er), true);
  case WDC65816::PUSH32r:
    return ExpandPUSH_POP(MIB, get(WDC65816::MOV32er), true);

  case WDC65816::POP8d:
    return ExpandPUSH_POP(MIB, get(WDC65816::MOV8do), false);
  case WDC65816::POP16d:
    return ExpandPUSH_POP(MIB, get(WDC65816::MOV16ro), false);
  case WDC65816::POP32r:
    return ExpandPUSH_POP(MIB, get(WDC65816::MOV32ro), false);

  case WDC65816::SETCS_C8d:
    return Expand2AddrUndef(MIB, get(WDC65816::SUBX8dd));
  case WDC65816::SETCS_C16d:
    return Expand2AddrUndef(MIB, get(WDC65816::SUBX16dd));
  case WDC65816::SETCS_C32d:
    return Expand2AddrUndef(MIB, get(WDC65816::SUBX32dd));
  }
  return false;
}

bool WDC65816InstrInfo::isPCRelRegisterOperandLegal(
    const MachineOperand &MO) const {
  assert(MO.isReg());

  // Check whether this MO belongs to an instruction with addressing mode 'k',
  // Refer to TargetInstrInfo.h for more information about this function.

  const MachineInstr *MI = MO.getParent();
  const unsigned NameIndices = WDC65816InstrNameIndices[MI->getOpcode()];
  StringRef InstrName(&WDC65816InstrNameData[NameIndices]);
  const unsigned OperandNo = MO.getOperandNo();

  // If this machine operand is the 2nd operand, then check
  // whether the instruction has destination addressing mode 'k'.
  if (OperandNo == 1)
    return Regex("[A-Z]+(8|16|32)k[a-z](_TC)?$").match(InstrName);

  // If this machine operand is the last one, then check
  // whether the instruction has source addressing mode 'k'.
  if (OperandNo == MI->getNumExplicitOperands() - 1)
    return Regex("[A-Z]+(8|16|32)[a-z]k(_TC)?$").match(InstrName);

  return false;
}

void WDC65816InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                const DebugLoc &DL, MCRegister DstReg,
                                MCRegister SrcReg, bool KillSrc) const {
  unsigned Opc = 0;

  // First deal with the normal symmetric copies.
  if (WDC65816::XR32RegClass.contains(DstReg, SrcReg))
    Opc = WDC65816::MOV32rr;
  else if (WDC65816::XR16RegClass.contains(DstReg, SrcReg))
    Opc = WDC65816::MOV16rr;
  else if (WDC65816::DR8RegClass.contains(DstReg, SrcReg))
    Opc = WDC65816::MOV8dd;

  if (Opc) {
    BuildMI(MBB, MI, DL, get(Opc), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
    return;
  }

  // Now deal with asymmetrically sized copies. The cases that follow are upcast
  // moves.
  //
  // NOTE
  // These moves are not aware of type nature of these values and thus
  // won't do any SExt or ZExt and upper bits will basically contain garbage.
  MachineInstrBuilder MIB(*MBB.getParent(), MI);
  if (WDC65816::DR8RegClass.contains(SrcReg)) {
    if (WDC65816::XR16RegClass.contains(DstReg))
      Opc = WDC65816::MOVXd16d8;
    else if (WDC65816::XR32RegClass.contains(DstReg))
      Opc = WDC65816::MOVXd32d8;
  } else if (WDC65816::XR16RegClass.contains(SrcReg) &&
             WDC65816::XR32RegClass.contains(DstReg))
    Opc = WDC65816::MOVXd32d16;

  if (Opc) {
    BuildMI(MBB, MI, DL, get(Opc), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
    return;
  }

  bool FromCCR = SrcReg == WDC65816::CCR;
  bool FromSR = SrcReg == WDC65816::SR;
  bool ToCCR = DstReg == WDC65816::CCR;
  bool ToSR = DstReg == WDC65816::SR;

  if (FromCCR) {
    assert(WDC65816::DR8RegClass.contains(DstReg) &&
           "Need DR8 register to copy CCR");
    Opc = WDC65816::MOV8dc;
  } else if (ToCCR) {
    assert(WDC65816::DR8RegClass.contains(SrcReg) &&
           "Need DR8 register to copy CCR");
    Opc = WDC65816::MOV8cd;
  } else if (FromSR || ToSR)
    llvm_unreachable("Cannot emit SR copy instruction");

  if (Opc) {
    BuildMI(MBB, MI, DL, get(Opc), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
    return;
  }

  LLVM_DEBUG(dbgs() << "Cannot copy " << RI.getName(SrcReg) << " to "
                    << RI.getName(DstReg) << '\n');
  llvm_unreachable("Cannot emit physreg copy instruction");
}

namespace {
unsigned getLoadStoreRegOpcode(unsigned Reg, const TargetRegisterClass *RC,
                               const TargetRegisterInfo *TRI,
                               const WDC65816Subtarget &STI, bool load) {
  switch (TRI->getRegSizeInBits(*RC)) {
  default:
    llvm_unreachable("Unknown spill size");
  case 8:
    if (WDC65816::DR8RegClass.hasSubClassEq(RC))
      return load ? WDC65816::MOV8dp : WDC65816::MOV8pd;
    if (WDC65816::CCRCRegClass.hasSubClassEq(RC))
      return load ? WDC65816::MOV16cp : WDC65816::MOV16pc;

    llvm_unreachable("Unknown 1-byte regclass");
  case 16:
    assert(WDC65816::XR16RegClass.hasSubClassEq(RC) && "Unknown 2-byte regclass");
    return load ? WDC65816::MOVM16mp_P : WDC65816::MOVM16pm_P;
  case 32:
    assert(WDC65816::XR32RegClass.hasSubClassEq(RC) && "Unknown 4-byte regclass");
    return load ? WDC65816::MOVM32mp_P : WDC65816::MOVM32pm_P;
  }
}

unsigned getStoreRegOpcode(unsigned SrcReg, const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI,
                           const WDC65816Subtarget &STI) {
  return getLoadStoreRegOpcode(SrcReg, RC, TRI, STI, false);
}

unsigned getLoadRegOpcode(unsigned DstReg, const TargetRegisterClass *RC,
                          const TargetRegisterInfo *TRI,
                          const WDC65816Subtarget &STI) {
  return getLoadStoreRegOpcode(DstReg, RC, TRI, STI, true);
}
} // end anonymous namespace

bool WDC65816InstrInfo::getStackSlotRange(const TargetRegisterClass *RC,
                                      unsigned SubIdx, unsigned &Size,
                                      unsigned &Offset,
                                      const MachineFunction &MF) const {
  // The slot size must be the maximum size so we can easily use MOVEM.L
  Size = 4;
  Offset = 0;
  return true;
}

void WDC65816InstrInfo::storeRegToStackSlot(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg,
    bool IsKill, int FrameIndex, const TargetRegisterClass *RC,
    const TargetRegisterInfo *TRI, Register VReg) const {
  const MachineFrameInfo &MFI = MBB.getParent()->getFrameInfo();
  assert(MFI.getObjectSize(FrameIndex) >= TRI->getSpillSize(*RC) &&
         "Stack slot is too small to store");
  (void)MFI;

  unsigned Opc = getStoreRegOpcode(SrcReg, RC, TRI, Subtarget);
  DebugLoc DL = MBB.findDebugLoc(MI);
  // (0,FrameIndex) <- $reg
  WDC65816::addFrameReference(BuildMI(MBB, MI, DL, get(Opc)), FrameIndex)
      .addReg(SrcReg, getKillRegState(IsKill));
}

void WDC65816InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator MI,
                                         Register DstReg, int FrameIndex,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI,
                                         Register VReg) const {
  const MachineFrameInfo &MFI = MBB.getParent()->getFrameInfo();
  assert(MFI.getObjectSize(FrameIndex) >= TRI->getSpillSize(*RC) &&
         "Stack slot is too small to load");
  (void)MFI;

  unsigned Opc = getLoadRegOpcode(DstReg, RC, TRI, Subtarget);
  DebugLoc DL = MBB.findDebugLoc(MI);
  WDC65816::addFrameReference(BuildMI(MBB, MI, DL, get(Opc), DstReg), FrameIndex);
}

/// Return a virtual register initialized with the global base register
/// value. Output instructions required to initialize the register in the
/// function entry block, if necessary.
///
/// TODO Move this function to WDC65816MachineFunctionInfo.
unsigned WDC65816InstrInfo::getGlobalBaseReg(MachineFunction *MF) const {
  WDC65816MachineFunctionInfo *MxFI = MF->getInfo<WDC65816MachineFunctionInfo>();
  unsigned GlobalBaseReg = MxFI->getGlobalBaseReg();
  if (GlobalBaseReg != 0)
    return GlobalBaseReg;

  // Create the register. The code to initialize it is inserted later,
  // by the WDC65816GlobalBaseReg pass (below).
  //
  // NOTE
  // Normally WDC65816 uses A5 register as global base pointer but this will
  // create unnecessary spill if we use less then 4 registers in code; since A5
  // is callee-save anyway we could try to allocate caller-save first and if
  // lucky get one, otherwise it does not really matter which callee-save to
  // use.
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  GlobalBaseReg = RegInfo.createVirtualRegister(&WDC65816::AR32_NOSPRegClass);
  MxFI->setGlobalBaseReg(GlobalBaseReg);
  return GlobalBaseReg;
}

std::pair<unsigned, unsigned>
WDC65816InstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char *>>
WDC65816InstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  using namespace WDC65816II;
  static const std::pair<unsigned, const char *> TargetFlags[] = {
      {MO_ABSOLUTE_ADDRESS, "m68k-absolute"},
      {MO_PC_RELATIVE_ADDRESS, "m68k-pcrel"},
      {MO_GOT, "m68k-got"},
      {MO_GOTOFF, "m68k-gotoff"},
      {MO_GOTPCREL, "m68k-gotpcrel"},
      {MO_PLT, "m68k-plt"},
      {MO_TLSGD, "m68k-tlsgd"},
      {MO_TLSLD, "m68k-tlsld"},
      {MO_TLSLDM, "m68k-tlsldm"},
      {MO_TLSIE, "m68k-tlsie"},
      {MO_TLSLE, "m68k-tlsle"}};
  return ArrayRef(TargetFlags);
}

#undef DEBUG_TYPE
#define DEBUG_TYPE "m68k-create-global-base-reg"

#define PASS_NAME "WDC65816 PIC Global Base Reg Initialization"

namespace {
/// This initializes the PIC global base register
struct WDC65816GlobalBaseReg : public MachineFunctionPass {
  static char ID;
  WDC65816GlobalBaseReg() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override {
    const WDC65816Subtarget &STI = MF.getSubtarget<WDC65816Subtarget>();
    WDC65816MachineFunctionInfo *MxFI = MF.getInfo<WDC65816MachineFunctionInfo>();

    unsigned GlobalBaseReg = MxFI->getGlobalBaseReg();

    // If we didn't need a GlobalBaseReg, don't insert code.
    if (GlobalBaseReg == 0)
      return false;

    // Insert the set of GlobalBaseReg into the first MBB of the function
    MachineBasicBlock &FirstMBB = MF.front();
    MachineBasicBlock::iterator MBBI = FirstMBB.begin();
    DebugLoc DL = FirstMBB.findDebugLoc(MBBI);
    const WDC65816InstrInfo *TII = STI.getInstrInfo();

    // Generate lea (__GLOBAL_OFFSET_TABLE_,%PC), %A5
    BuildMI(FirstMBB, MBBI, DL, TII->get(WDC65816::LEA32q), GlobalBaseReg)
        .addExternalSymbol("_GLOBAL_OFFSET_TABLE_", WDC65816II::MO_GOTPCREL);

    return true;
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};
char WDC65816GlobalBaseReg::ID = 0;
} // namespace

INITIALIZE_PASS(WDC65816GlobalBaseReg, DEBUG_TYPE, PASS_NAME, false, false)

FunctionPass *llvm::createWDC65816GlobalBaseRegPass() {
  return new WDC65816GlobalBaseReg();
}
