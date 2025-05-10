//===-- WDC65816AsmParser.cpp - Parse WDC65816 assembly to MCInst instructions ----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "WDC65816InstrInfo.h"
#include "WDC65816RegisterInfo.h"
#include "TargetInfo/WDC65816TargetInfo.h"

#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/TargetRegistry.h"

#include <sstream>

#define DEBUG_TYPE "m68k-asm-parser"

using namespace llvm;

static cl::opt<bool> RegisterPrefixOptional(
    "m68k-register-prefix-optional", cl::Hidden,
    cl::desc("Enable specifying registers without the % prefix"),
    cl::init(false));

namespace {
/// Parses WDC65816 assembly from a stream.
class WDC65816AsmParser : public MCTargetAsmParser {
  const MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;

#define GET_ASSEMBLER_HEADER
#include "WDC65816GenAsmMatcher.inc"

  // Helpers for Match&Emit.
  bool invalidOperand(const SMLoc &Loc, const OperandVector &Operands,
                      const uint64_t &ErrorInfo);
  bool missingFeature(const SMLoc &Loc, const uint64_t &ErrorInfo);
  bool emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const;
  bool parseRegisterName(MCRegister &RegNo, SMLoc Loc, StringRef RegisterName);
  ParseStatus parseRegister(MCRegister &RegNo);

  // Parser functions.
  void eatComma();

  bool isExpr();
  ParseStatus parseImm(OperandVector &Operands);
  ParseStatus parseMemOp(OperandVector &Operands);
  ParseStatus parseRegOrMoveMask(OperandVector &Operands);

public:
  WDC65816AsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
                const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), STI(STI), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;
  bool parseRegister(MCRegister &Reg, SMLoc &StartLoc, SMLoc &EndLoc) override;
  ParseStatus tryParseRegister(MCRegister &Reg, SMLoc &StartLoc,
                               SMLoc &EndLoc) override;
  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;
  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;
};

struct WDC65816MemOp {
  enum class Kind {
    Addr,
    RegMask,
    Reg,
    RegIndirect,
    RegPostIncrement,
    RegPreDecrement,
    RegIndirectDisplacement,
    RegIndirectDisplacementIndex,
  };

  // These variables are used for the following forms:
  // Addr: (OuterDisp)
  // RegMask: RegMask (as register mask)
  // Reg: %OuterReg
  // RegIndirect: (%OuterReg)
  // RegPostIncrement: (%OuterReg)+
  // RegPreDecrement: -(%OuterReg)
  // RegIndirectDisplacement: OuterDisp(%OuterReg)
  // RegIndirectDisplacementIndex:
  //   OuterDisp(%OuterReg, %InnerReg.Size * Scale, InnerDisp)

  Kind Op;
  MCRegister OuterReg;
  MCRegister InnerReg;
  const MCExpr *OuterDisp;
  const MCExpr *InnerDisp;
  uint8_t Size : 4;
  uint8_t Scale : 4;
  const MCExpr *Expr;
  uint16_t RegMask;

  WDC65816MemOp() {}
  WDC65816MemOp(Kind Op) : Op(Op) {}

  void print(raw_ostream &OS) const;
};

/// An parsed WDC65816 assembly operand.
class WDC65816Operand : public MCParsedAsmOperand {
  typedef MCParsedAsmOperand Base;

  enum class KindTy {
    Invalid,
    Token,
    Imm,
    MemOp,
  };

  KindTy Kind;
  SMLoc Start, End;
  union {
    StringRef Token;
    const MCExpr *Expr;
    WDC65816MemOp MemOp;
  };

  template <unsigned N> bool isAddrN() const;

public:
  WDC65816Operand(KindTy Kind, SMLoc Start, SMLoc End)
      : Base(), Kind(Kind), Start(Start), End(End) {}

  SMLoc getStartLoc() const override { return Start; }
  SMLoc getEndLoc() const override { return End; }

  void print(raw_ostream &OS) const override;

  bool isMem() const override { return false; }
  bool isMemOp() const { return Kind == KindTy::MemOp; }

  static void addExpr(MCInst &Inst, const MCExpr *Expr);

  // Reg
  bool isReg() const override;
  bool isAReg() const;
  bool isDReg() const;
  bool isFPDReg() const;
  bool isFPCReg() const;
  MCRegister getReg() const override;
  void addRegOperands(MCInst &Inst, unsigned N) const;

  static std::unique_ptr<WDC65816Operand> createMemOp(WDC65816MemOp MemOp, SMLoc Start,
                                                  SMLoc End);

  // Token
  bool isToken() const override;
  StringRef getToken() const;
  static std::unique_ptr<WDC65816Operand> createToken(StringRef Token, SMLoc Start,
                                                  SMLoc End);

  // Imm
  bool isImm() const override;
  void addImmOperands(MCInst &Inst, unsigned N) const;

  static std::unique_ptr<WDC65816Operand> createImm(const MCExpr *Expr, SMLoc Start,
                                                SMLoc End);

  // Imm for TRAP instruction
  bool isTrapImm() const;
  // Imm for BKPT instruction
  bool isBkptImm() const;

  // MoveMask
  bool isMoveMask() const;
  void addMoveMaskOperands(MCInst &Inst, unsigned N) const;

  // Addr
  bool isAddr() const;
  bool isAddr8() const { return isAddrN<8>(); }
  bool isAddr16() const { return isAddrN<16>(); }
  bool isAddr32() const { return isAddrN<32>(); }
  void addAddrOperands(MCInst &Inst, unsigned N) const;

  // ARI
  bool isARI() const;
  void addARIOperands(MCInst &Inst, unsigned N) const;

  // ARID
  bool isARID() const;
  void addARIDOperands(MCInst &Inst, unsigned N) const;

  // ARII
  bool isARII() const;
  void addARIIOperands(MCInst &Inst, unsigned N) const;

  // ARIPD
  bool isARIPD() const;
  void addARIPDOperands(MCInst &Inst, unsigned N) const;

  // ARIPI
  bool isARIPI() const;
  void addARIPIOperands(MCInst &Inst, unsigned N) const;

  // PCD
  bool isPCD() const;
  void addPCDOperands(MCInst &Inst, unsigned N) const;

  // PCI
  bool isPCI() const;
  void addPCIOperands(MCInst &Inst, unsigned N) const;
};

} // end anonymous namespace.

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeWDC65816AsmParser() {
  RegisterMCAsmParser<WDC65816AsmParser> X(getTheWDC65816Target());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "WDC65816GenAsmMatcher.inc"

static inline unsigned getRegisterByIndex(unsigned RegisterIndex) {
  static unsigned RegistersByIndex[] = {
      WDC65816::D0,  WDC65816::D1,  WDC65816::D2,  WDC65816::D3,  WDC65816::D4,  WDC65816::D5,
      WDC65816::D6,  WDC65816::D7,  WDC65816::A0,  WDC65816::A1,  WDC65816::A2,  WDC65816::A3,
      WDC65816::A4,  WDC65816::A5,  WDC65816::A6,  WDC65816::SP,  WDC65816::FP0, WDC65816::FP1,
      WDC65816::FP2, WDC65816::FP3, WDC65816::FP4, WDC65816::FP5, WDC65816::FP6, WDC65816::FP7};
  assert(RegisterIndex <=
         sizeof(RegistersByIndex) / sizeof(RegistersByIndex[0]));
  return RegistersByIndex[RegisterIndex];
}

static inline unsigned getRegisterIndex(unsigned Register) {
  if (Register >= WDC65816::D0 && Register <= WDC65816::D7)
    return Register - WDC65816::D0;
  if (Register >= WDC65816::A0 && Register <= WDC65816::A6)
    return Register - WDC65816::A0 + 8;
  if (Register >= WDC65816::FP0 && Register <= WDC65816::FP7)
    return Register - WDC65816::FP0 + 16;

  switch (Register) {
  case WDC65816::SP:
    // SP is sadly not contiguous with the rest of the An registers
    return 15;

  // We don't care about the indices of these registers.
  case WDC65816::PC:
  case WDC65816::CCR:
  case WDC65816::FPC:
  case WDC65816::FPS:
  case WDC65816::FPIAR:
    return UINT_MAX;

  default:
    llvm_unreachable("unexpected register number");
  }
}

void WDC65816MemOp::print(raw_ostream &OS) const {
  switch (Op) {
  case Kind::Addr:
    OS << OuterDisp;
    break;
  case Kind::RegMask:
    OS << "RegMask(" << format("%04x", RegMask) << ")";
    break;
  case Kind::Reg:
    OS << '%' << OuterReg;
    break;
  case Kind::RegIndirect:
    OS << "(%" << OuterReg << ')';
    break;
  case Kind::RegPostIncrement:
    OS << "(%" << OuterReg << ")+";
    break;
  case Kind::RegPreDecrement:
    OS << "-(%" << OuterReg << ")";
    break;
  case Kind::RegIndirectDisplacement:
    OS << OuterDisp << "(%" << OuterReg << ")";
    break;
  case Kind::RegIndirectDisplacementIndex:
    OS << OuterDisp << "(%" << OuterReg << ", " << InnerReg << "." << Size
       << ", " << InnerDisp << ")";
    break;
  }
}

void WDC65816Operand::addExpr(MCInst &Inst, const MCExpr *Expr) {
  if (auto Const = dyn_cast<MCConstantExpr>(Expr)) {
    Inst.addOperand(MCOperand::createImm(Const->getValue()));
    return;
  }

  Inst.addOperand(MCOperand::createExpr(Expr));
}

// Reg
bool WDC65816Operand::isReg() const {
  return Kind == KindTy::MemOp && MemOp.Op == WDC65816MemOp::Kind::Reg;
}

MCRegister WDC65816Operand::getReg() const {
  assert(isReg());
  return MemOp.OuterReg;
}

void WDC65816Operand::addRegOperands(MCInst &Inst, unsigned N) const {
  assert(isReg() && "wrong operand kind");
  assert((N == 1) && "can only handle one register operand");

  Inst.addOperand(MCOperand::createReg(getReg()));
}

std::unique_ptr<WDC65816Operand> WDC65816Operand::createMemOp(WDC65816MemOp MemOp,
                                                      SMLoc Start, SMLoc End) {
  auto Op = std::make_unique<WDC65816Operand>(KindTy::MemOp, Start, End);
  Op->MemOp = MemOp;
  return Op;
}

// Token
bool WDC65816Operand::isToken() const { return Kind == KindTy::Token; }
StringRef WDC65816Operand::getToken() const {
  assert(isToken());
  return Token;
}

std::unique_ptr<WDC65816Operand> WDC65816Operand::createToken(StringRef Token,
                                                      SMLoc Start, SMLoc End) {
  auto Op = std::make_unique<WDC65816Operand>(KindTy::Token, Start, End);
  Op->Token = Token;
  return Op;
}

// Imm
bool WDC65816Operand::isImm() const { return Kind == KindTy::Imm; }
void WDC65816Operand::addImmOperands(MCInst &Inst, unsigned N) const {
  assert(isImm() && "wrong operand kind");
  assert((N == 1) && "can only handle one register operand");

  WDC65816Operand::addExpr(Inst, Expr);
}

std::unique_ptr<WDC65816Operand> WDC65816Operand::createImm(const MCExpr *Expr,
                                                    SMLoc Start, SMLoc End) {
  auto Op = std::make_unique<WDC65816Operand>(KindTy::Imm, Start, End);
  Op->Expr = Expr;
  return Op;
}

bool WDC65816Operand::isTrapImm() const {
  int64_t Value;
  if (!isImm() || !Expr->evaluateAsAbsolute(Value))
    return false;

  return isUInt<4>(Value);
}

bool WDC65816Operand::isBkptImm() const {
  int64_t Value;
  if (!isImm() || !Expr->evaluateAsAbsolute(Value))
    return false;

  return isUInt<3>(Value);
}

// MoveMask
bool WDC65816Operand::isMoveMask() const {
  if (!isMemOp())
    return false;

  if (MemOp.Op == WDC65816MemOp::Kind::RegMask)
    return true;

  if (MemOp.Op != WDC65816MemOp::Kind::Reg)
    return false;

  // Only regular address / data registers are allowed to be used
  // in register masks.
  return getRegisterIndex(MemOp.OuterReg) < 16;
}

void WDC65816Operand::addMoveMaskOperands(MCInst &Inst, unsigned N) const {
  assert(isMoveMask() && "wrong operand kind");
  assert((N == 1) && "can only handle one immediate operand");

  uint16_t MoveMask = MemOp.RegMask;
  if (MemOp.Op == WDC65816MemOp::Kind::Reg)
    MoveMask = 1 << getRegisterIndex(MemOp.OuterReg);

  Inst.addOperand(MCOperand::createImm(MoveMask));
}

// Addr
bool WDC65816Operand::isAddr() const {
  return isMemOp() && MemOp.Op == WDC65816MemOp::Kind::Addr;
}
// TODO: Maybe we can also store the size of OuterDisp
// in Size?
template <unsigned N> bool WDC65816Operand::isAddrN() const {
  if (isAddr()) {
    int64_t Res;
    if (MemOp.OuterDisp->evaluateAsAbsolute(Res))
      return isInt<N>(Res);
    return true;
  }
  return false;
}
void WDC65816Operand::addAddrOperands(MCInst &Inst, unsigned N) const {
  WDC65816Operand::addExpr(Inst, MemOp.OuterDisp);
}

// ARI
bool WDC65816Operand::isARI() const {
  return isMemOp() && MemOp.Op == WDC65816MemOp::Kind::RegIndirect &&
         WDC65816::AR32RegClass.contains(MemOp.OuterReg);
}
void WDC65816Operand::addARIOperands(MCInst &Inst, unsigned N) const {
  Inst.addOperand(MCOperand::createReg(MemOp.OuterReg));
}

// ARID
bool WDC65816Operand::isARID() const {
  return isMemOp() && MemOp.Op == WDC65816MemOp::Kind::RegIndirectDisplacement &&
         WDC65816::AR32RegClass.contains(MemOp.OuterReg);
}
void WDC65816Operand::addARIDOperands(MCInst &Inst, unsigned N) const {
  WDC65816Operand::addExpr(Inst, MemOp.OuterDisp);
  Inst.addOperand(MCOperand::createReg(MemOp.OuterReg));
}

// ARII
bool WDC65816Operand::isARII() const {
  return isMemOp() &&
         MemOp.Op == WDC65816MemOp::Kind::RegIndirectDisplacementIndex &&
         WDC65816::AR32RegClass.contains(MemOp.OuterReg);
}
void WDC65816Operand::addARIIOperands(MCInst &Inst, unsigned N) const {
  WDC65816Operand::addExpr(Inst, MemOp.OuterDisp);
  Inst.addOperand(MCOperand::createReg(MemOp.OuterReg));
  Inst.addOperand(MCOperand::createReg(MemOp.InnerReg));
}

// ARIPD
bool WDC65816Operand::isARIPD() const {
  return isMemOp() && MemOp.Op == WDC65816MemOp::Kind::RegPreDecrement &&
         WDC65816::AR32RegClass.contains(MemOp.OuterReg);
}
void WDC65816Operand::addARIPDOperands(MCInst &Inst, unsigned N) const {
  Inst.addOperand(MCOperand::createReg(MemOp.OuterReg));
}

// ARIPI
bool WDC65816Operand::isARIPI() const {
  return isMemOp() && MemOp.Op == WDC65816MemOp::Kind::RegPostIncrement &&
         WDC65816::AR32RegClass.contains(MemOp.OuterReg);
}
void WDC65816Operand::addARIPIOperands(MCInst &Inst, unsigned N) const {
  Inst.addOperand(MCOperand::createReg(MemOp.OuterReg));
}

// PCD
bool WDC65816Operand::isPCD() const {
  return isMemOp() && MemOp.Op == WDC65816MemOp::Kind::RegIndirectDisplacement &&
         MemOp.OuterReg == WDC65816::PC;
}
void WDC65816Operand::addPCDOperands(MCInst &Inst, unsigned N) const {
  WDC65816Operand::addExpr(Inst, MemOp.OuterDisp);
}

// PCI
bool WDC65816Operand::isPCI() const {
  return isMemOp() &&
         MemOp.Op == WDC65816MemOp::Kind::RegIndirectDisplacementIndex &&
         MemOp.OuterReg == WDC65816::PC;
}
void WDC65816Operand::addPCIOperands(MCInst &Inst, unsigned N) const {
  WDC65816Operand::addExpr(Inst, MemOp.OuterDisp);
  Inst.addOperand(MCOperand::createReg(MemOp.InnerReg));
}

static inline bool checkRegisterClass(unsigned RegNo, bool Data, bool Address,
                                      bool SP, bool FPDR = false,
                                      bool FPCR = false) {
  switch (RegNo) {
  case WDC65816::A0:
  case WDC65816::A1:
  case WDC65816::A2:
  case WDC65816::A3:
  case WDC65816::A4:
  case WDC65816::A5:
  case WDC65816::A6:
    return Address;

  case WDC65816::SP:
    return SP;

  case WDC65816::D0:
  case WDC65816::D1:
  case WDC65816::D2:
  case WDC65816::D3:
  case WDC65816::D4:
  case WDC65816::D5:
  case WDC65816::D6:
  case WDC65816::D7:
    return Data;

  case WDC65816::SR:
  case WDC65816::CCR:
    return false;

  case WDC65816::FP0:
  case WDC65816::FP1:
  case WDC65816::FP2:
  case WDC65816::FP3:
  case WDC65816::FP4:
  case WDC65816::FP5:
  case WDC65816::FP6:
  case WDC65816::FP7:
    return FPDR;

  case WDC65816::FPC:
  case WDC65816::FPS:
  case WDC65816::FPIAR:
    return FPCR;

  default:
    llvm_unreachable("unexpected register type");
    return false;
  }
}

bool WDC65816Operand::isAReg() const {
  return isReg() && checkRegisterClass(getReg(),
                                       /*Data=*/false,
                                       /*Address=*/true, /*SP=*/true);
}

bool WDC65816Operand::isDReg() const {
  return isReg() && checkRegisterClass(getReg(),
                                       /*Data=*/true,
                                       /*Address=*/false, /*SP=*/false);
}

bool WDC65816Operand::isFPDReg() const {
  return isReg() && checkRegisterClass(getReg(),
                                       /*Data=*/false,
                                       /*Address=*/false, /*SP=*/false,
                                       /*FPDR=*/true);
}

bool WDC65816Operand::isFPCReg() const {
  return isReg() && checkRegisterClass(getReg(),
                                       /*Data=*/false,
                                       /*Address=*/false, /*SP=*/false,
                                       /*FPDR=*/false, /*FPCR=*/true);
}

unsigned WDC65816AsmParser::validateTargetOperandClass(MCParsedAsmOperand &Op,
                                                   unsigned Kind) {
  WDC65816Operand &Operand = (WDC65816Operand &)Op;

  switch (Kind) {
  case MCK_XR16:
  case MCK_SPILL:
    if (Operand.isReg() &&
        checkRegisterClass(Operand.getReg(), true, true, true)) {
      return Match_Success;
    }
    break;

  case MCK_AR16:
  case MCK_AR32:
    if (Operand.isReg() &&
        checkRegisterClass(Operand.getReg(), false, true, true)) {
      return Match_Success;
    }
    break;

  case MCK_AR32_NOSP:
    if (Operand.isReg() &&
        checkRegisterClass(Operand.getReg(), false, true, false)) {
      return Match_Success;
    }
    break;

  case MCK_DR8:
  case MCK_DR16:
  case MCK_DR32:
    if (Operand.isReg() &&
        checkRegisterClass(Operand.getReg(), true, false, false)) {
      return Match_Success;
    }
    break;

  case MCK_AR16_TC:
    if (Operand.isReg() &&
        ((Operand.getReg() == WDC65816::A0) || (Operand.getReg() == WDC65816::A1))) {
      return Match_Success;
    }
    break;

  case MCK_DR16_TC:
    if (Operand.isReg() &&
        ((Operand.getReg() == WDC65816::D0) || (Operand.getReg() == WDC65816::D1))) {
      return Match_Success;
    }
    break;

  case MCK_XR16_TC:
    if (Operand.isReg() &&
        ((Operand.getReg() == WDC65816::D0) || (Operand.getReg() == WDC65816::D1) ||
         (Operand.getReg() == WDC65816::A0) || (Operand.getReg() == WDC65816::A1))) {
      return Match_Success;
    }
    break;
  }

  return Match_InvalidOperand;
}

bool WDC65816AsmParser::parseRegisterName(MCRegister &RegNo, SMLoc Loc,
                                      StringRef RegisterName) {
  auto RegisterNameLower = RegisterName.lower();

  // CCR register
  if (RegisterNameLower == "ccr") {
    RegNo = WDC65816::CCR;
    return true;
  }

  // Parse simple general-purpose registers.
  if (RegisterNameLower.size() == 2) {

    switch (RegisterNameLower[0]) {
    case 'd':
    case 'a': {
      if (isdigit(RegisterNameLower[1])) {
        unsigned IndexOffset = (RegisterNameLower[0] == 'a') ? 8 : 0;
        unsigned RegIndex = (unsigned)(RegisterNameLower[1] - '0');
        if (RegIndex < 8) {
          RegNo = getRegisterByIndex(IndexOffset + RegIndex);
          return true;
        }
      }
      break;
    }

    case 's':
      if (RegisterNameLower[1] == 'p') {
        RegNo = WDC65816::SP;
        return true;
      } else if (RegisterNameLower[1] == 'r') {
        RegNo = WDC65816::SR;
        return true;
      }
      break;

    case 'p':
      if (RegisterNameLower[1] == 'c') {
        RegNo = WDC65816::PC;
        return true;
      }
      break;
    }
  } else if (StringRef(RegisterNameLower).starts_with("fp") &&
             RegisterNameLower.size() > 2) {
    auto RegIndex = unsigned(RegisterNameLower[2] - '0');
    if (RegIndex < 8 && RegisterNameLower.size() == 3) {
      // Floating point data register.
      RegNo = getRegisterByIndex(16 + RegIndex);
      return true;
    } else {
      // Floating point control register.
      RegNo = StringSwitch<unsigned>(RegisterNameLower)
                  .Cases("fpc", "fpcr", WDC65816::FPC)
                  .Cases("fps", "fpsr", WDC65816::FPS)
                  .Cases("fpi", "fpiar", WDC65816::FPIAR)
                  .Default(WDC65816::NoRegister);
      assert(RegNo != WDC65816::NoRegister &&
             "Unrecognized FP control register name");
      return true;
    }
  }

  return false;
}

ParseStatus WDC65816AsmParser::parseRegister(MCRegister &RegNo) {
  bool HasPercent = false;
  AsmToken PercentToken;

  LLVM_DEBUG(dbgs() << "parseRegister "; getTok().dump(dbgs()); dbgs() << "\n");

  if (getTok().is(AsmToken::Percent)) {
    HasPercent = true;
    PercentToken = Lex();
  } else if (!RegisterPrefixOptional.getValue()) {
    return ParseStatus::NoMatch;
  }

  if (!Parser.getTok().is(AsmToken::Identifier)) {
    if (HasPercent) {
      getLexer().UnLex(PercentToken);
    }
    return ParseStatus::NoMatch;
  }

  auto RegisterName = Parser.getTok().getString();
  if (!parseRegisterName(RegNo, Parser.getLexer().getLoc(), RegisterName)) {
    if (HasPercent) {
      getLexer().UnLex(PercentToken);
    }
    return ParseStatus::NoMatch;
  }

  Parser.Lex();
  return ParseStatus::Success;
}

bool WDC65816AsmParser::parseRegister(MCRegister &Reg, SMLoc &StartLoc,
                                  SMLoc &EndLoc) {
  ParseStatus Result = tryParseRegister(Reg, StartLoc, EndLoc);
  if (!Result.isSuccess())
    return Error(StartLoc, "expected register");

  return false;
}

ParseStatus WDC65816AsmParser::tryParseRegister(MCRegister &Reg, SMLoc &StartLoc,
                                            SMLoc &EndLoc) {
  StartLoc = getLexer().getLoc();
  ParseStatus Result = parseRegister(Reg);
  EndLoc = getLexer().getLoc();
  return Result;
}

bool WDC65816AsmParser::isExpr() {
  switch (Parser.getTok().getKind()) {
  case AsmToken::Identifier:
  case AsmToken::Integer:
    return true;
  case AsmToken::Minus:
    return getLexer().peekTok().getKind() == AsmToken::Integer;

  default:
    return false;
  }
}

ParseStatus WDC65816AsmParser::parseImm(OperandVector &Operands) {
  if (getLexer().isNot(AsmToken::Hash))
    return ParseStatus::NoMatch;
  SMLoc Start = getLexer().getLoc();
  Parser.Lex();

  SMLoc End;
  const MCExpr *Expr;

  if (getParser().parseExpression(Expr, End))
    return ParseStatus::Failure;

  Operands.push_back(WDC65816Operand::createImm(Expr, Start, End));
  return ParseStatus::Success;
}

ParseStatus WDC65816AsmParser::parseMemOp(OperandVector &Operands) {
  SMLoc Start = getLexer().getLoc();
  bool IsPD = false;
  WDC65816MemOp MemOp;

  // Check for a plain register or register mask.
  ParseStatus Result = parseRegOrMoveMask(Operands);
  if (!Result.isNoMatch())
    return Result;

  // Check for pre-decrement & outer displacement.
  bool HasDisplacement = false;
  if (getLexer().is(AsmToken::Minus)) {
    IsPD = true;
    Parser.Lex();
  } else if (isExpr()) {
    if (Parser.parseExpression(MemOp.OuterDisp))
      return ParseStatus::Failure;
    HasDisplacement = true;
  }

  if (getLexer().isNot(AsmToken::LParen)) {
    if (HasDisplacement) {
      MemOp.Op = WDC65816MemOp::Kind::Addr;
      Operands.push_back(
          WDC65816Operand::createMemOp(MemOp, Start, getLexer().getLoc()));
      return ParseStatus::Success;
    }
    if (IsPD)
      return Error(getLexer().getLoc(), "expected (");

    return ParseStatus::NoMatch;
  }
  Parser.Lex();

  // Check for constant dereference & MIT-style displacement
  if (!HasDisplacement && isExpr()) {
    if (Parser.parseExpression(MemOp.OuterDisp))
      return ParseStatus::Failure;
    HasDisplacement = true;

    // If we're not followed by a comma, we're a constant dereference.
    if (getLexer().isNot(AsmToken::Comma)) {
      MemOp.Op = WDC65816MemOp::Kind::Addr;
      Operands.push_back(
          WDC65816Operand::createMemOp(MemOp, Start, getLexer().getLoc()));
      return ParseStatus::Success;
    }

    Parser.Lex();
  }

  Result = parseRegister(MemOp.OuterReg);
  if (Result.isFailure())
    return ParseStatus::Failure;

  if (!Result.isSuccess())
    return Error(getLexer().getLoc(), "expected register");

  // Check for Index.
  bool HasIndex = false;
  if (Parser.getTok().is(AsmToken::Comma)) {
    Parser.Lex();

    Result = parseRegister(MemOp.InnerReg);
    if (Result.isFailure())
      return Result;

    if (Result.isNoMatch())
      return Error(getLexer().getLoc(), "expected register");

    // TODO: parse size, scale and inner displacement.
    MemOp.Size = 4;
    MemOp.Scale = 1;
    MemOp.InnerDisp = MCConstantExpr::create(0, Parser.getContext(), true, 4);
    HasIndex = true;
  }

  if (Parser.getTok().isNot(AsmToken::RParen))
    return Error(getLexer().getLoc(), "expected )");
  Parser.Lex();

  bool IsPI = false;
  if (!IsPD && Parser.getTok().is(AsmToken::Plus)) {
    Parser.Lex();
    IsPI = true;
  }

  SMLoc End = getLexer().getLoc();

  unsigned OpCount = IsPD + IsPI + (HasIndex || HasDisplacement);
  if (OpCount > 1)
    return Error(Start, "only one of post-increment, pre-decrement or "
                        "displacement can be used");

  if (IsPD) {
    MemOp.Op = WDC65816MemOp::Kind::RegPreDecrement;
  } else if (IsPI) {
    MemOp.Op = WDC65816MemOp::Kind::RegPostIncrement;
  } else if (HasIndex) {
    MemOp.Op = WDC65816MemOp::Kind::RegIndirectDisplacementIndex;
  } else if (HasDisplacement) {
    MemOp.Op = WDC65816MemOp::Kind::RegIndirectDisplacement;
  } else {
    MemOp.Op = WDC65816MemOp::Kind::RegIndirect;
  }

  Operands.push_back(WDC65816Operand::createMemOp(MemOp, Start, End));
  return ParseStatus::Success;
}

ParseStatus WDC65816AsmParser::parseRegOrMoveMask(OperandVector &Operands) {
  SMLoc Start = getLexer().getLoc();
  WDC65816MemOp MemOp(WDC65816MemOp::Kind::RegMask);
  MemOp.RegMask = 0;

  for (;;) {
    bool IsFirstRegister =
        (MemOp.Op == WDC65816MemOp::Kind::RegMask) && (MemOp.RegMask == 0);

    MCRegister FirstRegister;
    ParseStatus Result = parseRegister(FirstRegister);
    if (IsFirstRegister && Result.isNoMatch())
      return ParseStatus::NoMatch;
    if (!Result.isSuccess())
      return Error(getLexer().getLoc(), "expected start register");

    MCRegister LastRegister = FirstRegister;
    if (parseOptionalToken(AsmToken::Minus)) {
      Result = parseRegister(LastRegister);
      if (!Result.isSuccess())
        return Error(getLexer().getLoc(), "expected end register");
    }

    unsigned FirstRegisterIndex = getRegisterIndex(FirstRegister);
    unsigned LastRegisterIndex = getRegisterIndex(LastRegister);

    uint16_t NumNewBits = LastRegisterIndex - FirstRegisterIndex + 1;
    uint16_t NewMaskBits = ((1 << NumNewBits) - 1) << FirstRegisterIndex;

    if (IsFirstRegister && (FirstRegister == LastRegister)) {
      // First register range is a single register, simplify to just Reg
      // so that it matches more operands.
      MemOp.Op = WDC65816MemOp::Kind::Reg;
      MemOp.OuterReg = FirstRegister;
    } else {
      if (MemOp.Op == WDC65816MemOp::Kind::Reg) {
        // This is the second register being specified - expand the Reg operand
        // into a mask first.
        MemOp.Op = WDC65816MemOp::Kind::RegMask;
        MemOp.RegMask = 1 << getRegisterIndex(MemOp.OuterReg);

        if (MemOp.RegMask == 0)
          return Error(getLexer().getLoc(),
                       "special registers cannot be used in register masks");
      }

      if ((FirstRegisterIndex >= 16) || (LastRegisterIndex >= 16))
        return Error(getLexer().getLoc(),
                     "special registers cannot be used in register masks");

      if (NewMaskBits & MemOp.RegMask)
        return Error(getLexer().getLoc(), "conflicting masked registers");

      MemOp.RegMask |= NewMaskBits;
    }

    if (!parseOptionalToken(AsmToken::Slash))
      break;
  }

  Operands.push_back(
      WDC65816Operand::createMemOp(MemOp, Start, getLexer().getLoc()));
  return ParseStatus::Success;
}

void WDC65816AsmParser::eatComma() {
  if (Parser.getTok().is(AsmToken::Comma)) {
    Parser.Lex();
  }
}

bool WDC65816AsmParser::ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                                     SMLoc NameLoc, OperandVector &Operands) {
  SMLoc Start = getLexer().getLoc();
  Operands.push_back(WDC65816Operand::createToken(Name, Start, Start));

  bool First = true;
  while (Parser.getTok().isNot(AsmToken::EndOfStatement)) {
    if (!First) {
      eatComma();
    } else {
      First = false;
    }

    ParseStatus MatchResult = MatchOperandParserImpl(Operands, Name);
    if (MatchResult.isSuccess())
      continue;

    // Add custom operand formats here...
    SMLoc Loc = getLexer().getLoc();
    Parser.eatToEndOfStatement();
    return Error(Loc, "unexpected token parsing operands");
  }

  // Eat EndOfStatement.
  Parser.Lex();
  return false;
}

bool WDC65816AsmParser::invalidOperand(SMLoc const &Loc,
                                   OperandVector const &Operands,
                                   uint64_t const &ErrorInfo) {
  SMLoc ErrorLoc = Loc;
  char const *Diag = 0;

  if (ErrorInfo != ~0U) {
    if (ErrorInfo >= Operands.size()) {
      Diag = "too few operands for instruction.";
    } else {
      auto const &Op = (WDC65816Operand const &)*Operands[ErrorInfo];
      if (Op.getStartLoc() != SMLoc()) {
        ErrorLoc = Op.getStartLoc();
      }
    }
  }

  if (!Diag) {
    Diag = "invalid operand for instruction";
  }

  return Error(ErrorLoc, Diag);
}

bool WDC65816AsmParser::missingFeature(llvm::SMLoc const &Loc,
                                   uint64_t const &ErrorInfo) {
  return Error(Loc, "instruction requires a CPU feature not currently enabled");
}

bool WDC65816AsmParser::emit(MCInst &Inst, SMLoc const &Loc,
                         MCStreamer &Out) const {
  Inst.setLoc(Loc);
  Out.emitInstruction(Inst, STI);

  return false;
}

bool WDC65816AsmParser::MatchAndEmitInstruction(SMLoc Loc, unsigned &Opcode,
                                            OperandVector &Operands,
                                            MCStreamer &Out,
                                            uint64_t &ErrorInfo,
                                            bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);

  switch (MatchResult) {
  case Match_Success:
    return emit(Inst, Loc, Out);
  case Match_MissingFeature:
    return missingFeature(Loc, ErrorInfo);
  case Match_InvalidOperand:
    return invalidOperand(Loc, Operands, ErrorInfo);
  case Match_MnemonicFail:
    return Error(Loc, "invalid instruction");
  default:
    return true;
  }
}

void WDC65816Operand::print(raw_ostream &OS) const {
  switch (Kind) {
  case KindTy::Invalid:
    OS << "invalid";
    break;

  case KindTy::Token:
    OS << "token '" << Token << "'";
    break;

  case KindTy::Imm: {
    int64_t Value;
    Expr->evaluateAsAbsolute(Value);
    OS << "immediate " << Value;
    break;
  }

  case KindTy::MemOp:
    MemOp.print(OS);
    break;
  }
}
