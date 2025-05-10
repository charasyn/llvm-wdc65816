//==-- WDC65816RegisterInfo.td - WDC65816 register definitions ------*- tablegen -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file describes the WDC65816 Register file, defining the registers
/// aliases between the registers, and the register classes built out of the
/// registers.
///
//===----------------------------------------------------------------------===//

class WDC65816Reg<string N, bits<16> ENC,
            list<Register> SUBREGS = [], list<SubRegIndex> SUBIDX,
            list<int> DWREGS = [], list<string> ALTNAMES = []>
    : Register<N, ALTNAMES>, DwarfRegNum<DWREGS> {
  let Namespace     = "WDC65816";
  let HWEncoding    = ENC;
  let SubRegs       = SUBREGS;
  let SubRegIndices = SUBIDX;
}

// Subregister indices.
let Namespace = "WDC65816" in {
  def SubRegIndex8Lo  : SubRegIndex<8, 0>;
  def MxSubRegIndex16Lo : SubRegIndex<16, 0>;
}


multiclass WDC65816Register<int INDEX, string REG_NAME, list<string> ALTNAMES = []> {
  def NAME : MxReg<REG_NAME, INDEX, [], [], [INDEX], ALTNAMES>;
}

multiclass WDC65816Register<int INDEX, string REG_NAME, list<string> ALTNAMES = []> {
  def "W"#NAME
    : MxReg<REG_NAME, INDEX, [], [], [!add(8,INDEX)], ALTNAMES>;
  def NAME
    : MxReg<REG_NAME, INDEX,
            [!cast<Register>("W"#NAME)], [MxSubRegIndex16Lo],
            [!add(8,INDEX)], ALTNAMES>;
}

defm A : WDC65816Register<0, "A">;
defm X : WDC65816Register<1, "X">;
defm Y : WDC65816Register<2, "Y">;

// Pseudo Registers
class WDC65816PseudoReg<string N, list<Register> SUBREGS = [], list<SubRegIndex> SUBIDX = []>
    : WDC65816Reg<N, 0, SUBREGS, SUBIDX>;

def PC : MxPseudoReg<"PC">;
def SP : MxPseudoReg<"SP">;

//===----------------------------------------------------------------------===//
// Register Classes
//===----------------------------------------------------------------------===//

// Alignment is always 8
class MxRegClass<list<ValueType> regTypes, dag regList>
    : RegisterClass<"WDC65816", regTypes, 8, regList>;

// Raw Register Classes
def AC : MxRegClass<[i16], (add "A")>;
def XC : MxRegClass<[i16], (add "X")>;
def YC : MxRegClass<[i16], (add "Y")>;

// Index Register Classes
def XR16 : MxRegClass<[i16], 16, (add XC, YC)>;

def SPC  : MxRegClass<[i32], 32, (add SP)>;

let isAllocatable = 0 in {
  def PCC  : MxRegClass<[i32], 32, (add PC)>;
}
