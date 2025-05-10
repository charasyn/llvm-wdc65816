//===-- WDC65816.td - Motorola 680x0 target definitions --------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This is a target description file for the Motorola 680x0 family, referred
/// to here as the "WDC65816" architecture.
///
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// WDC65816 Subtarget features
//===----------------------------------------------------------------------===//

def FeatureISA00
  : SubtargetFeature<"isa-68000", "SubtargetKind", "M00",
                     "Is M68000 ISA supported">;

def FeatureISA10
  : SubtargetFeature<"isa-68010", "SubtargetKind", "M10",
                     "Is M68010 ISA supported",
                     [ FeatureISA00 ]>;

def FeatureISA20
  : SubtargetFeature<"isa-68020", "SubtargetKind", "M20",
                     "Is M68020 ISA supported",
                     [ FeatureISA10 ]>;

def FeatureISA30
  : SubtargetFeature<"isa-68030", "SubtargetKind", "M30",
                     "Is M68030 ISA supported",
                     [ FeatureISA20 ]>;

def FeatureISA881
  : SubtargetFeature<"isa-68881", "FPUKind", "M881",
                     "Is M68881 (FPU) ISA supported">;

def FeatureISA882
  : SubtargetFeature<"isa-68882", "FPUKind", "M882",
                     "Is M68882 (FPU) ISA supported",
                     [ FeatureISA881 ]>;

def FeatureISA40
  : SubtargetFeature<"isa-68040", "SubtargetKind", "M40",
                     "Is M68040 ISA supported",
                     [ FeatureISA30, FeatureISA882 ]>;

def FeatureISA60
  : SubtargetFeature<"isa-68060", "SubtargetKind", "M60",
                     "Is M68060 ISA supported",
                     [ FeatureISA40 ]>;

foreach i = {0-6} in
  def FeatureReserveA#i :
      SubtargetFeature<"reserve-a"#i, "UserReservedRegister[WDC65816::A"#i#"]",
                       "true", "Reserve A"#i#" register">;
foreach i = {0-7} in
  def FeatureReserveD#i :
      SubtargetFeature<"reserve-d"#i, "UserReservedRegister[WDC65816::D"#i#"]",
                       "true", "Reserve D"#i#" register">;

//===----------------------------------------------------------------------===//
// WDC65816 processors supported.
//===----------------------------------------------------------------------===//

include "WDC65816Schedule.td"

class Proc<string Name, list<SubtargetFeature> Features>
    : ProcessorModel<Name, GenericWDC65816Model, Features>;

def : Proc<"generic", [ FeatureISA00 ]>;
def : Proc<"M68000",  [ FeatureISA00 ]>;
def : Proc<"M68010",  [ FeatureISA10 ]>;
def : Proc<"M68020",  [ FeatureISA20 ]>;
def : Proc<"M68030",  [ FeatureISA30 ]>;
def : Proc<"M68040",  [ FeatureISA40 ]>;
def : Proc<"M68060",  [ FeatureISA60 ]>;

//===----------------------------------------------------------------------===//
// Register File Description
//===----------------------------------------------------------------------===//

include "WDC65816RegisterInfo.td"
include "GISel/WDC65816RegisterBanks.td"

//===----------------------------------------------------------------------===//
// Instruction Descriptions
//===----------------------------------------------------------------------===//

include "WDC65816InstrInfo.td"

def WDC65816InstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// Calling Conventions
//===----------------------------------------------------------------------===//

include "WDC65816CallingConv.td"

//===---------------------------------------------------------------------===//
// Assembly Printers
//===---------------------------------------------------------------------===//

def WDC65816AsmWriter : AsmWriter {
 string AsmWriterClassName = "InstPrinter";
 bit isMCAsmWriter = 1;
}

//===---------------------------------------------------------------------===//
// Assembly Parsers
//===---------------------------------------------------------------------===//

def WDC65816AsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
  let ShouldEmitMatchRegisterAltName = 0;
}

def WDC65816AsmParserVariant : AsmParserVariant {
  int Variant = 0;
}

//===----------------------------------------------------------------------===//
// Target
//===----------------------------------------------------------------------===//

def WDC65816 : Target {
  let InstructionSet = WDC65816InstrInfo;
  let AssemblyParsers = [WDC65816AsmParser];
  let AssemblyWriters = [WDC65816AsmWriter];
}
