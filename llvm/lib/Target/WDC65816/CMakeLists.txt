add_llvm_component_group(WDC65816)

set(LLVM_TARGET_DEFINITIONS WDC65816.td)

tablegen(LLVM WDC65816GenGlobalISel.inc       -gen-global-isel)
tablegen(LLVM WDC65816GenRegisterInfo.inc     -gen-register-info)
tablegen(LLVM WDC65816GenRegisterBank.inc     -gen-register-bank)
tablegen(LLVM WDC65816GenInstrInfo.inc        -gen-instr-info)
tablegen(LLVM WDC65816GenSubtargetInfo.inc    -gen-subtarget)
tablegen(LLVM WDC65816GenMCCodeEmitter.inc    -gen-emitter)
tablegen(LLVM WDC65816GenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM WDC65816GenDAGISel.inc          -gen-dag-isel)
tablegen(LLVM WDC65816GenCallingConv.inc      -gen-callingconv)
tablegen(LLVM WDC65816GenAsmWriter.inc        -gen-asm-writer)
tablegen(LLVM WDC65816GenAsmMatcher.inc       -gen-asm-matcher)
tablegen(LLVM WDC65816GenDisassemblerTable.inc -gen-disassembler)

add_public_tablegen_target(WDC65816CommonTableGen)

add_llvm_target(WDC65816CodeGen
  GISel/WDC65816CallLowering.cpp
  GISel/WDC65816InstructionSelector.cpp
  GISel/WDC65816LegalizerInfo.cpp
  GISel/WDC65816RegisterBankInfo.cpp
  WDC65816AsmPrinter.cpp
  WDC65816CollapseMOVEMPass.cpp
  WDC65816ExpandPseudo.cpp
  WDC65816FrameLowering.cpp
  WDC65816InstrInfo.cpp
  WDC65816ISelLowering.cpp
  WDC65816ISelDAGToDAG.cpp
  WDC65816MachineFunction.cpp
  WDC65816MCInstLower.cpp
  WDC65816RegisterInfo.cpp
  WDC65816Subtarget.cpp
  WDC65816TargetMachine.cpp
  WDC65816TargetObjectFile.cpp

  LINK_COMPONENTS
  Analysis
  AsmPrinter
  CodeGen
  CodeGenTypes
  Core
  GlobalISel
  WDC65816Desc
  WDC65816Info
  MC
  SelectionDAG
  Support
  Target

  ADD_TO_COMPONENT
  WDC65816
  )

add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
