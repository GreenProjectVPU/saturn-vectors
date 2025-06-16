package saturn.exu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import saturn.common._
import saturn.insns.{VectorInstruction}

abstract class FunctionalUnitIO(implicit p: Parameters) extends CoreBundle()(p) with HasVectorParams {
  val stall = Output(Bool())
  val iss = new Bundle {
    val valid = Input(Bool())
    val op = Input(new ExecuteMicroOpWithData(1))
  }

  val scalar_write = Decoupled(new ScalarWrite)
  val set_vxsat = Output(Bool())
  val set_fflags = Output(Valid(UInt(5.W)))
}

class PipelinedFunctionalUnitIO(depth: Int)(implicit p: Parameters) extends FunctionalUnitIO {
  require(depth <= 8)
  val write = Valid(new VectorWrite(dLen))
  val pipe = Input(Vec(depth, Valid(new ExecuteMicroOpWithData(1))))
}

class IterativeFunctionalUnitIO(implicit p: Parameters) extends FunctionalUnitIO {
  val write = Decoupled(new VectorWrite(dLen))
  val hazard = Output(Valid(new PipeHazard(10)))
  val acc = Output(Bool())
  val tail = Output(Bool())
  val valid = Output(Bool())
  val op = Output(new ExecuteMicroOpWithData(1))
  val last = Output(Bool())

  val busy = Output(Bool())
}

trait FunctionalUnitFactory {
  def insns: Seq[VectorInstruction]
  def generate(implicit p: Parameters): FunctionalUnit
}

abstract class FunctionalUnit(implicit p: Parameters) extends CoreModule()(p) with HasVectorParams {
  val io: FunctionalUnitIO
}

abstract class PipelinedFunctionalUnit(val depth: Int)(implicit p: Parameters) extends FunctionalUnit()(p) {
  val io = IO(new PipelinedFunctionalUnitIO(depth))

  // the last stage is always invalid if the pipeline is more than 1 stage long.
  val lastStage = if (depth > 1) {
    RegNext(io.pipe(depth - 2))
  } else {
    io.pipe(depth - 1)
  }

  io.scalar_write.bits.uopId := lastStage.bits.uopId
  io.write.bits.uopId := lastStage.bits.uopId

  require (depth > 0)

  def narrow2_expand(bits: Seq[UInt], eew: UInt, upper: Bool, sext: Bool): Vec[UInt] = {
    val narrow_eew = (0 until 3).map { eew => Wire(Vec(dLenB >> (eew + 1), UInt((16 << eew).W))) }
    for (eew <- 0 until 3) {
      val in_vec = bits.grouped(1 << eew).map(g => VecInit(g).asUInt).toSeq
      for (i <- 0 until dLenB >> (eew + 1)) {
        val lo = Mux(upper, in_vec(i + (dLenB >> (eew + 1))), in_vec(i))
        val hi = Fill(16 << eew, lo((8 << eew)-1) && sext)
        narrow_eew(eew)(i) := Cat(hi, lo)
      }
    }
    VecInit(narrow_eew.map(_.asUInt))(eew).asTypeOf(Vec(dLenB, UInt(8.W)))
  }
}

abstract class IterativeFunctionalUnit(implicit p: Parameters) extends FunctionalUnit()(p) {
  val io = IO(new IterativeFunctionalUnitIO)

  val valid = RegInit(false.B)
  val op = Reg(new ExecuteMicroOpWithData(1))
  val last = Wire(Bool())

  io.busy := valid
  io.valid := valid
  io.op := op
  io.last := last
  io.write.bits.uopId := op.uopId

  when (io.iss.valid) {
    assert(!valid || last)
    valid := true.B
    op := io.iss.op
  } .elsewhen (last) {
    valid := false.B
  }
}
