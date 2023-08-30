pub mod cpu;
pub mod memory_bus;
pub mod opcodes;

use self::cpu::Cpu6502;

fn main() {
    let mut cpu = Cpu6502::new();

    let program = vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00];

    cpu.load_and_run(program)
}
