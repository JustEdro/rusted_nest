#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect_X,
    Indirect_Y,
    NoneAddressing,
    Relative, // branches 
}

pub struct OpCode {
    pub opcode: u8,
    pub menmonic: &'static str,
    pub length: u16,
    pub cycles: u8,
    pub address_mode: AddressingMode,
    pub page_cross_penalty: u8,
}

impl OpCode {
    pub const fn new(
        opcode: u8,
        mnemonic: &'static str,
        length: u16,
        cycles: u8,
        address_mode: AddressingMode,
        page_cross_penalty: u8,
    ) -> Self {
        OpCode {
            opcode: opcode,
            menmonic: mnemonic,
            length: length,
            cycles: cycles,
            address_mode: address_mode,
            page_cross_penalty: page_cross_penalty,
        }
    }
}


const OPCODE_COUNT: usize = 46;

#[rustfmt::skip]
const CPU_OPS_CODES_LIST: [OpCode; OPCODE_COUNT] = [
    OpCode::new( 0xAA, "TAX", 1, 2, AddressingMode::NoneAddressing, 0),
    OpCode::new( 0xE8, "INX", 1, 2, AddressingMode::NoneAddressing, 0),
    
    OpCode::new( 0x69, "ADC", 2, 2, AddressingMode::Immediate, 0),
    OpCode::new( 0x65, "ADC", 2, 3, AddressingMode::ZeroPage, 0),
    OpCode::new( 0x75, "ADC", 2, 4, AddressingMode::ZeroPage_X, 0),
    OpCode::new( 0x6D, "ADC", 3, 4, AddressingMode::Absolute, 0),
    OpCode::new( 0x7D, "ADC", 3, 4, AddressingMode::Absolute_X, 1),
    OpCode::new( 0x79, "ADC", 3, 4, AddressingMode::Absolute_Y, 1),
    OpCode::new( 0x61, "ADC", 2, 6, AddressingMode::Indirect_X, 0),
    OpCode::new( 0x71, "ADC", 2, 5, AddressingMode::Indirect_Y, 1),

    OpCode::new( 0x29, "AND", 2, 2, AddressingMode::Immediate, 0),
    OpCode::new( 0x25, "AND", 2, 3, AddressingMode::ZeroPage, 0),
    OpCode::new( 0x35, "AND", 2, 4, AddressingMode::ZeroPage_X, 0),
    OpCode::new( 0x2D, "AND", 3, 4, AddressingMode::Absolute, 0),
    OpCode::new( 0x3D, "AND", 3, 4, AddressingMode::Absolute_X, 1),
    OpCode::new( 0x39, "AND", 3, 4, AddressingMode::Absolute_Y, 1),
    OpCode::new( 0x21, "AND", 2, 6, AddressingMode::Indirect_X, 0),
    OpCode::new( 0x31, "AND", 2, 5, AddressingMode::Indirect_Y, 1),

    OpCode::new( 0x0A, "ASL", 1, 2, AddressingMode::NoneAddressing, 0),
    OpCode::new( 0x06, "ASL", 2, 5, AddressingMode::ZeroPage, 0),
    OpCode::new( 0x16, "ASL", 2, 6, AddressingMode::ZeroPage_X, 0),
    OpCode::new( 0x0E, "ASL", 3, 6, AddressingMode::Absolute, 0),
    OpCode::new( 0x1E, "ASL", 3, 7, AddressingMode::Absolute_X, 0),

    // branches
    OpCode::new( 0x90, "BCC", 2, 2, AddressingMode::Relative, 2),
    OpCode::new( 0xB0, "BCS", 2, 2, AddressingMode::Relative, 2),
    OpCode::new( 0xF0, "BEQ", 2, 2, AddressingMode::Relative, 2),
    OpCode::new( 0x30, "BMI", 2, 2, AddressingMode::Relative, 2),
    OpCode::new( 0xD0, "BNE", 2, 2, AddressingMode::Relative, 2),
    OpCode::new( 0x10, "BPL", 2, 2, AddressingMode::Relative, 2),
    OpCode::new( 0x50, "BVC", 2, 2, AddressingMode::Relative, 2),
    OpCode::new( 0x70, "BVS", 2, 2, AddressingMode::Relative, 2),
    
    OpCode::new( 0x24, "BIT", 2, 3, AddressingMode::ZeroPage, 0),
    OpCode::new( 0x2C, "BIT", 3, 4, AddressingMode::Absolute, 0),
    
    OpCode::new( 0x00, "BRK", 1, 7, AddressingMode::NoneAddressing, 0),
    
    // flag cleaner
    OpCode::new( 0x18, "CLC", 1, 2, AddressingMode::NoneAddressing, 0),
    OpCode::new( 0xD8, "CLD", 1, 2, AddressingMode::NoneAddressing, 0),
    OpCode::new( 0x58, "CLI", 1, 2, AddressingMode::NoneAddressing, 0),
    OpCode::new( 0xB8, "CLV", 1, 2, AddressingMode::NoneAddressing, 0),

    // TODO

    OpCode::new( 0xA9, "LDA", 2, 2, AddressingMode::Immediate, 0),
    OpCode::new( 0xA5, "LDA", 2, 3, AddressingMode::ZeroPage, 0),
    OpCode::new( 0xB5, "LDA", 2, 4, AddressingMode::ZeroPage_X, 0),
    OpCode::new( 0xAD, "LDA", 3, 4, AddressingMode::Absolute, 0),
    OpCode::new( 0xBD, "LDA", 3, 4, AddressingMode::Absolute_X, 1),
    OpCode::new( 0xB9, "LDA", 3, 4, AddressingMode::Absolute_Y, 1),
    OpCode::new( 0xA1, "LDA", 2, 6, AddressingMode::Indirect_X, 0),
    OpCode::new( 0xB1, "LDA", 2, 5, AddressingMode::Indirect_Y, 1),

];

// reindex the array based on the opcode
pub const CPU_OPS_CODES: [&OpCode; 0xFF] = {
    let mut indexed = [&CPU_OPS_CODES_LIST[0]; 0xFF];
    let mut flags = [false; 0xFF];
    let mut i = 0;
    while i < OPCODE_COUNT {
        let opcode = CPU_OPS_CODES_LIST[i].opcode;
        // check for the opcode duplication
        if flags[opcode as usize] {
            i += 1000; // simulate out of bounds
        }
        flags[opcode as usize] = true;
        // IF OUT OF BOUNDS - there is a duplicated opcode
        indexed[opcode as usize] = &CPU_OPS_CODES_LIST[i];
        i += 1;
    }
    indexed
};

