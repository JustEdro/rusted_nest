#[derive(Debug, PartialEq)]
pub enum Mirroring {
    Vertical,
    Horiznotal,
    FourScreen,
}

pub struct Rom {
    pub prg_rom: Vec<u8>,
    pub chr_rom: Vec<u8>,
    pub mapper: u8,
    pub screen_mirroring: Mirroring,
}

const NES_TAG: &[u8] = &[b'N', b'E', b'S', 0x1A];
const PRG_ROM_PAGE_SIZE:usize = 16 * 1024;
const CHR_ROM_PAGE_SIZE:usize = 8 * 1024;

impl Rom {
    pub fn new(raw: &Vec<u8>) -> Result<Rom, String> {
        if &raw[0..4] != NES_TAG {
            return Err("File is not in iNES file format".to_string());
        }

        let mapper = (raw[7] & 0b1111_0000) | (raw[6] >> 4);

        let ines_ver = (raw[7] >> 2) & 0b11;
        if ines_ver != 0 {
            return Err("NES2.0 format is not supported".to_string());
        }

        let four_screen = raw[6] & 0b1000 != 0;
        let vertical_mirroring = raw[6] & 0b1 != 0;
        let screen_mirroring = match (four_screen, vertical_mirroring) {
            (true, _) => Mirroring::FourScreen,
            (false, true) => Mirroring::Vertical,
            (false, false) => Mirroring::Horiznotal,
        };

        let prg_rom_size = raw[4] as usize * PRG_ROM_PAGE_SIZE;
        let chr_rom_size = raw[5] as usize * CHR_ROM_PAGE_SIZE;

        let skip_trainer = raw[6] & 0b100 != 0;

        let prg_rom_start = 16 + if skip_trainer { 512 } else { 0 };
        let chr_rom_start = prg_rom_start + prg_rom_size;

        Ok(Rom {
            prg_rom: raw[prg_rom_start..(prg_rom_start + prg_rom_size)].to_vec(),
            chr_rom: raw[chr_rom_start..(chr_rom_start + chr_rom_size)].to_vec(),
            mapper: mapper,
            screen_mirroring: screen_mirroring,
        })
    }

    pub fn empty() -> Result<Rom, String> {
        Ok(Rom {
            prg_rom: vec![0; PRG_ROM_PAGE_SIZE],
            chr_rom: vec![],
            mapper: 0,
            screen_mirroring: Mirroring::Vertical,
        })
    }

    pub fn test(raw: &Vec<u8>) -> Result<Rom, String> {
        let mut padded = raw.to_vec();
        padded.resize(PRG_ROM_PAGE_SIZE, 0);
        //padded[0xFFFC - 0x8000 + 0] = 0x00; // cpu resets PC to here (u16)
        padded[(0xFFFC - 0x8000 + 1) % 0x4000] = 0x80; 
        Ok(Rom {
            prg_rom: padded,
            chr_rom: vec!(),
            mapper: 0,
            screen_mirroring: Mirroring::Vertical,
        })
    }
}
