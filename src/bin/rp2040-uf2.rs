use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

use goblin::elf::Elf;
use goblin::elf::program_header::PT_LOAD;

const UF2_MAGIC_START0: u32 = 0x0A32_4655;
const UF2_MAGIC_START1: u32 = 0x9E5D_5157;
const UF2_MAGIC_END: u32 = 0x0AB1_6F30;
const UF2_FLAG_FAMILY_ID_PRESENT: u32 = 0x0000_2000;
const RP2040_FAMILY_ID: u32 = 0xE48B_FF56;
const UF2_BLOCK_SIZE: usize = 512;
const UF2_PAYLOAD_SIZE: usize = 256;

#[derive(Debug)]
struct Block {
    address: u32,
    payload: Vec<u8>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let input = PathBuf::from(
        args.next()
            .ok_or_else(|| "usage: rp2040-uf2 <input.elf> [output.uf2]")?,
    );
    let output = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| input.with_extension("uf2"));

    let elf_bytes = fs::read(&input)?;
    let elf = Elf::parse(&elf_bytes)?;

    let mut blocks = collect_blocks(&elf_bytes, &elf);
    if blocks.is_empty() {
        return Err("no loadable segments found in ELF".into());
    }

    blocks.sort_by_key(|b| b.address);
    let total_blocks = blocks.len() as u32;

    let mut uf2_bytes = Vec::with_capacity(blocks.len() * UF2_BLOCK_SIZE);

    for (index, block) in blocks.iter().enumerate() {
        uf2_bytes.extend_from_slice(&encode_block(block, index as u32, total_blocks));
    }

    fs::write(&output, uf2_bytes)?;

    println!(
        "Wrote {} blocks ({} bytes payload) to {}",
        total_blocks,
        total_blocks as usize * UF2_PAYLOAD_SIZE,
        output.display()
    );

    Ok(())
}

fn collect_blocks(elf_bytes: &[u8], elf: &Elf<'_>) -> Vec<Block> {
    let mut blocks = Vec::new();

    for header in &elf.program_headers {
        if header.p_type != PT_LOAD {
            continue;
        }
        if header.p_filesz == 0 {
            continue;
        }

        let start = header.p_offset as usize;
        let end = start + header.p_filesz as usize;
        if end > elf_bytes.len() || start > elf_bytes.len() {
            continue;
        }

        let mut address = header.p_paddr as u32;
        let data = &elf_bytes[start..end];

        for chunk in data.chunks(UF2_PAYLOAD_SIZE) {
            let mut payload = vec![0u8; UF2_PAYLOAD_SIZE];
            payload[..chunk.len()].copy_from_slice(chunk);
            blocks.push(Block { address, payload });
            address += chunk.len() as u32;
        }
    }

    blocks
}

fn encode_block(block: &Block, index: u32, total: u32) -> [u8; UF2_BLOCK_SIZE] {
    let mut buf = [0u8; UF2_BLOCK_SIZE];

    buf[0..4].copy_from_slice(&UF2_MAGIC_START0.to_le_bytes());
    buf[4..8].copy_from_slice(&UF2_MAGIC_START1.to_le_bytes());
    buf[8..12].copy_from_slice(&UF2_FLAG_FAMILY_ID_PRESENT.to_le_bytes());
    buf[12..16].copy_from_slice(&block.address.to_le_bytes());
    buf[16..20].copy_from_slice(&(UF2_PAYLOAD_SIZE as u32).to_le_bytes());
    buf[20..24].copy_from_slice(&index.to_le_bytes());
    buf[24..28].copy_from_slice(&total.to_le_bytes());
    buf[28..32].copy_from_slice(&RP2040_FAMILY_ID.to_le_bytes());
    buf[32..32 + UF2_PAYLOAD_SIZE].copy_from_slice(&block.payload);
    buf[UF2_BLOCK_SIZE - 4..UF2_BLOCK_SIZE].copy_from_slice(&UF2_MAGIC_END.to_le_bytes());

    buf
}
