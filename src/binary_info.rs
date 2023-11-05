use rp_binary_info::{ID_RP_PROGRAM_BUILD_ATTRIBUTE, TAG_RASPBERRY_PI};

extern "C" {
    static __bi_entries_start: rp_binary_info::entry::Addr;
    static __bi_entries_end: rp_binary_info::entry::Addr;
    static __sdata: u32;
    static __edata: u32;
    static __sidata: u32;
}

/// Picotool can find this block in our ELF file and report interesting metadata.
#[link_section = ".bi_header"]
#[used]
pub static PICOTOOL_META: rp_binary_info::Header =
    unsafe { rp_binary_info::Header::new(&__bi_entries_start, &__bi_entries_end, &MAPPING_TABLE) };

/// This tells picotool how to convert RAM addresses back into Flash addresses
static MAPPING_TABLE: [rp_binary_info::MappingTableEntry; 2] = [
    // This is the entry for .data
    rp_binary_info::MappingTableEntry {
        source_addr_start: unsafe { &__sidata },
        dest_addr_start: unsafe { &__sdata },
        dest_addr_end: unsafe { &__edata },
    },
    // This is the terminating marker
    rp_binary_info::MappingTableEntry {
        source_addr_start: core::ptr::null(),
        dest_addr_start: core::ptr::null(),
        dest_addr_end: core::ptr::null(),
    },
];

/// This is a list of references to our table entries
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [rp_binary_info::entry::Addr; 3] =
    [PROGRAM_NAME.addr(), PROGRAM_VERSION.addr(), BUILD_TYPE.addr()];

/// This is the name of our program
static PROGRAM_NAME: rp_binary_info::entry::IdAndString =
    rp_binary_info::program_name(concat!("Pico W Neopixel Driver", "\0"));

/// This is the version of our program
static PROGRAM_VERSION: rp_binary_info::entry::IdAndString =
    rp_binary_info::version(concat!(env!("GIT_VERSION"), "\0"));

static BUILD_TYPE: rp_binary_info::entry::IdAndString = rp_binary_info::custom_string(
    TAG_RASPBERRY_PI,
    ID_RP_PROGRAM_BUILD_ATTRIBUTE,
    concat!(env!("PROFILE"), "\0"),
);
