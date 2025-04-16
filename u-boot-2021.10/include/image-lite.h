/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2008 Semihalf
 *
 * (C) Copyright 2000-2005
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 ********************************************************************
 * NOTE: This header file defines an interface to U-Boot. Including
 * this (unmodified) header file in another file is considered normal
 * use of U-Boot, and does *not* fall under the heading of "derived
 * work".
 ********************************************************************
 */

#ifndef __IMAGE_LITE_H__
#define __IMAGE_LITE_H__

#include "compiler.h"
#include <asm/byteorder.h>
#include <stdbool.h>

/* Define this to avoid #ifdefs later on */
struct lmb;
struct fdt_region;

#ifdef USE_HOSTCC
#include <sys/types.h>

#define IMAGE_ENABLE_OF_LIBFDT 1
#define CONFIG_MD5
#define CONFIG_SHA1
#define CONFIG_SHA256
#define CONFIG_SHA384
#define CONFIG_SHA512

#define IMAGE_ENABLE_IGNORE 0
#define IMAGE_INDENT_STRING ""

#else

#include <asm/u-boot.h>
#include <command.h>
#include <linker_lists.h>
#include <lmb.h>

/* Take notice of the 'ignore' property for hashes */
#define IMAGE_ENABLE_IGNORE 1
#define IMAGE_INDENT_STRING "   "

#define IMAGE_ENABLE_OF_LIBFDT CONFIG_IS_ENABLED(OF_LIBFDT)

#endif /* USE_HOSTCC */

extern ulong image_load_addr; /* Default Load Address */
extern ulong image_save_addr; /* Default Save Address */
extern ulong image_save_size; /* Default Save Size */

/* An invalid size, meaning that the image size is not known */
#define IMAGE_SIZE_INVAL (-1UL)

enum ilh_category {
  ILH_TARGET,
  ILH_COMP,

  ILH_COUNT,
};

enum ilh_target {
    ILH_TARGET_RV64,

    ILH_TARGET_COUNT,
}

/*
 * Compression Types
 *
 * The following are exposed to uImage header.
 * New IDs *MUST* be appended at the end of the list and *NEVER*
 * inserted for backward compatibility.
 */
enum {
  ILH_COMP_NONE = 0, /*  No	 Compression Used	*/
  ILH_COMP_GZIP,     /* gzip	 Compression Used	*/

  ILH_COMP_COUNT,
};

#define ILH_MAGIC 0x27051956   /* Image Magic Number		*/
#define ILH_NMLEN 32           /* Image Name Length		*/

/* Reused from common.h */
#define ROUND(a, b) (((a) + (b) - 1) & ~((b) - 1))

/*
 * Lite format image header,
 * all data in network byte order (aka natural aka bigendian).
 */
typedef struct lite_image_header {
  uint32_t ilh_magic;         /* Image Header Magic Number	*/
  uint32_t ilh_size;          /* Image Data Size		*/
  uint32_t ilh_load;          /* Kernel Load  Address		*/
  uint32_t ilh_ep;            /* Entry Point Address		*/\
  uint8_t ilh_name[ILH_NMLEN]; /* Image Name		*/
  uint8_t ilh_target;         /* Target		*/
  uint8_t ilh_comp;           /* Compression Type		*/
} lite_image_header_t;

typedef struct lite_image_info {
  ulong start, end;             /* start/end of blob */
  ulong image_start, image_len; /* start of image within blob, len of image */
  ulong load;                   /* load addr for the image */
  uint8_t comp, target;         /* compression, type of image, os type */
  uint8_t arch;                 /* CPU architecture */
} lite_image_info_t;

/*
 * Legacy and FIT format headers used by do_bootm() and do_bootm_<os>()
 * routines.
 */
typedef struct bootm_headers {
  /*
   * Legacy os image header, if it is a multi component image
   * then boot_get_ramdisk() and get_fdt() will attempt to get
   * data from second and third component accordingly.
   */
  lite_image_header_t *legacy_hdr_os;     /* image header pointer */
  lite_image_header_t legacy_hdr_os_copy; /* header copy */
  ulong legacy_hdr_valid;

#ifndef USE_HOSTCC
  lite_image_info_t os; /* os image info */
  ulong ep;        /* entry point of OS */

  ulong rd_start, rd_end; /* ramdisk start/end */

  char *ft_addr; /* flat dev tree address */
  ulong ft_len;  /* length of flat device tree */

  ulong initrd_start;
  ulong initrd_end;
  ulong cmdline_start;
  ulong cmdline_end;
  struct bd_info *kbd;
#endif

  int verify; /* env_get("verify")[0] != 'n' */

#define BOOTM_STATE_START (0x00000001)
#define BOOTM_STATE_FINDOS (0x00000002)
#define BOOTM_STATE_FINDOTHER (0x00000004)
#define BOOTM_STATE_LOADOS (0x00000008)
#define BOOTM_STATE_RAMDISK (0x00000010)
#define BOOTM_STATE_FDT (0x00000020)
#define BOOTM_STATE_OS_CMDLINE (0x00000040)
#define BOOTM_STATE_OS_BD_T (0x00000080)
#define BOOTM_STATE_OS_PREP (0x00000100)
#define BOOTM_STATE_OS_FAKE_GO (0x00000200) /* 'Almost' run the OS */
#define BOOTM_STATE_OS_GO (0x00000400)
  int state;

#if defined(CONFIG_LMB) && !defined(USE_HOSTCC)
  struct lmb lmb; /* for memory mgmt */
#endif
} bootm_headers_t;

extern bootm_headers_t images;

/*
 * Some systems (for example LWMON) have very short watchdog periods;
 * we must make sure to split long operations like memmove() or
 * checksum calculations into reasonable chunks.
 */
#ifndef CHUNKSZ
#define CHUNKSZ (64 * 1024)
#endif

#ifndef CHUNKSZ_CRC32
#define CHUNKSZ_CRC32 (64 * 1024)
#endif

#ifndef CHUNKSZ_MD5
#define CHUNKSZ_MD5 (64 * 1024)
#endif

#ifndef CHUNKSZ_SHA1
#define CHUNKSZ_SHA1 (64 * 1024)
#endif

#define uimage_to_cpu(x) be32_to_cpu(x)
#define cpu_to_uimage(x) cpu_to_be32(x)

/*
 * Compression type and magic number mapping table.
 */
struct comp_magic_map {
  int comp_id;
  const char *name;
  unsigned char magic[2];
};

/*
 * get_table_entry_id() scans the translation table trying to find an
 * entry that matches the given short name. If a matching entry is
 * found, it's id is returned to the caller.
 */
int get_table_entry_id(const table_entry_t *table, const char *table_name,
                       const char *name);
/*
 * get_table_entry_name() scans the translation table trying to find
 * an entry that matches the given id. If a matching entry is found,
 * its long name is returned to the caller.
 */
char *get_table_entry_name(const table_entry_t *table, char *msg, int id);

const char *genimg_get_os_name(uint8_t os);

/**
 * genimg_get_os_short_name() - get the short name for an OS
 *
 * @param os	OS (ILH_OS_...)
 * @return OS short name, or "unknown" if unknown
 */
const char *genimg_get_os_short_name(uint8_t comp);

const char *genimg_get_arch_name(uint8_t arch);

/**
 * genimg_get_arch_short_name() - get the short name for an architecture
 *
 * @param arch	Architecture type (ILH_ARCH_...)
 * @return architecture short name, or "unknown" if unknown
 */
const char *genimg_get_arch_short_name(uint8_t arch);

const char *genimg_get_type_name(uint8_t type);

/**
 * genimg_get_type_short_name() - get the short name for an image type
 *
 * @param type	Image type (ILH_TYPE_...)
 * @return image short name, or "unknown" if unknown
 */
const char *genimg_get_type_short_name(uint8_t type);

const char *genimg_get_comp_name(uint8_t comp);

/**
 * genimg_get_comp_short_name() - get the short name for a compression method
 *
 * @param comp	compression method (ILH_COMP_...)
 * @return compression method short name, or "unknown" if unknown
 */
const char *genimg_get_comp_short_name(uint8_t comp);

/**
 * genimg_get_cat_name() - Get the name of an item in a category
 *
 * @category:	Category of item
 * @id:		Item ID
 * @return name of item, or "Unknown ..." if unknown
 */
const char *genimg_get_cat_name(enum ilh_category category, uint id);

/**
 * genimg_get_cat_short_name() - Get the short name of an item in a category
 *
 * @category:	Category of item
 * @id:		Item ID
 * @return short name of item, or "Unknown ..." if unknown
 */
const char *genimg_get_cat_short_name(enum ilh_category category, uint id);

/**
 * genimg_get_cat_count() - Get the number of items in a category
 *
 * @category:	Category to check
 * @return the number of items in the category (ILH_xxx_COUNT)
 */
int genimg_get_cat_count(enum ilh_category category);

/**
 * genimg_get_cat_desc() - Get the description of a category
 *
 * @category:	Category to check
 * @return the description of a category, e.g. "architecture". This
 * effectively converts the enum to a string.
 */
const char *genimg_get_cat_desc(enum ilh_category category);

/**
 * genimg_cat_has_id() - Check whether a category has an item
 *
 * @category:	Category to check
 * @id:		Item ID
 * @return true or false as to whether a category has an item
 */
bool genimg_cat_has_id(enum ilh_category category, uint id);

int genimg_get_os_id(const char *name);
int genimg_get_arch_id(const char *name);
int genimg_get_type_id(const char *name);
int genimg_get_comp_id(const char *name);
void genimg_print_size(uint32_t size);

#if defined(CONFIG_TIMESTAMP) || defined(CONFIG_CMD_DATE) || defined(USE_HOSTCC)
#define IMAGE_ENABLE_TIMESTAMP 1
#else
#define IMAGE_ENABLE_TIMESTAMP 0
#endif
void genimg_print_time(time_t timestamp);

#ifndef USE_HOSTCC
/* Image format types, returned by _get_format() routine */
#define IMAGE_FORMAT_INVALID 0x00
#if defined(CONFIG_LEGACY_IMAGE_FORMAT)
#define IMAGE_FORMAT_LEGACY 0x01 /* legacy image_LITE_header based format */
#endif
#define IMAGE_FORMAT_FIT 0x02     /* new, libfdt based format */
#define IMAGE_FORMAT_ANDROID 0x03 /* Android boot image */
#define IMAGE_FORMAT_LITE 0x4

ulong genimg_get_kernel_addr(char *const img_addr);
int genimg_get_format(const void *img_addr);
int genimg_has_config(bootm_headers_t *images);

int boot_get_fpga(int argc, char *const argv[], bootm_headers_t *images,
                  uint8_t arch, const ulong *ld_start, ulong *const ld_len);
int boot_get_ramdisk(int argc, char *const argv[], bootm_headers_t *images,
                     uint8_t arch, ulong *rd_start, ulong *rd_end);

/**
 * boot_get_loadable - routine to load a list of binaries to memory
 * @argc: Ignored Argument
 * @argv: Ignored Argument
 * @images: pointer to the bootm images structure
 * @arch: expected architecture for the image
 * @ld_start: Ignored Argument
 * @ld_len: Ignored Argument
 *
 * boot_get_loadable() will take the given FIT configuration, and look
 * for a field named "loadables".  Loadables, is a list of elements in
 * the FIT given as strings.  exe:
 *   loadables = "linux_kernel", "fdt-2";
 * this function will attempt to parse each string, and load the
 * corresponding element from the FIT into memory.  Once placed,
 * no aditional actions are taken.
 *
 * @return:
 *     0, if only valid images or no images are found
 *     error code, if an error occurs during fit_image_load
 */
int boot_get_loadable(int argc, char *const argv[], bootm_headers_t *images,
                      uint8_t arch, const ulong *ld_start, ulong *const ld_len);
#endif /* !USE_HOSTCC */

#ifndef USE_HOSTCC
int boot_get_fdt(int flag, int argc, char *const argv[], uint8_t arch,
                 bootm_headers_t *images, char **of_flat_tree, ulong *of_size);
void boot_fdt_add_mem_rsv_regions(struct lmb *lmb, void *fdt_blob);
int boot_relocate_fdt(struct lmb *lmb, char **of_flat_tree, ulong *of_size);

int boot_ramdisk_high(struct lmb *lmb, ulong rd_data, ulong rd_len,
                      ulong *initrd_start, ulong *initrd_end);
int boot_get_cmdline(struct lmb *lmb, ulong *cmd_start, ulong *cmd_end);
#ifdef CONFIG_SYS_BOOT_GET_KBD
int boot_get_kbd(struct lmb *lmb, struct bd_info **kbd);
#endif /* CONFIG_SYS_BOOT_GET_KBD */
#endif /* !USE_HOSTCC */

/*******************************************************************/
/* Legacy format specific code (prefixed with image_) */
/*******************************************************************/
static inline uint32_t image_get_header_size(void) {
  return (sizeof(lite_image_header_t));
}

#define image_get_hdr_l(f)                                                     \
  static inline uint32_t image_get_##f(const lite_image_header_t *hdr) {            \
    return uimage_to_cpu(hdr->ilh_##f);                                         \
  }
image_get_hdr_l(magic)    /* image_get_magic */
    image_get_hdr_l(hcrc) /* image_get_hcrc */
    image_get_hdr_l(time) /* image_get_time */
    image_get_hdr_l(size) /* image_get_size */
    image_get_hdr_l(load) /* image_get_load */
    image_get_hdr_l(ep)   /* image_get_ep */

#define image_get_hdr_b(f)                                                     \
  static inline uint8_t image_get_##f(const lite_image_header_t *hdr) {             \
    return hdr->ilh_##f;                                                        \
  }
    image_get_hdr_b(target)   /* image_get_target */
    image_get_hdr_b(comp) /* image_get_comp */

    static inline char *image_get_name(const lite_image_header_t *hdr) {
  return (char *)hdr->ilh_name;
}

static inline uint32_t image_get_data_size(const lite_image_header_t *hdr) {
  return image_get_size(hdr);
}

/**
 * image_get_data - get image payload start address
 * @hdr: image header
 *
 * image_get_data() returns address of the image payload. For single
 * component images it is image data start. For multi component
 * images it points to the null terminated table of sub-images sizes.
 *
 * returns:
 *     image payload data start address
 */
static inline ulong image_get_data(const lite_image_header_t *hdr) {
  return ((ulong)hdr + image_get_header_size());
}

static inline uint32_t image_get_image_size(const lite_image_header_t *hdr) {
  return (image_get_size(hdr) + image_get_header_size());
}
static inline ulong image_get_image_end(const lite_image_header_t *hdr) {
  return ((ulong)hdr + image_get_image_size(hdr));
}

#define image_set_hdr_l(f)                                                     \
  static inline void image_set_##f(lite_image_header_t *hdr, uint32_t val) {        \
    hdr->ilh_##f = cpu_to_uimage(val);                                          \
  }
image_set_hdr_l(magic)    /* image_set_magic */
    image_set_hdr_l(time) /* image_set_time */
    image_set_hdr_l(size) /* image_set_size */
    image_set_hdr_l(load) /* image_set_load */
    image_set_hdr_l(ep)   /* image_set_ep */

#define image_set_hdr_b(f)                                                     \
  static inline void image_set_##f(lite_image_header_t *hdr, uint8_t val) {         \
    hdr->ilh_##f = val;                                                         \
  }
    image_set_hdr_b(target)   /* image_set_target */
    image_set_hdr_b(comp) /* image_set_comp */

    static inline void image_set_name(lite_image_header_t *hdr, const char *name) {
  strncpy(image_get_name(hdr), name, ILH_NMLEN);
}

int image_check_hcrc(const lite_image_header_t *hdr);
int image_check_dcrc(const lite_image_header_t *hdr);
#ifndef USE_HOSTCC
ulong env_get_bootm_low(void);
phys_size_t env_get_bootm_size(void);
phys_size_t env_get_bootm_mapsize(void);
#endif
void memmove_wd(void *to, void *from, size_t len, ulong chunksz);

static inline int image_check_magic(const lite_image_header_t *hdr) {
  return (image_get_magic(hdr) == ILH_MAGIC);
}
static inline int image_check_target(const lite_image_header_t *hdr, uint8_t target) {
  return (image_get_target(hdr) == target);
}

ulong image_multi_count(const lite_image_header_t *hdr);
void image_multi_getimg(const lite_image_header_t *hdr, ulong idx, ulong *data,
                        ulong *len);

void image_print_contents(const void *hdr);

#ifndef USE_HOSTCC
static inline int image_check_target_arch(const lite_image_header_t *hdr) {
#ifndef ILH_ARCH_DEFAULT
#error "please define ILH_ARCH_DEFAULT in your arch asm/u-boot.h"
#endif
  return image_check_arch(hdr, ILH_ARCH_DEFAULT);
}
#endif /* USE_HOSTCC */

/**
 * image_decomp_type() - Find out compression type of an image
 *
 * @buf:	Address in U-Boot memory where image is loaded.
 * @len:	Length of the compressed image.
 * @return	compression type or ILH_COMP_NONE if not compressed.
 *
 * Note: Only following compression types are supported now.
 * lzo, lzma, gzip, bzip2
 */
int image_decomp_type(const unsigned char *buf, ulong len);

/**
 * image_decomp() - decompress an image
 *
 * @comp:	Compression algorithm that is used (ILH_COMP_...)
 * @load:	Destination load address in U-Boot memory
 * @image_start Image start address (where we are decompressing from)
 * @type:	OS type (ILH_OS_...)
 * @load_bug:	Place to decompress to
 * @image_buf:	Address to decompress from
 * @image_len:	Number of bytes in @image_buf to decompress
 * @unc_len:	Available space for decompression
 * @return 0 if OK, -ve on error (BOOTM_ERR_...)
 */
int image_decomp(int comp, ulong load, ulong image_start, int type,
                 void *load_buf, void *image_buf, ulong image_len, uint unc_len,
                 ulong *load_end);

/**
 * Set up properties in the FDT
 *
 * This sets up properties in the FDT that is to be passed to linux.
 *
 * @images:	Images information
 * @blob:	FDT to update
 * @of_size:	Size of the FDT
 * @lmb:	Points to logical memory block structure
 * @return 0 if ok, <0 on failure
 */
int image_setup_libfdt(bootm_headers_t *images, void *blob, int of_size,
                       struct lmb *lmb);

/**
 * Set up the FDT to use for booting a kernel
 *
 * This performs ramdisk setup, sets up the FDT if required, and adds
 * paramters to the FDT if libfdt is available.
 *
 * @param images	Images information
 * @return 0 if ok, <0 on failure
 */
int image_setup_linux(bootm_headers_t *images);

/*
 * At present we only support signing on the host, and verification on the
 * device
 */
#define IMAGE_ENABLE_SIGN 0
#define FIT_IMAGE_ENABLE_VERIFY 0

#define FDT_ERROR ((ulong)(-1))

ulong fdt_getprop_u32(const void *fdt, int node, const char *prop);

#endif /* __IMAGE_LITE_H__ */
