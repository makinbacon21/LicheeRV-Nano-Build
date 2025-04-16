// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2011 Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 * Copyright (c) 2025 Thomas Makin
 */

#include <common.h>
#include <env.h>
#include <image.h>
#include <image-android-dt.h>
#include <lite_image.h>
#include <malloc.h>
#include <errno.h>
#include <asm/unaligned.h>
#include <mapmem.h>
#include <linux/libfdt.h>

#define LITE_IMAGE_DEFAULT_KERNEL_ADDR 0x10008000

static ulong lite_image_get_kernel_addr(const struct lite_img_hdr *hdr) {
  if (hdr->kernel_addr == LITE_IMAGE_DEFAULT_KERNEL_ADDR)
    return (ulong)hdr + hdr->page_size;

  /*
   * abootimg creates images where all load addresses are 0
   * and we need to fix them.
   */
  if (hdr->kernel_addr == 0 && hdr->ramdisk_addr == 0)
    return env_get_ulong("kernel_addr_r", 16, 0);

  return hdr->kernel_addr;
}

/**
 * lite_image_get_kernel() - processes kernel part of Lite boot images
 * @hdr:	Pointer to image header, which is at the start
 *			of the image.
 * @verify:	Checksum verification flag. Currently unimplemented.
 * @os_data:	Pointer to a ulong variable, will hold os data start
 *			address.
 * @os_len:	Pointer to a ulong variable, will hold os data length.
 *
 * This function returns the os image's start address and length. Also,
 * it appends the kernel command line to the bootargs env variable.
 *
 * Return: Zero, os start address and length on success,
 *		otherwise on failure.
 */
int lite_image_get_kernel(const struct lite_img_hdr *hdr, int verify,
                             ulong *os_data, ulong *os_len) {
  u32 kernel_addr = lite_image_get_kernel_addr(hdr);

  printf("Image name: %s\n", hdr->name);
  printf("Kernel load addr 0x%08x size %u KiB\n", kernel_addr,
    DIV_ROUND_UP(hdr->kernel_size, 1024));

  /**
   * TODO: BOOTARGS HANDLING
   * Arguments (config parameters etc.) can be passed to the OS once we boot.
   * We need to do the following:
   * - check if there are bootargs in the image header, if so, append to list
   * - use env_get to check ig the "bootargs" variable is defined, if so,
   *   append to list
   * - malloc enough legnth in a new char* for the bootargs + a trailing space
   *   (convention) + \0
   * - strcpy/strcat all bootargs into this new char*
   */

  /**
   * TODO: OS DATA HANDLING
   * We are expected to modify the target of the passed in os_data, os_len ptrs
   * We are assuming this is not a uImage, so the header with loading info will
   * be in the header itself
   *
   * os_data should point to start of the kernel--after the header page. You
   * should take the address/pointer of the header and add the page size to it,
   * page size can be found in the header
   *
   * os_len should be the full size of the kernel, which is stored in the header
   */

  return 0;
}

int lite_image_check_header(const struct lite_img_hdr *hdr) {
  /* TODO: use memcmp (like strcmp) to check the header magic vs the define in the header */
  return -1;
}

ulong lite_image_get_end(const struct lite_img_hdr *hdr) {
  ulong end;

  /*
   * The header takes a full page, the remaining components are aligned
   * on page boundary
   */
  end = (ulong)hdr;
  end += hdr->page_size;
  /**
   * TODO: END CALCULATION
   * We will add aligned sizes starting from tohe end of the header (1 page)
   * Aliigned in this case means each section size needs to be a multiple of the page sizse
   * To facilitate this we must use the ALIGN() macro function
   *
   * Example:
   * end += ALIGN(hdr->partition_size, hdr->page_size)
   */

  return end;
}

ulong lite_image_get_kload(const struct lite_img_hdr *hdr) {
  return lite_image_get_kernel_addr(hdr);
}

ulong lite_image_get_kcomp(const struct lite_img_hdr *hdr) {
  const void *p = (void *)((uintptr_t)hdr + hdr->page_size);

  if (get_unaligned_le32(p) == LZ4F_MAGIC)
    return IH_COMP_LZ4;
  else
    return image_decomp_type(p, sizeof(u32));
}

int lite_image_get_ramdisk(const struct lite_img_hdr *hdr, ulong *rd_data,
                              ulong *rd_len) {
  if (!hdr->ramdisk_size) {
    *rd_data = *rd_len = 0;
    return -1;
  }

  printf("RAM disk load addr 0x%08x size %u KiB\n", hdr->ramdisk_addr,
         DIV_ROUND_UP(hdr->ramdisk_size, 1024));

  *rd_data = (unsigned long)hdr;
  *rd_data += hdr->page_size;
  /**
   * TODO: RAMDISK IMAGE CALCULATION
   * Same as end calcuation except we want the start of the ramdisk section
   * instead of the end of the image--refer to the header for order and side
   */

  *rd_len = hdr->ramdisk_size;
  return 0;
}

/**
 * lite_image_get_dtb_img_addr() - Get the address of DTB area in boot image.
 * @hdr_addr: Boot image header address
 * @addr: Will contain the address of DTB area in boot image
 *
 * Return: true on success or false on fail.
 */
static bool lite_image_get_dtb_img_addr(ulong hdr_addr, ulong *addr) {
  const struct lite_img_hdr *hdr;
  ulong dtb_img_addr;
  bool ret = true;

  hdr = map_sysmem(hdr_addr, sizeof(*hdr));
  if (lite_image_check_header(hdr)) {
    printf("Error: Boot Image header is incorrect\n");
    ret = false;
    goto exit;
  }

  if (hdr->dtb_size == 0) {
    printf("Error: dtb_size is 0\n");
    ret = false;
    goto exit;
  }

  /* Calculate the address of DTB area in boot image */
  dtb_img_addr = hdr_addr;
  dtb_img_addr += hdr->page_size;
  /**
   * TODO: DTB IMAGE CALCULATION
   * Same as end calcuation except we want the start of the dtb section
   * instead of the end of the image--refer to the header for order and side
   */

  *addr = dtb_img_addr;

exit:
  unmap_sysmem(hdr);
  return ret;
}

#if !defined(CONFIG_SPL_BUILD)
/**
 * lite_print_contents - prints out the contents of the Lite format image
 * @hdr: pointer to the Lite format image header
 *
 * lite_print_contents() formats a multi line Lite image contents
 * description.
 * The routine prints out Lite image properties
 *
 * returns:
 *     no returned results
 */
void lite_print_contents(const struct lite_img_hdr *hdr) {
  const char *const p = IMAGE_INDENT_STRING;
  printf("%skernel size:          %x\n", p, hdr->kernel_size);
  printf("%skernel address:       %x\n", p, hdr->kernel_addr);
  printf("%sramdisk size:         %x\n", p, hdr->ramdisk_size);
  printf("%sramdisk address:      %x\n", p, hdr->ramdisk_addr);
  printf("%stags address:         %x\n", p, hdr->tags_addr);
  printf("%spage size:            %x\n", p, hdr->page_size);
  printf("%sname:                 %s\n", p, hdr->name);
  printf("%scmdline:              %s\n", p, hdr->cmdline);
  printf("%sheader size:          %x\n", p, hdr->header_size);

  printf("%sdtb size:             %x\n", p, hdr->dtb_size);
  printf("%sdtb addr:             %llx\n", p, hdr->dtb_addr);
}

/**
 * lite_image_print_dtb_info - Print info for one DTB blob in DTB area.
 * @fdt: DTB header
 * @index: Number of DTB blob in DTB area.
 *
 * Return: true on success or false on error.
 */
static bool lite_image_print_dtb_info(const struct fdt_header *fdt,
                                         u32 index) {
  int root_node_off;
  u32 fdt_size;
  const char *model;
  const char *compatible;

  root_node_off = fdt_path_offset(fdt, "/");
  if (root_node_off < 0) {
    printf("Error: Root node not found\n");
    return false;
  }

  fdt_size = fdt_totalsize(fdt);
  compatible = fdt_getprop(fdt, root_node_off, "compatible", NULL);
  model = fdt_getprop(fdt, root_node_off, "model", NULL);

  printf(" - DTB #%u:\n", index);
  printf("           (DTB)size = %d\n", fdt_size);
  printf("          (DTB)model = %s\n", model ? model : "(unknown)");
  printf("     (DTB)compatible = %s\n", compatible ? compatible : "(unknown)");

  return true;
}

/**
 * lite_image_print_dtb_contents() - Print info for DTB blobs in DTB area.
 * @hdr_addr: Boot image header address
 *
 * DTB payload in Lite Boot Image is a concatenated DTB blob
 *
 * This function does next:
 *   1. Prints out the format used in DTB area
 *   2. Iterates over all DTB blobs in DTB area and prints out the info for
 *      each blob.
 *
 * Return: true on success or false on error.
 */
bool lite_image_print_dtb_contents(ulong hdr_addr) {
  const struct lite_img_hdr *hdr;
  bool res;
  ulong dtb_img_addr; /* address of DTB part in boot image */
  u32 dtb_img_size;   /* size of DTB payload in boot image */
  ulong dtb_addr;     /* address of DTB blob with specified index  */
  u32 i;              /* index iterator */

  res = lite_image_get_dtb_img_addr(hdr_addr, &dtb_img_addr);
  if (!res)
    return false;

  printf("## DTB area contents:\n");

  /* Iterate over concatenated DTB blobs */
  hdr = map_sysmem(hdr_addr, sizeof(*hdr));
  dtb_img_size = hdr->dtb_size;
  unmap_sysmem(hdr);
  i = 0;
  dtb_addr = dtb_img_addr;
  while (dtb_addr < dtb_img_addr + dtb_img_size) {
    const struct fdt_header *fdt;
    u32 dtb_size;

    fdt = map_sysmem(dtb_addr, sizeof(*fdt));
    if (fdt_check_header(fdt) != 0) {
      unmap_sysmem(fdt);
      printf("Error: Invalid FDT header for index %u\n", i);
      return false;
    }

    res = lite_image_print_dtb_info(fdt, i);
    if (!res) {
      unmap_sysmem(fdt);
      return false;
    }

    dtb_size = fdt_totalsize(fdt);
    unmap_sysmem(fdt);
    dtb_addr += dtb_size;
    ++i;
  }

  return true;
}
#endif
