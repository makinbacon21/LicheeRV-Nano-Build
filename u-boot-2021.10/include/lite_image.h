/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * This is from the Android Project,
 * Repository: https://android.googlesource.com/platform/system/tools/mkbootimg
 * File: include/bootimg/bootimg.h
 * Commit: e55998a0f2b61b685d5eb4a486ca3a0c680b1a2f
 *
 * Copyright (C) 2007 The Android Open Source Project
 * Copyright (C) 2025 Thomas Makin
 */

#ifndef _LITE_IMAGE_H_
#define _LITE_IMAGE_H_

#include <linux/compiler.h>
#include <linux/types.h>

#define LITE_BOOT_MAGIC "MAGIC123"
#define LITE_BOOT_MAGIC_SIZE 8
#define LITE_BOOT_NAME_SIZE 16
#define LITE_BOOT_ARGS_SIZE 512
#define LITE_BOOT_EXTRA_ARGS_SIZE 1024

struct lite_img_hdr {
  /* Must be LITE_BOOT_MAGIC. */
  char magic[LITE_BOOT_MAGIC_SIZE];

  u32 kernel_size; /* size in bytes */
  u32 kernel_addr; /* physical load addr */

  u32 ramdisk_size; /* size in bytes */
  u32 ramdisk_addr; /* physical load addr */

  u32 tags_addr; /* physical addr for kernel tags */
  u32 page_size; /* flash page size we assume */

  char name[LITE_BOOT_NAME_SIZE]; /* asciiz product name */

  char cmdline[LITE_BOOT_ARGS_SIZE];

  u32 id[8]; /* timestamp / checksum / sha1 / etc */

  /* Supplemental command line data; kept here to maintain
   * binary compatibility with older versions of mkbootimg. */
  char extra_cmdline[LITE_BOOT_EXTRA_ARGS_SIZE];

  u32 header_size;

  u32 dtb_size; /* size in bytes for DTB image */
  u64 dtb_addr; /* physical load address for DTB image */
} __attribute__((packed));

/* The structure of the lite boot image is as follows:
 *
 * +---------------------+
 * | boot header         | 1 page
 * +---------------------+
 * | kernel              | n pages
 * +---------------------+
 * | ramdisk             | m pages
 * +---------------------+
 * | dtb                 | q pages
 * +---------------------+
 *
 * n = (kernel_size + page_size - 1) / page_size
 * m = (ramdisk_size + page_size - 1) / page_size
 * q = (dtb_size + page_size - 1) / page_size
 *
 * 0. all entities are page_size aligned in flash
 * 1. kernel, ramdisk and DTB are required (size != 0)
 * 2. load each element (kernel, ramdisk, dtb) at
 *    the specified physical address (kernel_addr, etc)
 * 3. prepare tags at tag_addr.  kernel_args[] is
 *    appended to the kernel commandline in the tags.
 * 4. r0 = 0, r1 = MACHINE_TYPE, r2 = tags_addr
 */

#endif
