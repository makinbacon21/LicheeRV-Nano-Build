
#!/usr/bin/env python3
#
# Copyright 2015, The Android Open Source Project
# Copyright 2025, Thomas Makin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Creates the lite boot image."""
from argparse import (ArgumentParser, ArgumentTypeError,
                      FileType, RawDescriptionHelpFormatter)
from hashlib import sha1
from os import fstat
from struct import pack
# Constant and structure definition is in
# system/tools/mkbootimg/include/bootimg/bootimg.h
BOOT_MAGIC = 'MAGIC123'
BOOT_MAGIC_SIZE = 8
BOOT_NAME_SIZE = 16
BOOT_ARGS_SIZE = 512
BOOT_EXTRA_ARGS_SIZE = 1024
BOOT_IMAGE_HEADER_SIZE = 3152
PAGE_SIZE=2048

def filesize(f):
    if f is None:
        return 0
    try:
        return fstat(f.fileno()).st_size
    except OSError:
        return 0
def update_sha(sha, f):
    if f:
        sha.update(f.read())
        f.seek(0)
        sha.update(pack('I', filesize(f)))
    else:
        sha.update(pack('I', 0))
def pad_file(f, padding):
    pad = (padding - (f.tell() & (padding - 1))) & (padding - 1)
    f.write(pack(str(pad) + 'x'))
def get_number_of_pages(image_size, page_size):
    """calculates the number of pages required for the image"""
    return (image_size + page_size - 1) // page_size

def write_header(args):
    ramdisk_load_address = ((args.base + args.ramdisk_offset)
                            if filesize(args.ramdisk) > 0 else 0)
    args.output.write(pack(f'{BOOT_MAGIC_SIZE}s', BOOT_MAGIC.encode()))
    # kernel size in bytes
    args.output.write(pack('I', filesize(args.kernel)))
    # kernel physical load address
    args.output.write(pack('I', args.base + args.kernel_offset))
    # ramdisk size in bytes
    args.output.write(pack('I', filesize(args.ramdisk)))
    # ramdisk physical load address
    args.output.write(pack('I', ramdisk_load_address))
    # kernel tags physical load address
    args.output.write(pack('I', args.base + args.tags_offset))
    # flash page size
    args.output.write(pack('I', PAGE_SIZE))
    # asciiz product name
    args.output.write(pack(f'{BOOT_NAME_SIZE}s', args.board))
    args.output.write(pack(f'{BOOT_ARGS_SIZE}s', args.cmdline))
    sha = sha1()
    update_sha(sha, args.kernel)
    update_sha(sha, args.ramdisk)
    update_sha(sha, args.dtb)
    img_id = pack('32s', sha.digest())
    args.output.write(img_id)
    args.output.write(pack(f'{BOOT_EXTRA_ARGS_SIZE}s', args.extra_cmdline))
    args.output.write(pack('I', BOOT_IMAGE_HEADER_SIZE))
    if filesize(args.dtb) == 0:
        raise ValueError('DTB image must not be empty.')
    # dtb size in bytes
    args.output.write(pack('I', filesize(args.dtb)))
    # dtb physical load address
    args.output.write(pack('Q', args.base + args.dtb_offset))
    pad_file(args.output, PAGE_SIZE)
    return img_id

class AsciizBytes:
    """Parses a string and encodes it as an asciiz bytes object.
    >>> AsciizBytes(bufsize=4)('foo')
    b'foo\\x00'
    >>> AsciizBytes(bufsize=4)('foob')
    Traceback (most recent call last):
        ...
    argparse.ArgumentTypeError: Encoded asciiz length exceeded: max 4, got 5
    """
    def __init__(self, bufsize):
        self.bufsize = bufsize
    def __call__(self, arg):
        arg_bytes = arg.encode() + b'\x00'
        if len(arg_bytes) > self.bufsize:
            raise ArgumentTypeError(
                'Encoded asciiz length exceeded: '
                f'max {self.bufsize}, got {len(arg_bytes)}')
        return arg_bytes

def write_padded_file(f_out, f_in, padding):
    if f_in is None:
        return
    f_out.write(f_in.read())
    pad_file(f_out, padding)
def parse_int(x):
    return int(x, 0)

def parse_cmdline():
    cmdline_size = BOOT_ARGS_SIZE + BOOT_EXTRA_ARGS_SIZE - 1
    parser = ArgumentParser(formatter_class=RawDescriptionHelpFormatter)
    parser.add_argument('--kernel', type=FileType('rb'),
                        help='path to the kernel')
    parser.add_argument('--ramdisk', type=FileType('rb'),
                        help='path to the ramdisk')
    parser.add_argument('--dtb', type=FileType('rb'), help='path to the dtb')
    parser.add_argument('--cmdline', type=AsciizBytes(bufsize=cmdline_size),
                        default='', help='kernel command line arguments')
    parser.add_argument('--base', type=parse_int, default=0x10000000,
                        help='base address')
    parser.add_argument('--kernel_offset', type=parse_int, default=0x00008000,
                        help='kernel offset')
    parser.add_argument('--ramdisk_offset', type=parse_int, default=0x01000000,
                        help='ramdisk offset')
    parser.add_argument('--dtb_offset', type=parse_int, default=0x01f00000,
                        help='dtb offset')
    parser.add_argument('--tags_offset', type=parse_int, default=0x00000100,
                        help='tags offset')
    parser.add_argument('--board', type=AsciizBytes(bufsize=BOOT_NAME_SIZE),
                        default='', help='board name')
    parser.add_argument('--id', action='store_true',
                        help='print the image ID on standard output')
    parser.add_argument('-o', '--output', type=FileType('wb'),
                        help='output file name')
    args, extra_args = parser.parse_known_args()
    if len(extra_args) > 0:
        raise ValueError(f'Unrecognized arguments: {extra_args}')
    args.extra_cmdline = args.cmdline[BOOT_ARGS_SIZE-1:]
    args.cmdline = args.cmdline[:BOOT_ARGS_SIZE-1] + b'\x00'
    assert len(args.cmdline) <= BOOT_ARGS_SIZE
    assert len(args.extra_cmdline) <= BOOT_EXTRA_ARGS_SIZE
    return args

def write_data(args, pagesize):
    write_padded_file(args.output, args.kernel, pagesize)
    write_padded_file(args.output, args.ramdisk, pagesize)
    write_padded_file(args.output, args.dtb, pagesize)
def main():
    args = parse_cmdline()
    if args.output is not None:
        img_id = write_header(args)
        write_data(args, PAGE_SIZE)
        if args.id and img_id is not None:
            print('0x' + ''.join(f'{octet:02x}' for octet in img_id))
if __name__ == '__main__':
    main()
