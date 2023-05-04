import sys
import datetime
import argparse
import re
import shutil
import struct
import os
import ntpath


from os.path import join, dirname
from pathlib import Path

_VERSION = "1.1.0"

# type id in minor image
MINOR_IMAGE_TYPE_CONTROLLER   = 0
MINOR_IMAGE_TYPE_MODULE       = 1

# type id in major image
TYPE_MAIN_CONTROLLER = 0
TYPE_EXTERNAL_MODULE = 1
TYPE_SCREEN_MODULE = 2


def pack_minor_image(image_type, start_id, end_id, version, input, output):
    with open(input, 'rb') as f:
        raw_bin = f.read()

    date  = datetime.datetime.today().strftime('%Y%m%d')
    checksum = 0
    for c in raw_bin:
        checksum = (checksum + c) & 0xFFFFFFF
    print('checksum of raw binary: ', checksum)

    # Construct full version
    if image_type == MINOR_IMAGE_TYPE_CONTROLLER:
        full_version = 'Snapmaker_J1_{}'.format(version).encode('UTF-8')
    else:
        full_version = 'Snapmaker_J1_{}_{}'.format(version, date).encode('UTF-8')
    if len(full_version) >= 32:
        full_version = full_version[:32]
    else:
        full_version += b'\0' * (32 - len(full_version))
    full_version += b'\0'

    with open(output, 'wb') as f:
        # Header (2048)
        # - type (1 byte)
        # - start ID (2 bytes)
        # - end ID (2 bytes)
        # - Version (33 bytes)
        # - Reserved (2 bytes)
        # - Content Length (4 bytes)
        # - Checksum (4 bytes)
        f.write(struct.pack('<b', image_type))
        f.write(struct.pack('<h', start_id))  # start ID
        f.write(struct.pack('<h', end_id))  # end ID
        f.write(full_version)
        f.write(struct.pack('<h', 0)) # Reserved (2 bytes)
        f.write(struct.pack('<i', len(raw_bin)))
        f.write(struct.pack('<i', checksum))

        f.seek(2048)
        f.write(raw_bin)


def append_head(output, type_, offset, filename):
    file_path = os.path.join(os.getcwd(), filename)
    size = os.path.getsize(file_path)

    output.write(struct.pack('>b', type_))
    output.write(struct.pack('>i', offset))
    output.write(struct.pack('>i', size))

    return size


def append_body(output, filename):
    file_path = os.path.join(os.getcwd(), filename)

    with open(file_path, 'rb') as f:
        output.write(f.read())


def pack_major_image(controller=None, module=None, screen=None, version=None):
    count = 0
    date  = datetime.datetime.today().strftime('%Y%m%d')
    version_pattern = r"V\d+\.\d+\.\d+"

    if isinstance(module, str):
        print("module path: {}".format(module))
        count += 1
        cur_dir = ntpath.dirname(module)
        if version == None:
            version = re.search(version_pattern, module)[0]

    if isinstance(controller, str):
        print("controller path: {}".format(controller))
        count += 1
        cur_dir = ntpath.dirname(controller)
        if version == None:
            version = re.search(version_pattern, controller)[0]

    if isinstance(screen, str):
        print("screen path: {}".format(screen))
        count += 1
        cur_dir = ntpath.dirname(screen)
        if version == None:
            version = re.search(version_pattern, screen)[0]

    full_version = "J1_{}_{}".format(version, date)

    major_image = join(cur_dir, "{}.bin".format(full_version))
    if os.path.exists(major_image):
        try:
            os.remove("{}.old".format(major_image))
        except Exception:
            pass
        os.rename(major_image, "{}.old".format(major_image))

    full_version = full_version.encode('ASCII')

    if len(full_version) >= 31:
        full_version = full_version[:31]
    else:
        full_version += b'\0' * (31 - len(full_version))
    full_version += b'\0'

    if count == 0:
        raise RuntimeError("Please specify minor image to be packaged!")

    print("Major image: {}".format(major_image))
    header_size = 39 + 9 * count
    with open(major_image, 'wb') as f:
        # Header (39 bytes)
        # - Length (2 bytes)
        # - Version (32 bytes)
        # - Flag (4 bytes)
        # - Count (1 byte)
        f.write(struct.pack('>h', header_size))  # Length 2 Bytes

        f.write(full_version)  # version 32 bytes

        f.write(struct.pack('>i', 0))  # flag 4 bytes
        f.write(struct.pack('>b', count))  # count

        offset = header_size

        # Module Header (9 bytes for each module)
        if isinstance(module, str):
            offset += append_head(f, TYPE_EXTERNAL_MODULE, offset, module)
        if isinstance(controller, str):
            offset += append_head(f, TYPE_MAIN_CONTROLLER, offset, controller)
        if isinstance(screen, str):
            offset += append_head(f, TYPE_SCREEN_MODULE, offset, screen)

        # Module Body
        if isinstance(module, str):
            append_body(f, module)
        if isinstance(controller, str):
            append_body(f, controller)
        if isinstance(screen, str):
            append_body(f, screen)



def pack_minor_controller_image(raw_bin, source_dir, version):
    date = datetime.datetime.today().strftime('%Y%m%d')

    if version == None and source_dir != None:
        with open(join(source_dir, 'Marlin', 'src', 'inc','Version.h'), 'r', encoding='utf-8') as version_file:
            lines = version_file.readlines()

        pattern = r"SM2-\d+\.\d+\.\S+\n"
        for line in lines:
            match_obj = re.search(pattern, line, re.I)
            if match_obj:
                version = 'V' + match_obj[0][4:-2]
                break

    print("controller version: {}".format(version))
    if version == None:
        raise RuntimeError("No version specify!")

    image_name = "SM2_MC_APP_{}_{}.bin".format(version.upper(), date)
    print("minor controller image name: {}".format(image_name))

    if source_dir:
        release_dir = Path(join(source_dir, 'release'))
    else:
        release_dir = Path(join(dirname(raw_bin), 'release'))

    if not release_dir.exists() or not release_dir.is_dir():
        release_dir.mkdir()

    shutil.copy(raw_bin, release_dir)
    shutil.copy(join(dirname(raw_bin), "firmware.elf"), release_dir)

    image_path = release_dir.joinpath(image_name)
    if image_path.exists():
        if image_path.with_suffix('.bin.old').exists():
            os.remove(image_path.with_suffix('.bin.old'))
        os.rename(image_path, image_path.with_suffix('.bin.old'))

    pack_minor_image(MINOR_IMAGE_TYPE_CONTROLLER, 0, 0, version, raw_bin, str(image_path))

    return image_path


# TODO
def pack_factory_module_image(raw_app_bin, source_dir, serial_number, module_id, version):
    pass


def pack_minor_module_image(raw_bin, source_dir, version):
    date = datetime.datetime.today().strftime('%Y%m%d')

    if version == None and source_dir != None:
        with open(join(source_dir, 'Marlin', 'src', 'configuration.h'), 'r', encoding='utf-8') as version_file:
            lines = version_file.readlines()

        pattern = r"v\d+\.\d+\.\d+"
        for line in lines:
            match_obj = re.search(pattern, line, re.I)
            if match_obj:
                print("got version: {}".format(match_obj[0]))
                version = match_obj[0]
                break

    if version == None:
        raise RuntimeError("No version specify!")

    image_name = "SM2_EM_APP_{}_{}.bin".format(version.upper(), date)
    print("minor module image name: {}".format(image_name))

    if source_dir:
        release_dir = Path(join(source_dir, 'release'))
    else:
        release_dir = Path(join(dirname(raw_bin), 'release'))

    if not release_dir.exists() or not release_dir.is_dir():
        release_dir.mkdir()

    shutil.copy(raw_bin, release_dir)
    shutil.copy(join(dirname(raw_bin), "firmware.elf"), release_dir)

    image_path = release_dir.joinpath(image_name)
    if image_path.exists():
        if image_path.with_suffix('.bin.old').exists():
            os.remove(image_path.with_suffix('.bin.old'))
        os.rename(image_path, image_path.with_suffix('.bin.old'))

    pack_minor_image(MINOR_IMAGE_TYPE_MODULE, 0, 0, version, raw_bin, str(image_path))

    return image_path


# TODO
def get_minor_image_from_dir(directory):
    return {}


def main(argv=None):
    parser = argparse.ArgumentParser("Package firmware for Snapmaker 2.0 -- V{}".format(_VERSION))

    parser.add_argument('-d', '--dir',
                        help="specify top directory of project",
                        type=str,
                        default=None)

    parser.add_argument('-c', '--controller',
                        help="specify raw binary image of controller",
                        type=str,
                        default=None)

    parser.add_argument('-vc', '--version_controller',
                        help="specify version of minor image of controller",
                        type=str,
                        default=None)

    parser.add_argument('-m', '--module',
                        help="specify raw binary image of module",
                        type=str,
                        default=None)

    parser.add_argument('-f', '--factory',
                        help="generate image for factory, which contains the bootloader",
                        type=bool,
                        default=False)

    parser.add_argument('-M', '--module_id',
                        help="specify Module id for factory image",
                        type=int,
                        default=0)

    parser.add_argument('-sn', '--serial_number',
                        help="specify serial_number for factory image",
                        type=int,
                        default=1000)

    parser.add_argument('-vm', '--version_module',
                        help="specify version of minor image of module",
                        type=str,
                        default=None)

    parser.add_argument('-s', '--screen',
                        help="specify Application for screen",
                        type=str,
                        default=None)

    parser.add_argument('-a', '--all',
                        help="specify the directory which contains all minor images",
                        type=str,
                        default=None)

    parser.add_argument('-v', '--version',
                        help="specify version for major image",
                        type=str,
                        default=None)

    if argv != None:
        args = parser.parse_args(argv)
    else:
        args = parser.parse_args()

    pack_major_image(args.controller, args.module, args.screen, args.version)


if __name__ == "__main__":
    main()
