#
# Snapmaker2-Modules Firmware
# Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
#
# This file is part of Snapmaker2-Modules
# (see https://github.com/Snapmaker/Snapmaker2-Modules)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import sys
import os
import re, datetime
from os.path import join
Import("env", "projenv")

project_dir = projenv.get("PROJECT_DIR")

PIOENV = projenv.get("PIOENV")
if PIOENV.endswith('_boot'):
  print("You are building bootloader, won't package bin for HMI")

project_dir = projenv.get("PROJECT_DIR")
release_dir = join(project_dir, "release")
if not os.path.exists(release_dir):
  os.mkdir(release_dir)

# get version from Marlin\src\inc\Version.h
with open(join(project_dir, 'Marlin', 'src', 'inc','Version.h'), 'r', encoding='utf-8') as version_file:
  lines = version_file.readlines()

version = None
pattern = r'J1_BUILD_VERSION "\d+\.\d+\.\d+'
for line in lines:
  match_obj = re.search(pattern, line, re.I)
  if match_obj:
    version = match_obj[0].split('"')[-1]
    version = "V" + version
    break

if not version:
  print("cannot get app version from Marlin\\src\\inc\\Version.h")
  print("won't use default version: V0.0.0-2201")
  version = "V0.0.0"
else:
  print("got app version: {}".format(version))

app_fw_bin = join(projenv.get("PROJECT_BUILD_DIR"), PIOENV, projenv.get("PROGNAME") + '.bin')

date = datetime.datetime.today().strftime('%Y%m%d')
minor_bin = join(release_dir, "J1_MC_APP_{}_{}.bin".format(version, date))
if os.path.exists(minor_bin):
  try:
    os.remove("{}.old".format(minor_bin))
  except Exception:
    pass
  os.rename(minor_bin, "{}.old".format(minor_bin))

print("app raw bin: {}".format(app_fw_bin))
print("min bin name: {}".format(minor_bin))
# tools\ota_python\gen_header.py
minor_pack_script = join(project_dir, 'snapmaker', 'scripts', 'gen_header.py')

major_pack_script = join(project_dir, 'snapmaker', 'scripts', 'pack_for_hmi.py')


# print(project_dir)
# print(pack_script)
# print(app_fw_bin)
# print(boot_fw_bin)

env.AddCustomTarget(
  name="pack",
  dependencies=None,
  actions=[
    "python {} -t 3 -f {} -c 1 -v {} -o {}".format(minor_pack_script, app_fw_bin, version, minor_bin),
    "python {} -c {} -v {}".format(major_pack_script, minor_bin, version)
  ],
  title="Pack",
  description="Pack major Firmware"
)
