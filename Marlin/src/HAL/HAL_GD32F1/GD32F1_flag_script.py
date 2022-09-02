from __future__ import print_function
import sys

#dynamic build flags for generic compile options
if __name__ == "__main__":
  print("\r\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 747 dynamic build flags for generic compile options in main\r\n")
  args = " ".join([ "-std=gnu11",
                    "-std=gnu++11",
                    "-Os",
                    "-mcpu=cortex-m4",
                    "-mfpu=fpv4-sp-d16",
                    "-mfloat-abi=hard",
                    "-mthumb",

                    "-ffreestanding",
                    "-fsigned-char",
                    "-fno-move-loop-invariants",
                    "-fno-strict-aliasing",

                    "--specs=nano.specs",
                    "--specs=nosys.specs",

                    "-IMarlin/src/HAL/HAL_GD32F1",
                    "-Isnapmaker/lib/GD32F1/libraries/FreeRTOS1030",
                    "-Isnapmaker/lib/GD32F1/libraries/FreeRTOS1030/utility/include",

                    "-MMD",
                    "-MP",
                    "-DTARGET_GD32F1"
                  ])


  for i in range(1, len(sys.argv)):
    args += " " + sys.argv[i]

  print(args)

# extra script for linker options
else:
  print("\r\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 747 extra script for linker options\r\n")
  from SCons.Script import DefaultEnvironment
  env = DefaultEnvironment()
  env.Append(
      ARFLAGS=["rcs"],

      ASFLAGS=[ "-x",
                "assembler-with-cpp",
                "-mcpu=cortex-m4",
                "-mfloat-abi=hard"
              ],

      CFLAGS=[
              "-mthumb",
              "-mcpu=cortex-m4",
              "-mfloat-abi=hard",
              "-mfpu=fpv4-sp-d16"
            ],

      CXXFLAGS=[
          "-mthumb",
          "-mcpu=cortex-m4",
          "-mfloat-abi=hard",
          "-mfpu=fpv4-sp-d16",
          "-fabi-version=0",
          "-fno-use-cxa-atexit",
          "-fno-threadsafe-statics"
      ],

      LINKFLAGS=[
          "-Os",
          "-mthumb",
          "-mcpu=cortex-m4",
          "-mfloat-abi=hard",
          "-mfpu=fpv4-sp-d16",
          "-ffreestanding",
          "--specs=nano.specs",
          "--specs=nosys.specs",
          "-u_printf_float",
      ],
  )
