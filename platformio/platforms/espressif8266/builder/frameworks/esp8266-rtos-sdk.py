# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ESP8266 RTOS SDK

ESP8266 SDK based on FreeRTOS, a truly free professional grade RTOS for
microcontrollers

https://github.com/espressif/ESP8266_RTOS_SDK
"""

from os.path import isdir, join

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
platform = env.PioPlatform()

FRAMEWORK_DIR = platform.get_package_dir("framework-esp8266-rtos-sdk")
assert isdir(FRAMEWORK_DIR)

env.Prepend(
    CPPPATH=[
        join(FRAMEWORK_DIR, "include"),
        join(FRAMEWORK_DIR, "extra_include"),
        join(FRAMEWORK_DIR, "driver_lib", "include"),
        join(FRAMEWORK_DIR, "include", "espressif"),
        join(FRAMEWORK_DIR, "include", "lwip"),
        join(FRAMEWORK_DIR, "include", "lwip", "ipv4"),
        join(FRAMEWORK_DIR, "include", "lwip", "ipv6"),
        join(FRAMEWORK_DIR, "include", "nopoll"),
        join(FRAMEWORK_DIR, "include", "spiffs"),
        join(FRAMEWORK_DIR, "include", "ssl"),
        join(FRAMEWORK_DIR, "include", "json"),
        join(FRAMEWORK_DIR, "include", "openssl"),
    ],

    LIBPATH=[
        join(FRAMEWORK_DIR, "lib")
    ],

    LIBS=[
        "cirom", "crypto", "driver", "espconn", "espnow", "freertos", "gcc",
        "json", "hal", "lwip", "main", "mesh", "mirom", "net80211", "nopoll",
        "phy", "pp", "pwm", "smartconfig", "spiffs", "ssl", "wpa", "wps"
    ]
)

env.Replace(
    LDSCRIPT_PATH=[join(FRAMEWORK_DIR, "ld", "eagle.app.v6.ld")],
)

#
# Target: Build Driver Library
#

libs = []

libs.append(env.BuildLibrary(
    join(FRAMEWORK_DIR, "lib", "driver"),
    join(FRAMEWORK_DIR, "driver_lib")
))

env.Prepend(LIBS=libs)
