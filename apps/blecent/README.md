<!--
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#
-->


<img src="http://mynewt.apache.org/img/logo.svg" width="250" alt="Apache Mynewt">

## Overview

`apps/blecenthr` is an example central heart rate application. It can be run seperately or paired with bleprph to form an application.

## Split Image

Split applications allow the user to build the application content separate from the library content by splitting an application into two pieces:

* A "loader" which contains a separate application that can perform upgrades and manage split images
* A "split app" which contains the main application content and references the libraries in the loader by static linkage

See [split image architecture](http://mynewt.apache.org/latest/os/modules/split/split/) for the details of split image design.

## Contents

blecenthr waits for a button press, then starts blinking fast and scanning for nearby heart rate monitors. When it finds one it connects to it, subscribes to its heart rate characteristic (blinking slightly slower), and when it receives a heart rate blinks at that rate, begins blinking at that rate. You can hold the button on power up in order to move back to the loader in order to do firmware updates, get stats, configure, etc via newtmgr.

## Usage

```
pkg.deps:
    - "@apache-mynewt-core/boot/split"
    - "@apache-mynewt-core/boot/split_app"
    - "@apache-mynewt-core/mgmt/imgmgr"
    - "@apache-mynewt-core/sys/console/full"
    - "@apache-mynewt-core/sys/log/full"
```

```
syscfg.vals
    OS_MAIN_STACK_SIZE: 468
    BLE_MAX_CONNECTIONS: 1 #should already be set, but be explicit
    TIMER_1: 1 #debouncing

    BLECENT_LOADER_BUTTON_PIN: 'BUTTON_1'
    BLECENT_LOADER_BUTTON_PULL: 'HAL_GPIO_PULL_UP'
    BLECENT_HR_BUTTON_PIN: 'BUTTON_1'
    BLECENT_HR_BUTTON_PULL: 'HAL_GPIO_PULL_UP'
    BLECENT_HR_LED_PIN: 'LED_1'
    BLECENT_HR_LED_INVERTED: true

    # https://github.com/apache/mynewt-core/pull/257/files
    BLE_LL_CONN_INIT_MIN_WIN_OFFSET: 2
```

You can use blecenthr alone

```
targets/app
    app=@apache-mynewt-core/apps/blecenthr
    bsp=@apache-mynewt-core/hw/bsp/nrf52840pdk
    build_profile=optimized
```

Or you can use blecenthr as part of a split app by setting up your target.

```
targets/app
    app=@apache-mynewt-core/apps/blecenthr
    loader=@apache-mynewt-core/apps/bleprph
    bsp=@apache-mynewt-core/hw/bsp/nrf52840pdk
    build_profile=optimized
```



