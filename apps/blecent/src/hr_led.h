/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 * 
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_HR_LED_
#define H_HR_LED_

#include "blecent.h"

#ifdef __cplusplus
extern "C" {
#endif

void hr_led_init(void);

int hr_led_start();

int hr_led_should_connect(const struct ble_gap_disc_desc *disc);

void hr_led_read_write_subscribe(const struct peer *peer);

void hr_led_gap_event(struct ble_gap_event *event, void *arg);

void hr_led_stop();


#ifdef __cplusplus
}
#endif

#endif
