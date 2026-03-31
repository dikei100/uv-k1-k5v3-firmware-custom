/* Copyright 2025 dikei100
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#ifndef APP_CAT_H
#define APP_CAT_H

#include <stdbool.h>
#include <stdint.h>

// Check if a byte looks like the start of a CAT command (uppercase A-Z).
// Used by the UART dispatcher to route between CAT and the binary protocol.
bool CAT_IsCATByte(uint8_t byte);

// Try to parse and handle a CAT command from the VCP RX buffer.
// Returns true if a complete command was found and processed.
bool CAT_ProcessVCP(void);

#endif
