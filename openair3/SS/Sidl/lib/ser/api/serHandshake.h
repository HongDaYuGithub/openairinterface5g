/*
 * Copyright 2022 Sequans Communications.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "SIDL_Handshake.h"
#include "SidlCompiler.h"

SIDL_BEGIN_C_INTERFACE

int serHandshakeHandleFromSSEncClt(unsigned char* _buffer, size_t _size, size_t* _lidx, const struct AcpHandshake_Type* fromSS);

int serHandshakeHandleFromSSDecSrv(const unsigned char* _buffer, size_t _size, unsigned char* _arena, size_t _aSize, struct AcpHandshake_Type** fromSS);

void serHandshakeHandleFromSSFreeSrv(struct AcpHandshake_Type* fromSS);

int serHandshakeHandleToSSEncSrv(unsigned char* _buffer, size_t _size, size_t* _lidx, const struct AcpHandshakeRsp_Type* toSS);

int serHandshakeHandleToSSDecClt(const unsigned char* _buffer, size_t _size, unsigned char* _arena, size_t _aSize, struct AcpHandshakeRsp_Type** toSS);

void serHandshakeHandleToSSFreeClt(struct AcpHandshakeRsp_Type* toSS);

SIDL_END_C_INTERFACE
