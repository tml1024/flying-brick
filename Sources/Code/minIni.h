/*  minIni - Multi-Platform INI file parser, suitable for embedded systems
 *
 *  Copyright (c) CompuPhase, 2008-2017
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not
 *  use this file except in compliance with the License. You may obtain a copy
 *  of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 *  Version: $Id: minIni.h 53 2015-01-18 13:35:11Z thiadmer.riemersma@gmail.com $
 */
#ifndef MININI_H
#define MININI_H

typedef bool (*INI_CALLBACK)(const char *Section, const char *Key, const char *Value, void *UserData);
bool ini_browse(INI_CALLBACK Callback, void *UserData, const char *Filename);

#endif /* MININI_H */
