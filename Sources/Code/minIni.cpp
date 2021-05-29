/*  minIni - Multi-Platform INI file parser, suitable for embedded systems
 *
 *  These routines are in part based on the article "Multiplatform .INI Files"
 *  by Joseph J. Graf in the March 1994 issue of Dr. Dobb's Journal.
 *
 *  Copyright (c) CompuPhase, 2008-2020
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
 *  Version: $Id: minIni.c 53 2015-01-18 13:35:11Z thiadmer.riemersma@gmail.com $
 */

#include "minIni.h"

#include <assert.h>

#include <cstring>
#include <cstdio>
#include <cstdlib>

#define INI_BUFFERSIZE  512

#define sizearray(a)    (sizeof(a) / sizeof((a)[0]))

enum quote_option {
  QUOTE_NONE,
  QUOTE_ENQUOTE,
  QUOTE_DEQUOTE,
};

static char *skipleading(char *str)
{
  assert(str != nullptr);
  while ('\0' < *str && *str <= ' ')
    str++;
  return str;
}

static char *skiptrailing(char *str, const char *base)
{
  assert(str != nullptr);
  assert(base != nullptr);
  while (str > base && '\0' < *(str-1) && *(str-1) <= ' ')
    str--;
  return str;
}

static char *striptrailing(char *str)
{
  char *ptr = skiptrailing(std::strchr(str, '\0'), str);
  assert(ptr != nullptr);
  *ptr = '\0';
  return str;
}

static char *ini_strncpy(char *dest, const char *source, size_t maxlen, enum quote_option option)
{
  size_t d, s;

  assert(maxlen>0);
  assert(source != nullptr && dest != nullptr);
  assert((dest < source || (dest == source && option != QUOTE_ENQUOTE)) || dest > source + std::strlen(source));
  if (option == QUOTE_ENQUOTE && maxlen < 3)
    option = QUOTE_NONE;  /* cannot store two quotes and a terminating zero in less than 3 characters */

  switch (option) {
  case QUOTE_NONE:
    for (d = 0; d < maxlen - 1 && source[d] != '\0'; d++)
      dest[d] = source[d];
    assert(d < maxlen);
    dest[d] = '\0';
    break;
  case QUOTE_ENQUOTE:
    d = 0;
    dest[d++] = '"';
    for (s = 0; source[s] != '\0' && d < maxlen - 2; s++, d++) {
      if (source[s] == '"') {
        if (d >= maxlen - 3)
          break;  /* no space to store the escape character plus the one that follows it */
        dest[d++] = '\\';
      } /* if */
      dest[d] = source[s];
    } /* for */
    dest[d++] = '"';
    dest[d] = '\0';
    break;
  case QUOTE_DEQUOTE:
    for (d = s = 0; source[s] != '\0' && d < maxlen - 1; s++, d++) {
      if ((source[s] == '"' || source[s] == '\\') && source[s + 1] == '"')
        s++;
      dest[d] = source[s];
    } /* for */
    dest[d] = '\0';
    break;
  default:
    assert(0);
  } /* switch */

  return dest;
}

static char *cleanstring(char *string, enum quote_option *quotes)
{
  int isstring;
  char *ep;

  assert(string != nullptr);
  assert(quotes != nullptr);

  /* Remove a trailing comment */
  isstring = 0;
  for (ep = string; *ep != '\0' && ((*ep != ';' && *ep != '#') || isstring); ep++) {
    if (*ep == '"') {
      if (*(ep + 1) == '"')
        ep++;                 /* skip "" (both quotes) */
      else
        isstring = !isstring; /* single quote, toggle isstring */
    } else if (*ep == '\\' && *(ep + 1) == '"') {
      ep++;                   /* skip \" (both quotes */
    } /* if */
  } /* for */
  assert(ep != nullptr && (*ep == '\0' || *ep == ';' || *ep == '#'));
  *ep = '\0';                 /* terminate at a comment */
  striptrailing(string);
  /* Remove double quotes surrounding a value */
  *quotes = QUOTE_NONE;
  if (*string == '"' && (ep = std::strchr(string, '\0')) != nullptr && *(ep - 1) == '"') {
    string++;
    *--ep = '\0';
    *quotes = QUOTE_DEQUOTE;  /* this is a string, so remove escaped characters */
  } /* if */
  return string;
}

/** ini_browse()
 * \param Callback    a pointer to a function that will be called for every
 *                    setting in the INI file.
 * \param UserData    arbitrary data, which the function passes on the
 *                    \c Callback function
 * \param Filename    the name and full path of the .ini file to read from
 *
 * \return            1 on success, 0 on failure (INI file not found)
 *
 * \note              The \c Callback function must return 1 to continue
 *                    browsing through the INI file, or 0 to stop. Even when the
 *                    callback stops the browsing, this function will return 1
 *                    (for success).
 */
bool ini_browse(INI_CALLBACK Callback, void *UserData, const char *Filename)
{
  char LocalBuffer[INI_BUFFERSIZE];
  int lenSec, lenKey;
  enum quote_option quotes;
  std::FILE *fp;

  if (Callback == nullptr)
    return false;

  if (!(fp = std::fopen(Filename, "r")))
    return false;

  LocalBuffer[0] = '\0';   /* copy an empty section in the buffer */
  lenSec = std::strlen(LocalBuffer) + 1;
  for ( ;; ) {
    char *sp, *ep;
    if (!std::fgets(LocalBuffer + lenSec, INI_BUFFERSIZE - lenSec, fp))
      break;
    sp = skipleading(LocalBuffer + lenSec);
    /* ignore empty strings and comments */
    if (*sp == '\0' || *sp == ';' || *sp == '#')
      continue;
    /* see whether we reached a new section */
    ep = std::strrchr(sp, ']');
    if (*sp == '[' && ep != nullptr) {
      *ep = '\0';
      ini_strncpy(LocalBuffer, sp + 1, INI_BUFFERSIZE, QUOTE_NONE);
      lenSec = std::strlen(LocalBuffer) + 1;
      continue;
    } /* if */
    /* not a new section, test for a key/value pair */
    ep = std::strchr(sp, '=');    /* test for the equal sign or colon */
    if (ep == nullptr)
      ep = std::strchr(sp, ':');
    if (ep == nullptr)
      continue;               /* invalid line, ignore */
    *ep++ = '\0';             /* split the key from the value */
    striptrailing(sp);
    ini_strncpy(LocalBuffer + lenSec, sp, INI_BUFFERSIZE - lenSec, QUOTE_NONE);
    lenKey = std::strlen(LocalBuffer + lenSec) + 1;
    /* clean up the value */
    sp = skipleading(ep);
    sp = cleanstring(sp, &quotes);  /* Remove a trailing comment */
    ini_strncpy(LocalBuffer + lenSec + lenKey, sp, INI_BUFFERSIZE - lenSec - lenKey, quotes);
    /* call the callback */
    if (!Callback(LocalBuffer, LocalBuffer + lenSec, LocalBuffer + lenSec + lenKey, UserData))
      break;
  } /* for */

  std::fclose(fp);
  return true;
}
