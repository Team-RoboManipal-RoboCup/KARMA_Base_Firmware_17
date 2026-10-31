/*
 * MIT License
 *
 * Copyright (c) 2010 Serge Zaitsev
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef JSMN_H
#define JSMN_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef JSMN_STATIC
#define JSMN_API static
#else
#define JSMN_API extern
#endif

typedef enum {
  JSMN_UNDEFINED = 0,
  JSMN_OBJECT = 1 << 0,
  JSMN_ARRAY = 1 << 1,
  JSMN_STRING = 1 << 2,
  JSMN_PRIMITIVE = 1 << 3
} jsmntype_t;

enum jsmnerr {
  /* Not enough tokens were provided */
  JSMN_ERROR_NOMEM = -1,
  /* Invalid character inside JSON string */
  JSMN_ERROR_INVAL = -2,
  /* The string is not a full JSON packet, more bytes expected */
  JSMN_ERROR_PART = -3
};

typedef struct jsmntok {
  jsmntype_t type;
  int start;
  int end;
  int size;
#ifdef JSMN_PARENT_LINKS
  int parent;
#endif
} jsmntok_t;

typedef struct jsmn_parser {
  unsigned int pos;     /* offset in the JSON string */
  unsigned int toknext; /* next token to allocate */
  int toksuper;         /* superior token node, e.g. parent object or array */
} jsmn_parser;

static inline void jsmn_init(jsmn_parser *parser) {
  parser->pos = 0;
  parser->toknext = 0;
  parser->toksuper = -1;
}

static inline jsmntok_t *jsmn_alloc_token(jsmn_parser *parser, jsmntok_t *tokens,
                                   const size_t num_tokens) {
  unsigned int cur = parser->toknext++;
  if (cur >= num_tokens) {
    return NULL;
  }
  tokens[cur].start = tokens[cur].end = -1;
  tokens[cur].size = 0;
#ifdef JSMN_PARENT_LINKS
  tokens[cur].parent = -1;
#endif
  return &tokens[cur];
}

static inline void jsmn_fill_token(jsmntok_t *token, const jsmntype_t type,
                            const int start, const int end) {
  token->type = type;
  token->start = start;
  token->end = end;
  token->size = 0;
}

static inline int jsmn_parse_primitive(jsmn_parser *parser, const char *js,
                                const size_t len, jsmntok_t *tokens,
                                const size_t num_tokens) {
  jsmntok_t *token;
  int start = parser->pos;

  for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
    switch (js[parser->pos]) {
    case ':':
    case '\t':
    case '\r':
    case '\n':
    case ' ':
    case ',':
    case ']':
    case '}':
      goto found;
    default:
      if (js[parser->pos] < 32 || js[parser->pos] >= 127) {
        parser->pos = start;
        return JSMN_ERROR_INVAL;
      }
    }
  }
found:
  if (tokens == NULL) {
    parser->pos--;
    return 0;
  }
  token = jsmn_alloc_token(parser, tokens, num_tokens);
  if (token == NULL) {
    parser->pos = start;
    return JSMN_ERROR_NOMEM;
  }
  jsmn_fill_token(token, JSMN_PRIMITIVE, start, parser->pos);
#ifdef JSMN_PARENT_LINKS
  token->parent = parser->toksuper;
#endif
  parser->pos--;
  return 0;
}

static inline int jsmn_parse_string(jsmn_parser *parser, const char *js,
                             const size_t len, jsmntok_t *tokens,
                             const size_t num_tokens) {
  jsmntok_t *token;
  int start = parser->pos;
  parser->pos++;

  for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
    char c = js[parser->pos];
    if (c == '\"') {
      if (tokens == NULL) {
        return 0;
      }
      token = jsmn_alloc_token(parser, tokens, num_tokens);
      if (token == NULL) {
        parser->pos = start;
        return JSMN_ERROR_NOMEM;
      }
      jsmn_fill_token(token, JSMN_STRING, start + 1, parser->pos);
#ifdef JSMN_PARENT_LINKS
      token->parent = parser->toksuper;
#endif
      return 0;
    }
    if (c == '\\' && parser->pos + 1 < len) {
      parser->pos++;
      switch (js[parser->pos]) {
      case '\"':
      case '/':
      case '\\':
      case 'b':
      case 'f':
      case 'r':
      case 'n':
      case 't':
        break;
      case 'u':
        parser->pos += 4;
        break;
      default:
        parser->pos = start;
        return JSMN_ERROR_INVAL;
      }
    }
  }
  parser->pos = start;
  return JSMN_ERROR_PART;
}

static inline int jsmn_parse(jsmn_parser *parser, const char *js, const size_t len,
                      jsmntok_t *tokens, const unsigned int num_tokens) {
  int count = parser->toknext;
  int r;

  for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
    char c = js[parser->pos];
    switch (c) {
    case '{':
    case '[':
      count++;
      if (tokens != NULL) {
        jsmntok_t *token = jsmn_alloc_token(parser, tokens, num_tokens);
        if (token == NULL) {
          return JSMN_ERROR_NOMEM;
        }
        if (parser->toksuper != -1) {
          tokens[parser->toksuper].size++;
#ifdef JSMN_PARENT_LINKS
          token->parent = parser->toksuper;
#endif
        }
        token->type = (c == '{' ? JSMN_OBJECT : JSMN_ARRAY);
        token->start = parser->pos;
        parser->toksuper = parser->toknext - 1;
      }
      break;
    case '}':
    case ']':
      if (tokens != NULL) {
        jsmntype_t type = (c == '}' ? JSMN_OBJECT : JSMN_ARRAY);
        int i;
        for (i = parser->toknext - 1; i >= 0; i--) {
          jsmntok_t *token = &tokens[i];
          if (token->start >= 0 && token->end < 0) {
            if (token->type != type) {
              return JSMN_ERROR_INVAL;
            }
            token->end = parser->pos + 1;
            parser->toksuper = -1;
            break;
          }
        }
        if (i == -1) {
          return JSMN_ERROR_INVAL;
        }
        for (; i >= 0; i--) {
          jsmntok_t *token = &tokens[i];
          if (token->start >= 0 && token->end < 0) {
            parser->toksuper = i;
            break;
          }
        }
      }
      break;
    case '\"':
      r = jsmn_parse_string(parser, js, len, tokens, num_tokens);
      if (r < 0) return r;
      count++;
      if (tokens != NULL && parser->toksuper != -1) {
        tokens[parser->toksuper].size++;
      }
      break;
    case '\t':
    case '\r':
    case '\n':
    case ' ':
    case ':':
    case ',':
      break;
    default:
      r = jsmn_parse_primitive(parser, js, len, tokens, num_tokens);
      if (r < 0) return r;
      count++;
      if (tokens != NULL && parser->toksuper != -1) {
        tokens[parser->toksuper].size++;
      }
      break;
    }
  }

  if (tokens != NULL) {
    for (int i = parser->toknext - 1; i >= 0; i--) {
      if (tokens[i].start >= 0 && tokens[i].end < 0) {
        return JSMN_ERROR_PART;
      }
    }
  }

  return count;
}

#ifdef __cplusplus
}
#endif

#endif /* JSMN_H */
