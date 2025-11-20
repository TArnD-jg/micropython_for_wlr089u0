/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017-2018 Paul Sokolovsky
 * Copyright (c) 2018 Yonatan Goldschmidt
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 * TODO: Implement necessary changes according to Adding hardware features
 * https://github.com/micropython/micropython/wiki/ContributorGuidelines
 */

#include <hal_aes_sync.h>
#include "py/mpconfig.h"
#include <assert.h>
#include <string.h>
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mphal.h"
#include "py/runtime.h"



// This module implements crypto ciphers API, roughly following
// https://www.python.org/dev/peps/pep-0272/ . Exact implementation
// of PEP 272 can be made with a simple wrapper which adds all the
// needed boilerplate.
// values follow PEP 272

/*ASF4 needed to init HW*/
struct aes_sync_descriptor AES_HW_DESC;

enum
{
    UCRYPTOLIB_MODE_ECB = 1,
    UCRYPTOLIB_MODE_CBC = 2,
    UCRYPTOLIB_MODE_CTR = 6,
};

struct ctr_params
{
    // counter is the IV of the AES context.
    uint8_t buffer[16]; // in encrypted_counter
    // encrypted counter
    uint32_t nc_ofst;
};
struct aes_ctx_with_key
{
    uint8_t key[32];
    uint8_t keysize;
    unsigned char iv[16];
};
#define AES_CTX_IMPL struct aes_ctx_with_key

typedef struct _mp_obj_aes_t
{
    mp_obj_base_t base;
    AES_CTX_IMPL ctx;
    uint8_t block_mode : 6;
#define AES_KEYTYPE_NONE 0
#define AES_KEYTYPE_ENC 1
#define AES_KEYTYPE_DEC 2
    uint8_t key_type : 2;
    struct ctr_params ctr_params[]; // optional
} mp_obj_aes_t;

static inline bool is_ctr_mode(int block_mode)
{
    return block_mode == UCRYPTOLIB_MODE_CTR;
}

static inline struct ctr_params *ctr_params_from_aes(mp_obj_aes_t *o)
{
    return &o->ctr_params[0];
}

static void aes_initial_set_key_impl(AES_CTX_IMPL *ctx, const uint8_t *key, size_t keysize, const uint8_t iv[16])
{
    ctx->keysize = keysize;
    memcpy(ctx->key, key, keysize);

    if (NULL != iv)
    {
        memcpy(ctx->iv, iv, sizeof(ctx->iv));
    }
}

static void aes_final_set_key_impl(AES_CTX_IMPL *ctx, bool encrypt)
{
    // first, copy key aside
    uint8_t key[32];
    uint8_t keysize = ctx->keysize;
    memcpy(key, ctx->key, keysize);
}
static mp_obj_t cryptolib_aes_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 2, 3, false);

    const mp_int_t block_mode = mp_obj_get_int(args[1]);

    switch (block_mode)
    {
    case UCRYPTOLIB_MODE_ECB:
    case UCRYPTOLIB_MODE_CBC:
    case UCRYPTOLIB_MODE_CTR:
        break;

    default:
        mp_raise_ValueError(MP_ERROR_TEXT("mode"));
    }

    mp_obj_aes_t *o = mp_obj_malloc_var(mp_obj_aes_t, ctr_params, struct ctr_params, !!is_ctr_mode(block_mode), type);

    o->block_mode = block_mode;
    o->key_type = AES_KEYTYPE_NONE;

    mp_buffer_info_t keyinfo;
    mp_get_buffer_raise(args[0], &keyinfo, MP_BUFFER_READ);
    if (32 != keyinfo.len && 16 != keyinfo.len)
    {
        mp_raise_ValueError(MP_ERROR_TEXT("key"));
    }

    mp_buffer_info_t ivinfo;
    ivinfo.buf = NULL;
    if (n_args > 2 && args[2] != mp_const_none)
    {
        mp_get_buffer_raise(args[2], &ivinfo, MP_BUFFER_READ);

        if (16 != ivinfo.len)
        {
            mp_raise_ValueError(MP_ERROR_TEXT("IV"));
        }
    }
    else if (o->block_mode == UCRYPTOLIB_MODE_CBC || is_ctr_mode(o->block_mode))
    {
        mp_raise_ValueError(MP_ERROR_TEXT("IV"));
    }

    if (is_ctr_mode(block_mode))
    {
        ctr_params_from_aes(o)->nc_ofst = 0;
    }

    aes_initial_set_key_impl(&o->ctx, keyinfo.buf, keyinfo.len, ivinfo.buf);

    return MP_OBJ_FROM_PTR(o);
}

static mp_obj_t aes_process(size_t n_args, const mp_obj_t *args, bool encrypt)
{
    mp_obj_aes_t *self = MP_OBJ_TO_PTR(args[0]);

    mp_obj_t in_buf = args[1];
    mp_obj_t out_buf = MP_OBJ_NULL;
    if (n_args > 2)
    {
        out_buf = args[2];
    }

    mp_buffer_info_t in_bufinfo;
    mp_get_buffer_raise(in_buf, &in_bufinfo, MP_BUFFER_READ);

    if (!is_ctr_mode(self->block_mode) && in_bufinfo.len % 16 != 0)
    {
        mp_raise_ValueError(MP_ERROR_TEXT("blksize % 16"));
    }

    vstr_t vstr;
    mp_buffer_info_t out_bufinfo;
    uint8_t *out_buf_ptr;

    /* ASF4
    enum aes_action { AES_DECRYPT, AES_ENCRYPT };
    enum aes_keysize { AES_KEY_128, AES_KEY_192, AES_KEY_256 };
    */
    aes_sync_init(&AES_HW_DESC, AES);
    aes_sync_enable(&AES_HW_DESC);
    enum aes_action AES_action_ASF4 = AES_ENCRYPT;
    enum aes_keysize AES_keysize_ASF4 = AES_KEY_128;
    /*eASF4*/

    if (out_buf != MP_OBJ_NULL)
    {
        mp_get_buffer_raise(out_buf, &out_bufinfo, MP_BUFFER_WRITE);
        if (out_bufinfo.len < in_bufinfo.len)
        {
            mp_raise_ValueError(MP_ERROR_TEXT("output too small"));
        }
        out_buf_ptr = out_bufinfo.buf;
    }
    else
    {
        vstr_init_len(&vstr, in_bufinfo.len);
        out_buf_ptr = (uint8_t *)vstr.buf;
    }

    // Decide keysize AES128 or AES256, and if encrypt or decrypt needed
    if (AES_KEYTYPE_NONE == self->key_type)
    {
        self->key_type = encrypt ? AES_KEYTYPE_ENC : AES_KEYTYPE_DEC;
        const bool encrypt_mode = encrypt || is_ctr_mode(self->block_mode);
        // Fill buffer with key and keysize
        aes_final_set_key_impl(&self->ctx, encrypt_mode);
        // Decide key length
        if (self->ctx.keysize == 16)
        {
            AES_keysize_ASF4 = AES_KEY_128;
        }
        else if (self->ctx.keysize == 32)
        {
            AES_keysize_ASF4 = AES_KEY_256;
        }
        // Set encrypt key or decrypt key
        if (encrypt)
        {
            AES_action_ASF4 = AES_ENCRYPT;
            aes_sync_set_encrypt_key(&AES_HW_DESC, self->ctx.key, AES_keysize_ASF4);
        }
        else if (!encrypt)
        {
            AES_action_ASF4 = AES_DECRYPT;
            aes_sync_set_decrypt_key(&AES_HW_DESC, self->ctx.key, AES_keysize_ASF4);
        }
    }
    else
    {
        if ((encrypt && self->key_type == AES_KEYTYPE_DEC) ||
            (!encrypt && self->key_type == AES_KEYTYPE_ENC))
        {

            mp_raise_ValueError(MP_ERROR_TEXT("can't encrypt & decrypt"));
        }
    }

    switch (self->block_mode)
    {
    case UCRYPTOLIB_MODE_ECB:
    {
        uint8_t *in = in_bufinfo.buf, *out = out_buf_ptr;
        uint8_t *top = in + in_bufinfo.len;
        for (; in < top; in += 16, out += 16)
        {
            aes_sync_ecb_crypt(&AES_HW_DESC,
                               AES_action_ASF4,
                               in,
                               out);
        }
    }
    break;

    case UCRYPTOLIB_MODE_CBC:
    {

        aes_sync_cbc_crypt(&AES_HW_DESC,
                           AES_action_ASF4,
                           in_bufinfo.buf,
                           out_buf_ptr,
                           in_bufinfo.len,
                           self->ctx.iv);
    }
    break;

    case UCRYPTOLIB_MODE_CTR:
    {
        struct ctr_params *ctr = ctr_params_from_aes(self);
        aes_sync_ctr_crypt(&AES_HW_DESC,
                           in_bufinfo.buf,
                           out_buf_ptr,
                           in_bufinfo.len,
                           ctr->buffer,
                           self->ctx.iv,
                           &ctr->nc_ofst);
    }
    break;

    default:
        break;
    }

    // Deinit HW AES
    aes_sync_deinit(&AES_HW_DESC);

    if (out_buf != MP_OBJ_NULL)
    {
        return out_buf;
    }

    return mp_obj_new_bytes_from_vstr(&vstr);
}

static mp_obj_t cryptolib_aes_encrypt(size_t n_args, const mp_obj_t *args)
{
    return aes_process(n_args, args, true);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cryptolib_aes_encrypt_obj, 2, 3, cryptolib_aes_encrypt);

static mp_obj_t cryptolib_aes_decrypt(size_t n_args, const mp_obj_t *args)
{
    return aes_process(n_args, args, false);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cryptolib_aes_decrypt_obj, 2, 3, cryptolib_aes_decrypt);

static const mp_rom_map_elem_t cryptolib_aes_locals_dict_table[] = {
    {MP_ROM_QSTR(MP_QSTR_encrypt), MP_ROM_PTR(&cryptolib_aes_encrypt_obj)},
    {MP_ROM_QSTR(MP_QSTR_decrypt), MP_ROM_PTR(&cryptolib_aes_decrypt_obj)},
};
static MP_DEFINE_CONST_DICT(cryptolib_aes_locals_dict, cryptolib_aes_locals_dict_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    cryptolib_aes_type,
    MP_QSTR_aes,
    MP_TYPE_FLAG_NONE,
    make_new, cryptolib_aes_make_new,
    locals_dict, &cryptolib_aes_locals_dict);

static const mp_rom_map_elem_t mp_module_cryptolib_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_cryptolib)},
    {MP_ROM_QSTR(MP_QSTR_aes), MP_ROM_PTR(&cryptolib_aes_type)},
    {MP_ROM_QSTR(MP_QSTR_MODE_ECB), MP_ROM_INT(UCRYPTOLIB_MODE_ECB)},
    {MP_ROM_QSTR(MP_QSTR_MODE_CBC), MP_ROM_INT(UCRYPTOLIB_MODE_CBC)},
    {MP_ROM_QSTR(MP_QSTR_MODE_CTR), MP_ROM_INT(UCRYPTOLIB_MODE_CTR)},

};

static MP_DEFINE_CONST_DICT(mp_module_cryptolib_globals, mp_module_cryptolib_globals_table);

const mp_obj_module_t mp_module_cryptolib = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&mp_module_cryptolib_globals,
};

// This module should not be extensible (as it is not a CPython standard
// library nor is it necessary to override from the filesystem), however it
// has previously been known as `ucryptolib`, so by making it extensible the
// `ucryptolib` alias will continue to work.
MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_cryptolib, mp_module_cryptolib);
