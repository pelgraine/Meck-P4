// TransportKeyStore.cpp -- P4 adaptation using mbedtls instead of Arduino SHA256

#include "TransportKeyStore.h"
#include "mbedtls/sha256.h"
#include "mbedtls/md.h"
#include <cstring>

uint16_t TransportKey::calcTransportCode(const mesh::Packet* packet) const {
  uint16_t code;
  uint8_t hmac[32];

  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, key, sizeof(key));
  uint8_t type = packet->getPayloadType();
  mbedtls_md_hmac_update(&ctx, &type, 1);
  mbedtls_md_hmac_update(&ctx, packet->payload, packet->payload_len);
  mbedtls_md_hmac_finish(&ctx, hmac);
  mbedtls_md_free(&ctx);

  memcpy(&code, hmac, 2);
  if (code == 0) {
    code++;
  } else if (code == 0xFFFF) {
    code--;
  }
  return code;
}

bool TransportKey::isNull() const {
  for (size_t i = 0; i < sizeof(key); i++) {
    if (key[i]) return false;
  }
  return true;
}

void TransportKeyStore::putCache(uint16_t id, const TransportKey& key) {
  if (num_cache < MAX_TKS_ENTRIES) {
    cache_ids[num_cache] = id;
    cache_keys[num_cache] = key;
    num_cache++;
  }
}

void TransportKeyStore::getAutoKeyFor(uint16_t id, const char* name, TransportKey& dest) {
  for (int i = 0; i < num_cache; i++) {
    if (cache_ids[i] == id) {
      dest = cache_keys[i];
      return;
    }
  }
  uint8_t hash[32];
  mbedtls_sha256((const uint8_t*)name, strlen(name), hash, 0);
  memcpy(dest.key, hash, sizeof(dest.key));
  putCache(id, dest);
}

int TransportKeyStore::loadKeysFor(uint16_t id, TransportKey keys[], int max_num) {
  int n = 0;
  for (int i = 0; i < num_cache && n < max_num; i++) {
    if (cache_ids[i] == id) {
      keys[n++] = cache_keys[i];
    }
  }
  return n;
}

bool TransportKeyStore::saveKeysFor(uint16_t id, const TransportKey keys[], int num) {
  invalidateCache();
  return false;
}

bool TransportKeyStore::removeKeys(uint16_t id) {
  invalidateCache();
  return false;
}

bool TransportKeyStore::clear() {
  invalidateCache();
  return false;
}