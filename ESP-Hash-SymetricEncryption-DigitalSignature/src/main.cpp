#include <Arduino.h>
#include "mbedtls/md.h"
#include "mbedtls/aes.h"

// ============================================================================
// 1. FUNCIÓN HASH: SHA-256 (Integridad)
// ============================================================================
void demo_sha256(const char* payload) {
    Serial.println("\n--- [1] FUNCION HASH: SHA-256 ---");
    Serial.print("Mensaje de entrada: \"");
    Serial.print(payload);
    Serial.println("\"");

    byte shaResult[32];
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
    mbedtls_md_starts(&ctx);
    mbedtls_md_update(&ctx, (const unsigned char*)payload, strlen(payload));
    mbedtls_md_finish(&ctx, shaResult);
    mbedtls_md_free(&ctx);

    Serial.print("Hash SHA-256 calculado (Hex): ");
    for (int i = 0; i < 32; i++) {
        char str[3];
        sprintf(str, "%02x", (int)shaResult[i]);
        Serial.print(str);
    }
    Serial.println();
}

// ============================================================================
// 2. CIFRADO SIMETRICO: AES-128-CBC (Confidencialidad)
// ============================================================================
void demo_aes_cbc() {
    Serial.println("\n--- [2] CIFRADO SIMETRICO: AES-128-CBC ---");
    
    // Clave secreta simétrica de 128 bits (16 bytes)
    const unsigned char key[16] = {
        0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
        0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
    };

    // Vector de Inicialización (IV) de 16 bytes
    unsigned char iv_enc[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
    };
    unsigned char iv_dec[16];
    memcpy(iv_dec, iv_enc, 16); // Copia para el descifrado

    // Mensaje en texto plano múltiplo de 16 bytes (32 bytes)
    const char* plainText = "Telemetria_ESP32_BloqueSeguro!!!";
    size_t length = strlen(plainText); // 32 caracteres

    unsigned char cipherTextOutput[32];
    unsigned char decipheredTextOutput[33];

    mbedtls_aes_context aes;

    // Cifrado
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, key, 128);
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, length, iv_enc, (const unsigned char*)plainText, cipherTextOutput);
    mbedtls_aes_free(&aes);

    Serial.print("Texto original: \"");
    Serial.print(plainText);
    Serial.println("\"");

    Serial.print("Texto cifrado (Hex): ");
    for (size_t i = 0; i < length; i++) {
        char str[3];
        sprintf(str, "%02x", (int)cipherTextOutput[i]);
        Serial.print(str);
    }
    Serial.println();

    // Descifrado
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, key, 128);
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, length, iv_dec, cipherTextOutput, decipheredTextOutput);
    mbedtls_aes_free(&aes);
    decipheredTextOutput[length] = '\0'; // Terminador nulo

    Serial.print("Texto descifrado: \"");
    Serial.print((char*)decipheredTextOutput);
    Serial.println("\"");
}

// ============================================================================
// 3. AUTENTICACION DE MENSAJES: HMAC-SHA256 (Autenticidad e Integridad)
// ============================================================================
void demo_hmac_sha256(const char* payload) {
    Serial.println("\n--- [3] AUTENTICACION FUERTE: HMAC-SHA256 ---");
    
    const char* secret_key = "ClaveSecretaCompartidaIoT";
    byte hmacResult[32];

    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1); // 1 habilita HMAC
    mbedtls_md_hmac_starts(&ctx, (const unsigned char*)secret_key, strlen(secret_key));
    mbedtls_md_hmac_update(&ctx, (const unsigned char*)payload, strlen(payload));
    mbedtls_md_hmac_finish(&ctx, hmacResult);
    mbedtls_md_free(&ctx);

    Serial.print("Mensaje autenticado: \"");
    Serial.print(payload);
    Serial.println("\"");

    Serial.print("Firma HMAC-SHA256 (Hex): ");
    for (int i = 0; i < 32; i++) {
        char str[3];
        sprintf(str, "%02x", (int)hmacResult[i]);
        Serial.print(str);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("==================================================");
    Serial.println("   DEMOSTRACION CRIPTOGRAFICA EN ESP-WROOM-32   ");
    Serial.println("==================================================");

    const char* sample_data = "SensorTemp=24.5C;Presion=1013hPa";

    demo_sha256(sample_data);
    demo_aes_cbc();
    demo_hmac_sha256(sample_data);

    Serial.println("\nPruebas finalizadas con exito.");
}

void loop() {
    // Código de ejecución única en setup()
}
