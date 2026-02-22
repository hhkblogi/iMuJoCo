// mjc_quic_identity.swift
// TLS identity helper for QUIC transport.
// Uses an embedded PKCS12 identity (legacy 3DES encryption for iOS compat).

import Foundation
import Network
import Security
import os.log

private let logger = Logger(subsystem: "com.mujoco.video", category: "QUICIdentity")

// MARK: - QUIC TLS Identity

/// Provides a TLS identity for the QUIC transport server.
///
/// Uses an embedded self-signed PKCS12 certificate (ECDSA P-256, legacy 3DES
/// encryption for iOS compatibility). No file I/O, no Keychain, no bundle
/// lookup — the identity is decoded from a base64 literal at runtime.
enum MJQUICIdentity {

    /// ALPN protocol identifier for iMuJoCo video streams.
    static let alpn = "imujoco-video"

    /// PKCS12 password.
    private static let password = "imujoco"

    /// Embedded self-signed PKCS12 identity (ECDSA P-256, CN=iMuJoCo QUIC,
    /// 10-year validity, legacy PBE-SHA1-3DES encryption).
    /// Generated with:
    ///   openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 ...
    ///   openssl pkcs12 -export ... -certpbe PBE-SHA1-3DES -keypbe PBE-SHA1-3DES -macalg sha1
    private static let embeddedP12Base64 = """
    MIIDkgIBAzCCA1AGCSqGSIb3DQEHAaCCA0EEggM9MIIDOTCCAi8GCSqGSIb3DQEHBqCCAiAw\
    ggIcAgEAMIICFQYJKoZIhvcNAQcBMBwGCiqGSIb3DQEMAQMwDgQI1AfEXFM5up0CAggAgIIB\
    6FK3etO6+UTeNv8X//TkAvdQEgC5/N8MocZDT9sEo6CioW69ZwJDvawJaBmzfGS73EF3n5YH\
    JT4ZOHurgMs+sWx9vJ6vlGTVHITL1rlNWBbsia09xrki0S26Mv44osku5jyIeOERL1B7eLc/\
    GG2aub2wX9CDx9SKcmNHpC1Fwz8sQLAFQiO+Ey3jKVuxJOfoh+vGkRcbLEDiy+oQXCKAgHt\
    C5qo9Cp6P7MsGnojypsXStEGFJ4drH30wPe2i5Q276/LddPcjakT1JNVBu67tonkt2slsqwy\
    +lYJUBXu4OOu6m3cWD7YFz/P4NUtyDnRoPWG9Gxx/k2Nq4kTIPrx44qvwy1/j3y1L0HZaOF\
    Mf6v9w6Cbt95cdI4JOVfgns3FevMCVcBXAUmQ1iRbAAGscRLEurx8UT6KRmAveS4X4EDaquw\
    /1VLWBJFA3BNPB5f+bBFql1yMTCVA9bNbeOs7jDe97eJcdw1nE3CpSIhv40hgzVHWNSmomRC\
    3hM8+oSQHYRLfmUkIhotNNnhYMGSW51d+2O8VEfJ82vKFV6PaVg3OsNLhyfydDyn3zGVJrA2\
    /rHhM0J0Y4iba6FB5jL5fit9T8lAfhAPClB7xqLybUPmmuNob0JHrf3DzEgEM9DDn8Ej14p4\
    jBIhmcMIIBAgYJKoZIhvcNAQcBoIH0BIHxMIHuMIHrBgsqhkiG9w0BDAoBAqCBtDCBsTAcBgoq\
    hkiG9w0BDAEDMA4ECHnna/1VEAfTAgIIAASBkKFCanx/bAMCP+EqwFhxuwYtTWmk97r9YVsO\
    pGFfJluREbctJVa5p0Wi+ANvnyd930S/T5p0hG1PYlm9ABUeTAEx5H9PH63bM6of1lun3mve\
    o4IcESxjvRGlrYPY3tDKZcWd36YSeLHIWfxTKOiNnmkuV2qTg5iAHSYbEZvL0Nv/tpVIGyK\
    Fpa+eO4hJTZBkjzElMCMGCSqGSIb3DQEJFTEWBBRv8bRDYKEzoqaA2E5kLUTMIFMhiTA5MCEw\
    CQYFKw4DAhoFAAQUiTREZyGfmc2KKgCKpMXIFKRCCtMEEFTGy0yHgJnnK7BpOVDC5dACAggA
    """

    // MARK: - Public API

    /// Create QUIC protocol options configured with a TLS identity for server use.
    /// Returns `nil` only if identity decoding fails.
    static func quicOptions() -> NWProtocolQUIC.Options? {
        guard let identity = loadEmbeddedIdentity() else {
            logger.error("Failed to load embedded TLS identity for QUIC")
            return nil
        }

        guard let secIdentity = sec_identity_create(identity) else {
            logger.error("sec_identity_create returned nil")
            return nil
        }

        let quicOptions = NWProtocolQUIC.Options()
        quicOptions.direction = .bidirectional

        sec_protocol_options_set_local_identity(quicOptions.securityProtocolOptions, secIdentity)
        sec_protocol_options_add_tls_application_protocol(
            quicOptions.securityProtocolOptions, alpn)

        return quicOptions
    }

    // MARK: - Private

    /// Decode the embedded base64 PKCS12 and import it via SecPKCS12Import.
    private static func loadEmbeddedIdentity() -> SecIdentity? {
        // Strip whitespace and decode base64
        let cleaned = embeddedP12Base64
            .replacingOccurrences(of: " ", with: "")
            .replacingOccurrences(of: "\n", with: "")
        guard let p12Data = Data(base64Encoded: cleaned), p12Data.count > 32 else {
            logger.error("Failed to decode embedded PKCS12 base64")
            return nil
        }

        logger.debug("Decoded embedded PKCS12: \(p12Data.count) bytes")

        let options: [String: Any] = [kSecImportExportPassphrase as String: password]
        var items: CFArray?
        let status = SecPKCS12Import(p12Data as CFData, options as CFDictionary, &items)

        guard status == errSecSuccess else {
            logger.error("SecPKCS12Import failed with status \(status)")
            return nil
        }

        guard let itemArray = items as? [[String: Any]],
              let first = itemArray.first,
              let identityRef = first[kSecImportItemIdentity as String] else {
            logger.error("SecPKCS12Import returned no identity items")
            return nil
        }

        // swiftlint:disable:next force_cast
        let identity = identityRef as! SecIdentity
        logger.info("Loaded embedded QUIC TLS identity")
        return identity
    }
}
