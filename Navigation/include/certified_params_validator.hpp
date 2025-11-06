#ifndef CERTIFIED_PARAMS_VALIDATOR_HPP
#define CERTIFIED_PARAMS_VALIDATOR_HPP

#include <string>
#include <map>
#include <vector>
#include <openssl/sha.h>
#include <openssl/hmac.h>
#include <openssl/crypto.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <optional>
#include <utility>

/**
 * @brief Validator for certified safety parameters
 * 
 * Ensures safety-critical parameters cannot be modified at runtime
 * and validates certification using SHA-256 cryptographic hash.
 * 
 * Compliance:
 * - ISO 13849-1 §5.2.2: Safety parameters must be protected
 * - IEC 62304 §5.1.1: Configuration management required
 * - FDA/CE-MDR: Changes must be traceable and authorized
 * 
 * @version 1.0.0
 * @date 2025-10-28
 */
class CertifiedParamsValidator {
public:
    /**
     * @brief Certified parameter structure
     */
    struct CertifiedParam {
        std::string name;
        double value;
        std::string unit;
        std::string standard_reference;  // e.g., "ISO 3691-4 §5.2.5"
    };

    /**
     * @brief Certification metadata
     */
    struct CertificationInfo {
        std::string hash;                // SHA-256 of all parameters
        std::string date;                // Certification date (ISO 8601)
        std::string certified_by;        // Authority (e.g., "TÜV Rheinland")
        std::string certificate_id;      // Unique certificate ID
        std::string valid_until;         // Expiration date
        std::string project_version;     // Software version certified
    };

    /**
     * @brief Constructor
     * @param config_path Path to certified parameters YAML file
     * @param secret_path Path to the file containing the HMAC secret key
     */
    explicit CertifiedParamsValidator(const std::string& config_path, const std::string& secret_path);

    /**
     * @brief Load and validate certified parameters from file
     * @return true if valid, false if tampered/corrupted
     */
    bool loadAndValidate();

    /**
     * @brief Get parameter value by name
     * @param name Parameter name
     * @return Parameter value, or throws if not found
     */
    double getParameter(const std::string& name) const;

    /**
     * @brief Get all certified parameters
     * @return Map of parameter name -> value
     */
    std::map<std::string, double> getAllParameters() const;

    /**
     * @brief Get certification information
     * @return Certification metadata
     */
    CertificationInfo getCertificationInfo() const;

    /**
     * @brief Check if certification is still valid (not expired)
     * @return true if valid, false if expired
     */
    bool isCertificationValid() const;

    /**
     * @brief Compute SHA-256 hash of current parameters
     * @return Hex string of hash
     */
    std::string computeCurrentHash() const;

    /**
     * @brief Get list of all parameter names
     * @return Vector of parameter names
     */
    std::vector<std::string> getParameterNames() const;

private:
    std::string config_path_;
    std::string secret_path_;
    std::map<std::string, CertifiedParam> params_;
    CertificationInfo cert_info_;
    std::string canonical_representation_;

    /**
     * @brief Compute SHA-256 hash
     * @param data Input data
     * @return Hex string of hash
     */
    std::string computeSHA256(const std::string& data) const;

    /**
     * @brief Compute HMAC-SHA256 using shared secret key
     * @param data Canonical parameter representation
     * @param key Shared secret key
     * @return Hex encoded HMAC
     */
    std::string computeHMAC(const std::string& data, const std::string& key) const;

    /**
     * @brief Build canonical parameter representation for hashing/signing
     * @return Canonical string (sorted key=value pairs)
     */
    std::string buildCanonicalRepresentation() const;

    /**
     * @brief Parse ISO 8601 date string
     * @param date_str Date string (e.g., "2025-10-28T14:30:00Z")
     * @return Unix timestamp
     */
    time_t parseISO8601(const std::string& date_str) const;
};

#endif // CERTIFIED_PARAMS_VALIDATOR_HPP
