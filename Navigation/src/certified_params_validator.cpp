#include "certified_params_validator.hpp"
#include <ctime>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <cstdlib>

CertifiedParamsValidator::CertifiedParamsValidator(const std::string& config_path,
    const std::string& secret_path)
    : config_path_(config_path), secret_path_(secret_path) {
}

bool CertifiedParamsValidator::loadAndValidate() {
    try {
        // Load YAML file
        YAML::Node config = YAML::LoadFile(config_path_);

        // Extract certification info
        if (!config["certification"]) {
            std::cerr << "[CertifiedParams] ERROR: No 'certification' section found!" << std::endl;
            return false;
        }

        auto cert = config["certification"];
        cert_info_.hash = cert["hash"].as<std::string>();
        cert_info_.date = cert["date"].as<std::string>();
        cert_info_.certified_by = cert["certified_by"].as<std::string>();
        cert_info_.certificate_id = cert["certificate_id"].as<std::string>();
        cert_info_.valid_until = cert["valid_until"].as<std::string>();
        cert_info_.project_version = cert["project_version"].as<std::string>();

        // Extract parameters
        if (!config["safety_limits"]) {
            std::cerr << "[CertifiedParams] ERROR: No 'safety_limits' section found!" << std::endl;
            return false;
        }

        auto limits = config["safety_limits"];
        for (const auto& param : limits) {
            std::string name = param.first.as<std::string>();
            
            CertifiedParam cp;
            cp.name = name;
            cp.value = param.second["value"].as<double>();
            cp.unit = param.second["unit"].as<std::string>();
            cp.standard_reference = param.second["standard"].as<std::string>();
            
            params_[name] = cp;
        }

        // Build canonical representation and compute current hash
        canonical_representation_ = buildCanonicalRepresentation();
        std::string current_hash = computeSHA256(canonical_representation_);

        // Validate hash
        if (current_hash != cert_info_.hash) {
            std::cerr << "[CertifiedParams] ❌ TAMPERING DETECTED!" << std::endl;
            std::cerr << "  Expected hash: " << cert_info_.hash << std::endl;
            std::cerr << "  Current hash:  " << current_hash << std::endl;
            std::cerr << "  → Safety parameters have been MODIFIED without recertification!" << std::endl;
            return false;
        }

        // Validate authenticity with HMAC
        if (!cert["hmac"]) {
            std::cerr << "[CertifiedParams] ❌ AUTHENTICITY FIELD MISSING" << std::endl;
            std::cerr << "  Add 'hmac' entry to certification metadata" << std::endl;
            return false;
        }

        const std::string expected_hmac = cert["hmac"].as<std::string>();
        
        std::ifstream secret_file(secret_path_);
        if (!secret_file.is_open()) {
            std::cerr << "[CertifiedParams] ❌ AUTHENTICATION FAILED" << std::endl;
            std::cerr << "  Could not open secret file: " << secret_path_ << std::endl;
            std::cerr << "  → Cannot authenticate certified parameters" << std::endl;
            return false;
        }

        std::string secret;
        std::getline(secret_file, secret);
        secret_file.close();

        if (secret.empty()) {
            std::cerr << "[CertifiedParams] ❌ AUTHENTICATION FAILED" << std::endl;
            std::cerr << "  Secret file is empty: " << secret_path_ << std::endl;
            std::cerr << "  → Cannot authenticate certified parameters" << std::endl;
            return false;
        }

        std::string computed_hmac = computeHMAC(canonical_representation_, secret);

        if (expected_hmac.size() != computed_hmac.size() ||
            CRYPTO_memcmp(expected_hmac.data(), computed_hmac.data(), expected_hmac.size()) != 0) {
            std::cerr << "[CertifiedParams] ❌ AUTHENTICITY CHECK FAILED" << std::endl;
            std::cerr << "  Expected HMAC: " << expected_hmac << std::endl;
            std::cerr << "  Computed HMAC: " << computed_hmac << std::endl;
            std::cerr << "  → Possible unauthorized modification with recomputed hash" << std::endl;
            return false;
        }

        // Check expiration
        if (!isCertificationValid()) {
            std::cerr << "[CertifiedParams] ⚠️  WARNING: Certification EXPIRED!" << std::endl;
            std::cerr << "  Valid until: " << cert_info_.valid_until << std::endl;
            std::cerr << "  → Recertification required!" << std::endl;
            return false;
        }

        std::cout << "[CertifiedParams] ✅ Validation SUCCESS" << std::endl;
        std::cout << "  Certificate ID: " << cert_info_.certificate_id << std::endl;
        std::cout << "  Certified by:   " << cert_info_.certified_by << std::endl;
        std::cout << "  Date:           " << cert_info_.date << std::endl;
        std::cout << "  Hash:           " << current_hash.substr(0, 16) << "..." << std::endl;
        std::cout << "  Parameters:     " << params_.size() << " certified" << std::endl;

        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "[CertifiedParams] YAML error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[CertifiedParams] Error: " << e.what() << std::endl;
        return false;
    }
}

double CertifiedParamsValidator::getParameter(const std::string& name) const {
    auto it = params_.find(name);
    if (it == params_.end()) {
        throw std::runtime_error("Parameter not found: " + name);
    }
    return it->second.value;
}

std::map<std::string, double> CertifiedParamsValidator::getAllParameters() const {
    std::map<std::string, double> result;
    for (const auto& pair : params_) {
        result[pair.first] = pair.second.value;
    }
    return result;
}

CertifiedParamsValidator::CertificationInfo 
CertifiedParamsValidator::getCertificationInfo() const {
    return cert_info_;
}

bool CertifiedParamsValidator::isCertificationValid() const {
    time_t now = std::time(nullptr);
    time_t expiry = parseISO8601(cert_info_.valid_until);
    return now < expiry;
}

std::string CertifiedParamsValidator::computeCurrentHash() const {
    if (canonical_representation_.empty()) {
        return computeSHA256(buildCanonicalRepresentation());
    }
    return computeSHA256(canonical_representation_);
}

std::vector<std::string> CertifiedParamsValidator::getParameterNames() const {
    std::vector<std::string> names;
    for (const auto& pair : params_) {
        names.push_back(pair.first);
    }
    return names;
}

std::string CertifiedParamsValidator::computeSHA256(const std::string& data) const {
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256_CTX sha256;
    SHA256_Init(&sha256);
    SHA256_Update(&sha256, data.c_str(), data.length());
    SHA256_Final(hash, &sha256);

    std::stringstream ss;
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
    }
    return ss.str();
}

std::string CertifiedParamsValidator::computeHMAC(const std::string& data, const std::string& key) const {
    unsigned int len = SHA256_DIGEST_LENGTH;
    unsigned char hmac_value[SHA256_DIGEST_LENGTH];

    HMAC(EVP_sha256(), key.data(), static_cast<int>(key.size()),
         reinterpret_cast<const unsigned char*>(data.data()), data.size(),
         hmac_value, &len);

    std::stringstream ss;
    for (unsigned int i = 0; i < len; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hmac_value[i]);
    }
    return ss.str();
}

std::string CertifiedParamsValidator::buildCanonicalRepresentation() const {
    std::vector<std::string> names;
    for (const auto& pair : params_) {
        names.push_back(pair.first);
    }
    std::sort(names.begin(), names.end());

    std::stringstream ss;
    for (const auto& name : names) {
        const auto& param = params_.at(name);
        ss << name << "=" << std::fixed << std::setprecision(6) << param.value << ";";
    }

    return ss.str();
}

time_t CertifiedParamsValidator::parseISO8601(const std::string& date_str) const {
    struct tm tm = {0};
    std::istringstream ss(date_str);
    ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    
    if (ss.fail()) {
        std::cerr << "[CertifiedParams] Warning: Failed to parse date: " << date_str << std::endl;
        // Return far future to avoid false expiration
        return std::time(nullptr) + (365 * 24 * 3600);  // +1 year
    }
    
    return std::mktime(&tm);
}
