#include "certified_params_validator.hpp"
#include <ctime>
#include <stdexcept>
#include <algorithm>
#include <cstdlib>

CertifiedParamsValidator::CertifiedParamsValidator(
    const std::string& config_path,
    const std::string& secret_path,
    rclcpp::Logger logger)
    : config_path_(config_path), secret_path_(secret_path), logger_(logger) {
}

bool CertifiedParamsValidator::loadAndValidate() {
    try {
        // Load YAML file
        YAML::Node config = YAML::LoadFile(config_path_);

        // Extract certification info
        if (!config["certification"]) {
            RCLCPP_ERROR(logger_, "No 'certification' section found in config file!");
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
            RCLCPP_ERROR(logger_, "No 'safety_limits' section found in config file!");
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
            RCLCPP_FATAL(logger_, "❌ TAMPERING DETECTED!");
            RCLCPP_FATAL(logger_, "  Expected hash: %s", cert_info_.hash.c_str());
            RCLCPP_FATAL(logger_, "  Current hash:  %s", current_hash.c_str());
            RCLCPP_FATAL(logger_, "  → Safety parameters have been MODIFIED without recertification!");
            return false;
        }

        // Validate authenticity with HMAC
        if (!cert["hmac"]) {
            RCLCPP_FATAL(logger_, "❌ AUTHENTICITY FIELD MISSING");
            RCLCPP_FATAL(logger_, "  Add 'hmac' entry to certification metadata");
            return false;
        }

        const std::string expected_hmac = cert["hmac"].as<std::string>();
        
        std::ifstream secret_file(secret_path_);
        if (!secret_file.is_open()) {
            RCLCPP_FATAL(logger_, "❌ AUTHENTICATION FAILED");
            RCLCPP_FATAL(logger_, "  Could not open secret file: %s", secret_path_.c_str());
            RCLCPP_FATAL(logger_, "  → Cannot authenticate certified parameters");
            return false;
        }

        std::string secret;
        std::getline(secret_file, secret);
        secret_file.close();

        if (secret.empty()) {
            RCLCPP_FATAL(logger_, "❌ AUTHENTICATION FAILED");
            RCLCPP_FATAL(logger_, "  Secret file is empty: %s", secret_path_.c_str());
            RCLCPP_FATAL(logger_, "  → Cannot authenticate certified parameters");
            return false;
        }

        std::string computed_hmac = computeHMAC(canonical_representation_, secret);

        if (expected_hmac.size() != computed_hmac.size() ||
            CRYPTO_memcmp(expected_hmac.data(), computed_hmac.data(), expected_hmac.size()) != 0) {
            RCLCPP_FATAL(logger_, "❌ AUTHENTICITY CHECK FAILED");
            RCLCPP_FATAL(logger_, "  Expected HMAC: %s", expected_hmac.c_str());
            RCLCPP_FATAL(logger_, "  Computed HMAC: %s", computed_hmac.c_str());
            RCLCPP_FATAL(logger_, "  → Possible unauthorized modification with recomputed hash");
            return false;
        }

        // Check expiration
        if (!isCertificationValid()) {
            RCLCPP_WARN(logger_, "⚠️  WARNING: Certification EXPIRED!");
            RCLCPP_WARN(logger_, "  Valid until: %s", cert_info_.valid_until.c_str());
            RCLCPP_WARN(logger_, "  → Recertification required!");
            return false;
        }

        RCLCPP_INFO(logger_, "✅ Validation SUCCESS");
        RCLCPP_INFO(logger_, "  Certificate ID: %s", cert_info_.certificate_id.c_str());
        RCLCPP_INFO(logger_, "  Certified by:   %s", cert_info_.certified_by.c_str());
        RCLCPP_INFO(logger_, "  Date:           %s", cert_info_.date.c_str());
        RCLCPP_INFO(logger_, "  Hash:           %s...", current_hash.substr(0, 16).c_str());
        RCLCPP_INFO(logger_, "  Parameters:     %zu certified", params_.size());

        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(logger_, "YAML error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error: %s", e.what());
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
        RCLCPP_WARN(logger_, "Failed to parse date: %s (using default +1 year expiration)", date_str.c_str());
        // Return far future to avoid false expiration
        return std::time(nullptr) + (365 * 24 * 3600);  // +1 year
    }
    
    return std::mktime(&tm);
}
