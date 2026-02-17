/*
 * OIP Comms Shell - Standalone CLI for communicating with PLCs and OPC UA servers
 * Uses libplctag and open62541 directly, independent of Godot.
 *
 * Supported protocols: ab_eip, modbus_tcp, opc_ua
 */

#include "libplctag.h"
#include "open62541.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>

// --- Data structures (mirrors OIPComms internals) ---

struct PlcTag {
    int32_t tag_pointer = -1;
    int elem_count = 1;
    bool initialized = false;
};

struct OpcUaTag {
    UA_NodeId node_id;
    UA_Variant value;
    bool initialized = false;
};

struct TagGroup {
    std::string name;
    std::string protocol;
    std::string gateway;
    std::string path;
    std::string cpu;
    int polling_interval = 1000;

    // PLC tags
    std::map<std::string, PlcTag> plc_tags;

    // OPC UA
    UA_Client *client = nullptr;
    std::map<std::string, OpcUaTag> opc_ua_tags;
};

static std::map<std::string, TagGroup> tag_groups;
static const int TIMEOUT = 5000;

// --- Helper functions ---

static std::vector<std::string> split(const std::string &s) {
    std::vector<std::string> tokens;
    std::istringstream iss(s);
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

static void print_help() {
    std::cout << "\n=== OIP Comms Shell ===\n"
              << "Commands:\n"
              << "  connect <name> <protocol> <gateway> <path> <cpu>\n"
              << "      Create a tag group / connection.\n"
              << "      Protocols: ab_eip, modbus_tcp, opc_ua\n"
              << "      Examples:\n"
              << "        connect myplc ab_eip 192.168.1.200 1,0 ControlLogix\n"
              << "        connect myopc opc_ua opc.tcp://192.168.56.104:62541 1 \"\"\n"
              << "\n"
              << "  tag <group> <tag_name> [elem_count]\n"
              << "      Register a tag in a group. elem_count defaults to 1.\n"
              << "      For OPC UA, tag_name is the node ID (numeric or string).\n"
              << "      Examples:\n"
              << "        tag myplc MyDINT 1\n"
              << "        tag myopc the.temperature.value\n"
              << "\n"
              << "  read <group> <tag_name> [type]\n"
              << "      Read a tag value. Types: int32 (default), int16, int8,\n"
              << "      uint32, uint16, uint8, int64, uint64, float32, float64, bit\n"
              << "\n"
              << "  write <group> <tag_name> <type> <value>\n"
              << "      Write a value to a tag.\n"
              << "\n"
              << "  poll <group> <tag_name> [type] [interval_ms]\n"
              << "      Continuously read a tag. Press Enter to stop.\n"
              << "\n"
              << "  list\n"
              << "      List all tag groups and their tags.\n"
              << "\n"
              << "  disconnect <group>\n"
              << "      Disconnect and remove a tag group.\n"
              << "\n"
              << "  help\n"
              << "      Show this help message.\n"
              << "\n"
              << "  quit / exit\n"
              << "      Exit the shell.\n"
              << std::endl;
}

// --- PLC (libplctag) functions ---

static bool plc_connect_tag(TagGroup &group, const std::string &tag_name) {
    PlcTag &tag = group.plc_tags[tag_name];

    std::string conn_str = "protocol=" + group.protocol
        + "&gateway=" + group.gateway
        + "&path=" + group.path
        + "&cpu=" + group.cpu
        + "&elem_count=" + std::to_string(tag.elem_count)
        + "&name=" + tag_name;

    tag.tag_pointer = plc_tag_create(conn_str.c_str(), TIMEOUT);

    if (tag.tag_pointer < 0) {
        std::cerr << "Error: Failed to create PLC tag '" << tag_name
                  << "' (error: " << plc_tag_decode_error(tag.tag_pointer) << ")" << std::endl;
        return false;
    }

    tag.initialized = true;
    std::cout << "PLC tag '" << tag_name << "' connected (handle=" << tag.tag_pointer << ")" << std::endl;
    return true;
}

static bool plc_read_tag(PlcTag &tag) {
    int rc = plc_tag_read(tag.tag_pointer, TIMEOUT);
    if (rc != PLCTAG_STATUS_OK) {
        std::cerr << "Error: Failed to read PLC tag (error: " << plc_tag_decode_error(rc) << ")" << std::endl;
        return false;
    }
    return true;
}

static void plc_print_value(PlcTag &tag, const std::string &type) {
    if (type == "bit")         std::cout << (plc_tag_get_bit(tag.tag_pointer, 0) ? "true" : "false");
    else if (type == "int8")   std::cout << (int)plc_tag_get_int8(tag.tag_pointer, 0);
    else if (type == "uint8")  std::cout << (int)plc_tag_get_uint8(tag.tag_pointer, 0);
    else if (type == "int16")  std::cout << plc_tag_get_int16(tag.tag_pointer, 0);
    else if (type == "uint16") std::cout << plc_tag_get_uint16(tag.tag_pointer, 0);
    else if (type == "int32")  std::cout << plc_tag_get_int32(tag.tag_pointer, 0);
    else if (type == "uint32") std::cout << plc_tag_get_uint32(tag.tag_pointer, 0);
    else if (type == "int64")  std::cout << plc_tag_get_int64(tag.tag_pointer, 0);
    else if (type == "uint64") std::cout << plc_tag_get_uint64(tag.tag_pointer, 0);
    else if (type == "float32") std::cout << plc_tag_get_float32(tag.tag_pointer, 0);
    else if (type == "float64") std::cout << plc_tag_get_float64(tag.tag_pointer, 0);
    else std::cout << plc_tag_get_int32(tag.tag_pointer, 0);
}

static bool plc_write_value(PlcTag &tag, const std::string &type, const std::string &val_str) {
    if (type == "bit")         plc_tag_set_bit(tag.tag_pointer, 0, std::stoi(val_str));
    else if (type == "int8")   plc_tag_set_int8(tag.tag_pointer, 0, (int8_t)std::stoi(val_str));
    else if (type == "uint8")  plc_tag_set_uint8(tag.tag_pointer, 0, (uint8_t)std::stoi(val_str));
    else if (type == "int16")  plc_tag_set_int16(tag.tag_pointer, 0, (int16_t)std::stoi(val_str));
    else if (type == "uint16") plc_tag_set_uint16(tag.tag_pointer, 0, (uint16_t)std::stoi(val_str));
    else if (type == "int32")  plc_tag_set_int32(tag.tag_pointer, 0, (int32_t)std::stol(val_str));
    else if (type == "uint32") plc_tag_set_uint32(tag.tag_pointer, 0, (uint32_t)std::stoul(val_str));
    else if (type == "int64")  plc_tag_set_int64(tag.tag_pointer, 0, (int64_t)std::stoll(val_str));
    else if (type == "uint64") plc_tag_set_uint64(tag.tag_pointer, 0, (uint64_t)std::stoull(val_str));
    else if (type == "float32") plc_tag_set_float32(tag.tag_pointer, 0, std::stof(val_str));
    else if (type == "float64") plc_tag_set_float64(tag.tag_pointer, 0, std::stod(val_str));
    else {
        std::cerr << "Error: Unknown type '" << type << "'" << std::endl;
        return false;
    }

    int rc = plc_tag_write(tag.tag_pointer, TIMEOUT);
    if (rc != PLCTAG_STATUS_OK) {
        std::cerr << "Error: Failed to write PLC tag (error: " << plc_tag_decode_error(rc) << ")" << std::endl;
        return false;
    }
    return true;
}

// --- OPC UA functions ---

static bool opcua_connect(TagGroup &group) {
    if (group.client != nullptr) {
        UA_Client_delete(group.client);
        group.client = nullptr;
    }

    group.client = UA_Client_new();
    UA_ClientConfig *config = UA_Client_getConfig(group.client);
    UA_ClientConfig_setDefault(config);

    UA_StatusCode rc = UA_Client_connect(group.client, group.gateway.c_str());
    if (rc != UA_STATUSCODE_GOOD) {
        std::cerr << "Error: OPC UA connection failed: " << UA_StatusCode_name(rc) << std::endl;
        UA_Client_delete(group.client);
        group.client = nullptr;
        return false;
    }

    std::cout << "OPC UA connected to " << group.gateway << std::endl;
    return true;
}

static bool opcua_client_connected(TagGroup &group) {
    if (group.client == nullptr) return false;
    UA_StatusCode status;
    UA_Client_getState(group.client, nullptr, nullptr, &status);
    return status == UA_STATUSCODE_GOOD;
}

static bool opcua_init_tag(TagGroup &group, const std::string &tag_name) {
    OpcUaTag &tag = group.opc_ua_tags[tag_name];
    UA_Variant_init(&tag.value);

    UA_UInt16 ns = (UA_UInt16)std::stoi(group.path);

    // Check if tag_name is numeric
    bool is_numeric = true;
    for (char c : tag_name) {
        if (!isdigit(c)) { is_numeric = false; break; }
    }

    if (is_numeric) {
        tag.node_id = UA_NODEID_NUMERIC(ns, (UA_UInt32)std::stoul(tag_name));
    } else {
        tag.node_id = UA_NODEID_STRING_ALLOC(ns, tag_name.c_str());
    }

    tag.initialized = true;
    return true;
}

static bool opcua_read_tag(TagGroup &group, OpcUaTag &tag) {
    UA_StatusCode rc = UA_Client_readValueAttribute(group.client, tag.node_id, &(tag.value));
    if (rc != UA_STATUSCODE_GOOD) {
        std::cerr << "Error: OPC UA read failed: " << UA_StatusCode_name(rc) << std::endl;
        return false;
    }
    return true;
}

static void opcua_print_value(OpcUaTag &tag, const std::string &type) {
    UA_Variant &v = tag.value;

    // Auto-detect from OPC UA variant type if possible
    if (v.type == &UA_TYPES[UA_TYPES_BOOLEAN])       std::cout << (*(UA_Boolean *)v.data ? "true" : "false");
    else if (v.type == &UA_TYPES[UA_TYPES_SBYTE])     std::cout << (int)*(UA_SByte *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_BYTE])      std::cout << (int)*(UA_Byte *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_INT16])      std::cout << *(UA_Int16 *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_UINT16])     std::cout << *(UA_UInt16 *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_INT32])      std::cout << *(UA_Int32 *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_UINT32])     std::cout << *(UA_UInt32 *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_INT64])      std::cout << *(UA_Int64 *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_UINT64])     std::cout << *(UA_UInt64 *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_FLOAT])      std::cout << *(UA_Float *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_DOUBLE])     std::cout << *(UA_Double *)v.data;
    else if (v.type == &UA_TYPES[UA_TYPES_STRING]) {
        UA_String *s = (UA_String *)v.data;
        std::cout << std::string((char *)s->data, s->length);
    }
    else std::cout << "(unknown type: " << v.type->typeName << ")";
}

static bool opcua_write_value(TagGroup &group, OpcUaTag &tag, const std::string &type, const std::string &val_str) {
    UA_Variant newVal;
    UA_Variant_init(&newVal);
    UA_StatusCode rc;

    if (type == "bit") {
        UA_Boolean val = (UA_Boolean)std::stoi(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_BOOLEAN]);
    } else if (type == "int8") {
        UA_SByte val = (UA_SByte)std::stoi(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_SBYTE]);
    } else if (type == "uint8") {
        UA_Byte val = (UA_Byte)std::stoi(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_BYTE]);
    } else if (type == "int16") {
        UA_Int16 val = (UA_Int16)std::stoi(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_INT16]);
    } else if (type == "uint16") {
        UA_UInt16 val = (UA_UInt16)std::stoi(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_UINT16]);
    } else if (type == "int32") {
        UA_Int32 val = (UA_Int32)std::stol(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_INT32]);
    } else if (type == "uint32") {
        UA_UInt32 val = (UA_UInt32)std::stoul(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_UINT32]);
    } else if (type == "int64") {
        UA_Int64 val = (UA_Int64)std::stoll(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_INT64]);
    } else if (type == "uint64") {
        UA_UInt64 val = (UA_UInt64)std::stoull(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_UINT64]);
    } else if (type == "float32") {
        UA_Float val = std::stof(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_FLOAT]);
    } else if (type == "float64") {
        UA_Double val = std::stod(val_str);
        UA_Variant_setScalarCopy(&newVal, &val, &UA_TYPES[UA_TYPES_DOUBLE]);
    } else {
        std::cerr << "Error: Unknown type '" << type << "'" << std::endl;
        return false;
    }

    rc = UA_Client_writeValueAttribute(group.client, tag.node_id, &newVal);
    UA_Variant_clear(&newVal);

    if (rc != UA_STATUSCODE_GOOD) {
        std::cerr << "Error: OPC UA write failed: " << UA_StatusCode_name(rc) << std::endl;
        return false;
    }
    return true;
}

// --- Cleanup ---

static void cleanup_group(TagGroup &group) {
    if (group.protocol == "opc_ua") {
        for (auto &kv : group.opc_ua_tags) {
            UA_Variant_clear(&kv.second.value);
        }
        if (group.client != nullptr) {
            UA_Client_disconnect(group.client);
            UA_Client_delete(group.client);
            group.client = nullptr;
        }
    } else {
        for (auto &kv : group.plc_tags) {
            if (kv.second.tag_pointer >= 0)
                plc_tag_destroy(kv.second.tag_pointer);
        }
    }
}

static void cleanup_all() {
    for (auto &kv : tag_groups) {
        cleanup_group(kv.second);
    }
    tag_groups.clear();
}

// --- Command handlers ---

static void cmd_connect(const std::vector<std::string> &args) {
    // connect <name> <protocol> <gateway> <path> [cpu]
    if (args.size() < 5) {
        std::cerr << "Usage: connect <name> <protocol> <gateway> <path> [cpu]" << std::endl;
        return;
    }

    std::string name = args[1];
    std::string protocol = args[2];
    std::string gateway = args[3];
    std::string path = args[4];
    std::string cpu = args.size() > 5 ? args[5] : "";

    if (tag_groups.find(name) != tag_groups.end()) {
        std::cout << "Tag group '" << name << "' already exists. Disconnecting first." << std::endl;
        cleanup_group(tag_groups[name]);
        tag_groups.erase(name);
    }

    TagGroup group;
    group.name = name;
    group.protocol = protocol;
    group.gateway = gateway;
    group.path = path;
    group.cpu = cpu;

    if (protocol == "opc_ua") {
        if (!opcua_connect(group)) return;
    }

    tag_groups[name] = group;
    std::cout << "Tag group '" << name << "' created (" << protocol << " -> " << gateway << ")" << std::endl;
}

static void cmd_tag(const std::vector<std::string> &args) {
    // tag <group> <tag_name> [elem_count]
    if (args.size() < 3) {
        std::cerr << "Usage: tag <group> <tag_name> [elem_count]" << std::endl;
        return;
    }

    std::string group_name = args[1];
    std::string tag_name = args[2];
    int elem_count = args.size() > 3 ? std::stoi(args[3]) : 1;

    auto it = tag_groups.find(group_name);
    if (it == tag_groups.end()) {
        std::cerr << "Error: Tag group '" << group_name << "' not found." << std::endl;
        return;
    }

    TagGroup &group = it->second;

    if (group.protocol == "opc_ua") {
        if (!opcua_client_connected(group)) {
            std::cerr << "Error: OPC UA client not connected. Use 'connect' first." << std::endl;
            return;
        }
        OpcUaTag tag;
        tag.initialized = false;
        UA_Variant_init(&tag.value);
        tag.node_id = UA_NODEID_NULL;
        group.opc_ua_tags[tag_name] = tag;
        opcua_init_tag(group, tag_name);
        std::cout << "OPC UA tag '" << tag_name << "' registered in group '" << group_name << "'" << std::endl;
    } else {
        PlcTag tag;
        tag.elem_count = elem_count;
        group.plc_tags[tag_name] = tag;

        if (plc_connect_tag(group, tag_name)) {
            std::cout << "PLC tag '" << tag_name << "' registered and connected in group '" << group_name << "'" << std::endl;
        }
    }
}

static void cmd_read(const std::vector<std::string> &args) {
    // read <group> <tag_name> [type]
    if (args.size() < 3) {
        std::cerr << "Usage: read <group> <tag_name> [type]" << std::endl;
        return;
    }

    std::string group_name = args[1];
    std::string tag_name = args[2];
    std::string type = args.size() > 3 ? args[3] : "int32";

    auto it = tag_groups.find(group_name);
    if (it == tag_groups.end()) {
        std::cerr << "Error: Tag group '" << group_name << "' not found." << std::endl;
        return;
    }

    TagGroup &group = it->second;

    if (group.protocol == "opc_ua") {
        auto tag_it = group.opc_ua_tags.find(tag_name);
        if (tag_it == group.opc_ua_tags.end()) {
            std::cerr << "Error: Tag '" << tag_name << "' not found. Register it first with 'tag'." << std::endl;
            return;
        }
        OpcUaTag &tag = tag_it->second;
        if (!opcua_read_tag(group, tag)) return;
        std::cout << tag_name << " = ";
        opcua_print_value(tag, type);
        std::cout << std::endl;
    } else {
        auto tag_it = group.plc_tags.find(tag_name);
        if (tag_it == group.plc_tags.end()) {
            std::cerr << "Error: Tag '" << tag_name << "' not found. Register it first with 'tag'." << std::endl;
            return;
        }
        PlcTag &tag = tag_it->second;
        if (!tag.initialized) {
            std::cerr << "Error: Tag not initialized." << std::endl;
            return;
        }
        if (!plc_read_tag(tag)) return;
        std::cout << tag_name << " = ";
        plc_print_value(tag, type);
        std::cout << std::endl;
    }
}

static void cmd_write(const std::vector<std::string> &args) {
    // write <group> <tag_name> <type> <value>
    if (args.size() < 5) {
        std::cerr << "Usage: write <group> <tag_name> <type> <value>" << std::endl;
        return;
    }

    std::string group_name = args[1];
    std::string tag_name = args[2];
    std::string type = args[3];
    std::string value = args[4];

    auto it = tag_groups.find(group_name);
    if (it == tag_groups.end()) {
        std::cerr << "Error: Tag group '" << group_name << "' not found." << std::endl;
        return;
    }

    TagGroup &group = it->second;

    if (group.protocol == "opc_ua") {
        auto tag_it = group.opc_ua_tags.find(tag_name);
        if (tag_it == group.opc_ua_tags.end()) {
            std::cerr << "Error: Tag '" << tag_name << "' not found." << std::endl;
            return;
        }
        if (opcua_write_value(group, tag_it->second, type, value)) {
            std::cout << "Written " << value << " to " << tag_name << std::endl;
        }
    } else {
        auto tag_it = group.plc_tags.find(tag_name);
        if (tag_it == group.plc_tags.end()) {
            std::cerr << "Error: Tag '" << tag_name << "' not found." << std::endl;
            return;
        }
        PlcTag &tag = tag_it->second;
        if (!tag.initialized) {
            std::cerr << "Error: Tag not initialized." << std::endl;
            return;
        }
        if (plc_write_value(tag, type, value)) {
            std::cout << "Written " << value << " to " << tag_name << std::endl;
        }
    }
}

static void cmd_poll(const std::vector<std::string> &args) {
    // poll <group> <tag_name> [type] [interval_ms]
    if (args.size() < 3) {
        std::cerr << "Usage: poll <group> <tag_name> [type] [interval_ms]" << std::endl;
        return;
    }

    std::string group_name = args[1];
    std::string tag_name = args[2];
    std::string type = args.size() > 3 ? args[3] : "int32";
    int interval = args.size() > 4 ? std::stoi(args[4]) : 1000;

    auto it = tag_groups.find(group_name);
    if (it == tag_groups.end()) {
        std::cerr << "Error: Tag group '" << group_name << "' not found." << std::endl;
        return;
    }

    TagGroup &group = it->second;
    std::cout << "Polling '" << tag_name << "' every " << interval << "ms. Press Ctrl+C to stop.\n" << std::endl;

    while (true) {
        if (group.protocol == "opc_ua") {
            auto tag_it = group.opc_ua_tags.find(tag_name);
            if (tag_it == group.opc_ua_tags.end()) break;
            if (!opcua_read_tag(group, tag_it->second)) break;
            std::cout << "\r" << tag_name << " = ";
            opcua_print_value(tag_it->second, type);
            std::cout << "          " << std::flush;
        } else {
            auto tag_it = group.plc_tags.find(tag_name);
            if (tag_it == group.plc_tags.end()) break;
            if (!plc_read_tag(tag_it->second)) break;
            std::cout << "\r" << tag_name << " = ";
            plc_print_value(tag_it->second, type);
            std::cout << "          " << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
}

static void cmd_list(const std::vector<std::string> &) {
    if (tag_groups.empty()) {
        std::cout << "No tag groups registered." << std::endl;
        return;
    }

    for (auto &kv : tag_groups) {
        TagGroup &g = kv.second;
        std::cout << "[" << g.name << "] protocol=" << g.protocol
                  << " gateway=" << g.gateway
                  << " path=" << g.path;
        if (!g.cpu.empty()) std::cout << " cpu=" << g.cpu;
        std::cout << std::endl;

        if (g.protocol == "opc_ua") {
            for (auto &t : g.opc_ua_tags)
                std::cout << "  tag: " << t.first << (t.second.initialized ? " (initialized)" : " (pending)") << std::endl;
        } else {
            for (auto &t : g.plc_tags)
                std::cout << "  tag: " << t.first << " elem_count=" << t.second.elem_count
                          << (t.second.initialized ? " (initialized)" : " (pending)") << std::endl;
        }
    }
}

static void cmd_disconnect(const std::vector<std::string> &args) {
    if (args.size() < 2) {
        std::cerr << "Usage: disconnect <group>" << std::endl;
        return;
    }

    std::string name = args[1];
    auto it = tag_groups.find(name);
    if (it == tag_groups.end()) {
        std::cerr << "Error: Tag group '" << name << "' not found." << std::endl;
        return;
    }

    cleanup_group(it->second);
    tag_groups.erase(it);
    std::cout << "Disconnected tag group '" << name << "'" << std::endl;
}

// --- Main ---

int main(int argc, char *argv[]) {
    std::cout << "OIP Comms Shell v1.0" << std::endl;
    std::cout << "Standalone CLI for PLC (libplctag) and OPC UA (open62541) communication." << std::endl;
    std::cout << "Type 'help' for commands.\n" << std::endl;

    std::string line;
    while (true) {
        std::cout << "oip> ";
        if (!std::getline(std::cin, line)) break;

        auto args = split(line);
        if (args.empty()) continue;

        std::string cmd = args[0];

        if (cmd == "quit" || cmd == "exit") {
            break;
        } else if (cmd == "help") {
            print_help();
        } else if (cmd == "connect") {
            cmd_connect(args);
        } else if (cmd == "tag") {
            cmd_tag(args);
        } else if (cmd == "read") {
            cmd_read(args);
        } else if (cmd == "write") {
            cmd_write(args);
        } else if (cmd == "poll") {
            cmd_poll(args);
        } else if (cmd == "list") {
            cmd_list(args);
        } else if (cmd == "disconnect") {
            cmd_disconnect(args);
        } else {
            std::cerr << "Unknown command: " << cmd << ". Type 'help' for usage." << std::endl;
        }
    }

    cleanup_all();
    std::cout << "Goodbye." << std::endl;
    return 0;
}
