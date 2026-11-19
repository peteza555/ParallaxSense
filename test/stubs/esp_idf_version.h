#pragma once
#define ESP_IDF_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#ifndef IDF_MAJ
#define IDF_MAJ 5
#define IDF_MIN 5
#endif
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(IDF_MAJ, IDF_MIN, 0)
